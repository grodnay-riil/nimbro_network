// UDP receiver node (ROS2)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_receiver.h"
#include "udp_packet.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <cstring>
#include <chrono>
#include <algorithm>

namespace nimbro_topic_transport
{

UDPReceiver::UDPReceiver()
 : Node("udp_receiver")
 , m_receivedBytesInStatsInterval(0)
 , m_expectedPacketsInStatsInterval(0)
 , m_missingPacketsInStatsInterval(0)
 , m_remoteAddrLen(0)
{
	this->declare_parameter<int>("port", 5050);
	this->declare_parameter<bool>("drop_repeated_msgs", true);
	this->declare_parameter<bool>("warn_drop_incomplete", true);
	this->declare_parameter<bool>("keep_compressed", false);
	this->declare_parameter<bool>("fec", false);
	this->declare_parameter<std::string>("label", "");
	this->declare_parameter<std::string>("topic_prefix", "");

	m_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int port = this->get_parameter("port").as_int();

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	RCLCPP_INFO(this->get_logger(), "Binding to :%d", port);

	if(bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not bind socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not set broadcast flag: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	m_dropRepeatedMessages = this->get_parameter("drop_repeated_msgs").as_bool();
	m_warnDropIncomplete = this->get_parameter("warn_drop_incomplete").as_bool();
	m_keepCompressed = this->get_parameter("keep_compressed").as_bool();
	m_fec = this->get_parameter("fec").as_bool();

#if !(WITH_OPENFEC)
	if(m_fec)
		throw std::runtime_error("Please compile with FEC support to enable FEC");
#endif

	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = this->get_name();
	m_stats.protocol = "UDP";
	m_stats.host = hostnameBuf;
	m_stats.local_port = port;
	m_stats.fec = m_fec;
	m_stats.label = this->get_parameter("label").as_string();

	m_pub_stats = this->create_publisher<nimbro_topic_transport::msg::ReceiverStats>(
		"/network/receiver_stats", 1);

	m_statsIntervalSec = 2.0;
	m_statsTimer = this->create_wall_timer(
		std::chrono::milliseconds(static_cast<int>(m_statsIntervalSec * 1000)),
		std::bind(&UDPReceiver::updateStats, this)
	);

	m_topicPrefix = this->get_parameter("topic_prefix").as_string();
}

UDPReceiver::~UDPReceiver()
{
	if(m_fd >= 0)
		close(m_fd);
}

template<class HeaderType>
void UDPReceiver::handleFinishedMessage(Message* msg, HeaderType* header)
{
	if(msg->complete)
		return;

	msg->complete = true;

	// Enforce null termination
	header->topic_type[sizeof(header->topic_type)-1] = 0;
	header->topic_name[sizeof(header->topic_name)-1] = 0;

	RCLCPP_DEBUG(this->get_logger(), "Got a packet of type %s, topic %s, (msg id %d), size %d",
		header->topic_type, header->topic_name, msg->id, (int)msg->payload.size());

	// Find or create topic
	TopicMap::iterator topic_it = m_topics.find(header->topic_name);

	std::shared_ptr<TopicReceiver> topic;
	if(topic_it == m_topics.end())
	{
		m_topics.insert(std::pair<std::string, std::shared_ptr<TopicReceiver>>(
			header->topic_name,
			std::make_shared<TopicReceiver>(shared_from_this())
		));
		topic = m_topics[header->topic_name];
		topic->last_message_counter = -1;
	}
	else
		topic = topic_it->second;

	if(m_dropRepeatedMessages && header->topic_msg_counter() == topic->last_message_counter)
		return;

	bool is_compressed = header->flags & UDP_FLAG_COMPRESSED;

	// Create publisher if topic type changed or first message
	if(topic->last_message_counter == -1 ||
		topic->topic_type != std::string(header->topic_type) ||
		(m_keepCompressed && topic->compressed != is_compressed))
	{
		RCLCPP_INFO(this->get_logger(), "Received first message on topic '%s' (type '%s')",
			header->topic_name, header->topic_type);

		topic->topic_type = header->topic_type;

		if(m_keepCompressed && is_compressed)
		{
			topic->compressed_publisher =
				this->create_publisher<nimbro_topic_transport::msg::CompressedMsg>(
					m_topicPrefix + header->topic_name, 1);
			topic->publisher.reset();
		}
		else
		{
			topic->publisher = this->create_generic_publisher(
				m_topicPrefix + header->topic_name,
				header->topic_type,
				rclcpp::SensorDataQoS());
			topic->compressed_publisher.reset();
		}

		topic->compressed = is_compressed;

		// Give subscribers time to discover the new publisher
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	if(is_compressed && m_keepCompressed)
	{
		nimbro_topic_transport::msg::CompressedMsg compressed;
		compressed.type = header->topic_type;
		compressed.data.swap(msg->payload);

		topic->publishCompressed(compressed);
	}
	else if(header->flags & UDP_FLAG_COMPRESSED)
	{
		topic->takeForDecompression(std::make_shared<Message>(*msg));
	}
	else
	{
		// Publish CDR bytes directly
		topic->publish(msg->payload.data(), msg->size);
	}

	topic->last_message_counter = header->topic_msg_counter();
}

void UDPReceiver::run()
{
	std::vector<uint8_t> buf;

	RCLCPP_INFO(this->get_logger(), "UDP receiver ready");
	while(rclcpp::ok())
	{
		rclcpp::spin_some(shared_from_this());

		buf.resize(PACKET_SIZE);

		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);

		timeval timeout;
		timeout.tv_usec = 50 * 1000;
		timeout.tv_sec = 0;

		int ret = select(m_fd+1, &fds, nullptr, nullptr, &timeout);
		if(ret < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;

			RCLCPP_FATAL(this->get_logger(), "Could not select(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}
		if(ret == 0)
			continue;

		sockaddr_storage addr;
		socklen_t addrlen = sizeof(addr);

		ssize_t size = recvfrom(m_fd, buf.data(), buf.size(), 0, (sockaddr*)&addr, &addrlen);

		if(size < 0)
		{
			RCLCPP_FATAL(this->get_logger(), "Could not recv(): %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		if(addrlen != m_remoteAddrLen || memcmp(&addr, &m_remoteAddr, addrlen) != 0)
		{
			char nameBuf[256];
			char serviceBuf[256];

			auto startLookup = std::chrono::steady_clock::now();
			if(getnameinfo((sockaddr*)&addr, addrlen, nameBuf, sizeof(nameBuf),
				serviceBuf, sizeof(serviceBuf), NI_NUMERICSERV) == 0)
			{
				RCLCPP_INFO(this->get_logger(), "New remote: %s:%s", nameBuf, serviceBuf);
				m_stats.remote = nameBuf;
				m_stats.remote_port = atoi(serviceBuf);
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Could not resolve remote address to name");
				m_stats.remote = "unknown";
				m_stats.remote_port = 0;
			}
			auto endLookup = std::chrono::steady_clock::now();

			if(endLookup - startLookup > std::chrono::seconds(1))
			{
				RCLCPP_WARN(this->get_logger(),
					"Reverse address lookup took more than a second. "
					"Consider adding '%s' to /etc/hosts",
					m_stats.remote.c_str()
				);
			}

			m_remoteAddr = addr;
			m_remoteAddrLen = addrlen;
		}

		m_receivedBytesInStatsInterval += size;

		uint16_t msg_id;

		if(m_fec)
		{
			FECPacket::Header* header = (FECPacket::Header*)buf.data();
			msg_id = header->msg_id();
		}
		else
		{
			UDPGenericPacket* generic = (UDPGenericPacket*)buf.data();
			msg_id = generic->msg_id();
		}

		// Look up message ID
		MessageBuffer::iterator it = std::find_if(
			m_incompleteMessages.begin(), m_incompleteMessages.end(),
			[=](const Message& msg) { return msg.id == msg_id; }
		);

		if(it == m_incompleteMessages.end())
		{
			m_incompleteMessages.push_front(Message(msg_id));
			it = m_incompleteMessages.begin();
			pruneMessages();
		}

		handleMessagePacket(it, &buf, size);
	}
}

void UDPReceiver::updateStats()
{
	m_stats.header.stamp = this->now();
	m_stats.bandwidth = m_receivedBytesInStatsInterval / m_statsIntervalSec;

	if(m_expectedPacketsInStatsInterval > 0)
		m_stats.drop_rate = ((double)m_missingPacketsInStatsInterval) / m_expectedPacketsInStatsInterval;
	else
		m_stats.drop_rate = 0;

	m_pub_stats->publish(m_stats);

	m_receivedBytesInStatsInterval = 0;
	m_missingPacketsInStatsInterval = 0;
	m_expectedPacketsInStatsInterval = 0;
}

void UDPReceiver::pruneMessages()
{
	MessageBuffer::iterator itr = m_incompleteMessages.begin();
	MessageBuffer::iterator it_end = m_incompleteMessages.end();
	for(int i = 0; i < 31; ++i)
	{
		++itr;
		if(itr == it_end)
			break;
	}

	// Collect statistics on packets which will be deleted
	for(MessageBuffer::iterator itd = itr; itd != it_end; ++itd)
	{
		const Message& msg = *itd;

#if WITH_OPENFEC
		if(m_fec)
		{
			if(msg.params)
			{
				int total = msg.params->nb_source_symbols + msg.params->nb_repair_symbols;
				m_expectedPacketsInStatsInterval += total;
				m_missingPacketsInStatsInterval += total - msg.received_symbols;
			}
		}
		else
#endif
		{
			int num_fragments = msg.msgs.size();
			int received = 0;
			for(unsigned int i = 0; i < msg.msgs.size(); ++i)
			{
				if(msg.msgs[i])
					received++;
			}

			m_expectedPacketsInStatsInterval += num_fragments;
			m_missingPacketsInStatsInterval += num_fragments - received;
		}
	}

	if(m_warnDropIncomplete)
	{
		for(MessageBuffer::iterator itd = itr; itd != it_end; ++itd)
		{
			const Message& msg = *itd;

			if(msg.complete)
				continue;

			int num_fragments = msg.msgs.size();
			int received = 0;
			for(unsigned int i = 0; i < msg.msgs.size(); ++i)
			{
				if(msg.msgs[i])
					received++;
			}

#if WITH_OPENFEC
			if(msg.decoder)
			{
				RCLCPP_WARN(this->get_logger(), "Dropping FEC message %d (%u/%u symbols)",
					msg.id, msg.received_symbols, msg.params->nb_source_symbols);
			}
			else
#endif
			{
				RCLCPP_WARN(this->get_logger(), "Dropping message %d, %.2f%% of fragments received (%d/%d)",
					msg.id, 100.0 * received / num_fragments, received, num_fragments);
			}
		}
	}

	m_incompleteMessages.erase(itr, it_end);
}

void UDPReceiver::handleMessagePacket(MessageBuffer::iterator it, std::vector<uint8_t>* buf, std::size_t size)
{
	Message* msg = &*it;

	if(msg->complete)
	{
#if WITH_OPENFEC
		msg->received_symbols++;
#endif
		return;
	}

	if(m_fec)
	{
#if WITH_OPENFEC
		std::shared_ptr<std::vector<uint8_t>> fecBuffer = std::make_shared<std::vector<uint8_t>>();
		fecBuffer->swap(*buf);

		msg->fecPackets.push_back(fecBuffer);

		FECPacket* packet = (FECPacket*)fecBuffer->data();

		if(!msg->decoder)
		{
			of_session_t* ses = nullptr;
			of_parameters_t* params = nullptr;

			if(packet->header.source_symbols() >= MIN_PACKETS_LDPC)
			{
				if(of_create_codec_instance(&ses, OF_CODEC_LDPC_STAIRCASE_STABLE, OF_DECODER, 1) != OF_STATUS_OK)
				{
					RCLCPP_ERROR(this->get_logger(), "Could not create LDPC decoder");
					return;
				}

				of_ldpc_parameters_t* ldpc_params = (of_ldpc_parameters_t*)malloc(sizeof(of_ldpc_parameters_t));
				ldpc_params->nb_source_symbols = packet->header.source_symbols();
				ldpc_params->nb_repair_symbols = packet->header.repair_symbols();
				ldpc_params->encoding_symbol_length = packet->header.symbol_length();
				ldpc_params->prng_seed = packet->header.prng_seed();
				ldpc_params->N1 = 7;

				params = (of_parameters_t*)ldpc_params;
			}
			else
			{
				if(of_create_codec_instance(&ses, OF_CODEC_REED_SOLOMON_GF_2_M_STABLE, OF_DECODER, 1) != OF_STATUS_OK)
				{
					RCLCPP_ERROR(this->get_logger(), "Could not create REED_SOLOMON decoder");
					return;
				}

				of_rs_2_m_parameters_t* rs_params = (of_rs_2_m_parameters_t*)malloc(sizeof(of_rs_2_m_parameters_t));
				rs_params->nb_source_symbols = packet->header.source_symbols();
				rs_params->nb_repair_symbols = packet->header.repair_symbols();
				rs_params->encoding_symbol_length = packet->header.symbol_length();
				rs_params->m = 8;

				params = (of_parameters_t*)rs_params;
			}

			if(of_set_fec_parameters(ses, params) != OF_STATUS_OK)
			{
				RCLCPP_ERROR(this->get_logger(), "Could not set FEC parameters");
				of_release_codec_instance(ses);
				return;
			}

			msg->decoder.reset(ses, of_release_codec_instance);
			msg->params.reset(params, free);
			msg->received_symbols = 0;
		}

		msg->received_symbols++;

		uint8_t* symbol_begin = packet->data;

		if(size - sizeof(FECPacket::Header) != msg->params->encoding_symbol_length)
		{
			RCLCPP_ERROR(this->get_logger(), "Symbol size mismatch: got %d, expected %d",
				(int)(size - sizeof(FECPacket::Header)),
				(int)(msg->params->encoding_symbol_length));
			return;
		}

		if(of_decode_with_new_symbol(msg->decoder.get(), symbol_begin, packet->header.symbol_id()) != OF_STATUS_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Could not decode symbol");
			return;
		}

		bool done = false;

		if(msg->received_symbols >= msg->params->nb_source_symbols)
		{
			done = of_is_decoding_complete(msg->decoder.get());

			if(!done && msg->received_symbols >= msg->params->nb_source_symbols + msg->params->nb_repair_symbols / 2)
			{
				of_status_t ret = of_finish_decoding(msg->decoder.get());
				if(ret == OF_STATUS_OK)
					done = true;
				else
				{
					RCLCPP_ERROR(this->get_logger(), "ML decoding failed, dropping message...");
					msg->complete = true;
					return;
				}
			}
		}

		if(done)
		{
			std::vector<void*> symbols(msg->params->nb_source_symbols, nullptr);

			if(of_get_source_symbols_tab(msg->decoder.get(), symbols.data()) != OF_STATUS_OK)
			{
				RCLCPP_ERROR(this->get_logger(), "Could not get decoded symbols");
				return;
			}

			uint64_t payloadLength = msg->params->nb_source_symbols * msg->params->encoding_symbol_length;
			if(msg->params->encoding_symbol_length < sizeof(FECHeader) || payloadLength < sizeof(FECHeader))
			{
				RCLCPP_ERROR(this->get_logger(), "Invalid short payload");
				m_incompleteMessages.erase(it);
				return;
			}

			FECHeader msgHeader;
			memcpy(&msgHeader, symbols[0], sizeof(FECHeader));
			payloadLength -= sizeof(FECHeader);

			msg->payload.resize(payloadLength);

			uint8_t* writePtr = msg->payload.data();
			memcpy(
				msg->payload.data(),
				((uint8_t*)symbols[0]) + sizeof(FECHeader),
				msg->params->encoding_symbol_length - sizeof(FECHeader)
			);
			writePtr += msg->params->encoding_symbol_length - sizeof(FECHeader);

			for(unsigned int symbol = 1; symbol < msg->params->nb_source_symbols; ++symbol)
			{
				memcpy(writePtr, symbols[symbol], msg->params->encoding_symbol_length);
				writePtr += msg->params->encoding_symbol_length;
			}

			msg->size = payloadLength;

			handleFinishedMessage(msg, &msgHeader);
		}
#endif
	}
	else
	{
		UDPGenericPacket* generic = (UDPGenericPacket*)buf->data();
		if(generic->frag_id == 0)
		{
			UDPFirstPacket* first = (UDPFirstPacket*)buf->data();

			msg->header = first->header;

			uint32_t required_size = (msg->header.remaining_packets()+1) * PACKET_SIZE;
			uint32_t my_size = size - sizeof(UDPFirstPacket);
			if(msg->payload.size() < required_size)
				msg->payload.resize(required_size);
			memcpy(msg->payload.data(), first->data, my_size);

			if(msg->size < my_size)
				msg->size = my_size;

			if(((uint16_t)msg->msgs.size()) < msg->header.remaining_packets()+1)
				msg->msgs.resize(msg->header.remaining_packets()+1, false);
		}
		else
		{
			UDPDataPacket* data = (UDPDataPacket*)buf->data();

			uint32_t offset = UDPFirstPacket::MaxDataSize + (data->header.frag_id-1) * UDPDataPacket::MaxDataSize;
			uint32_t required_size = offset + size - sizeof(UDPDataPacket);
			if(msg->payload.size() < required_size)
				msg->payload.resize(required_size);
			memcpy(msg->payload.data() + offset, data->data, size - sizeof(UDPDataPacket));

			if(msg->size < required_size)
				msg->size = required_size;
		}

		if(generic->frag_id >= msg->msgs.size())
			msg->msgs.resize(generic->frag_id+1, false);

		msg->msgs[generic->frag_id] = true;

		if(std::all_of(msg->msgs.begin(), msg->msgs.end(), [](bool x){return x;}))
		{
			handleFinishedMessage(msg, &msg->header);

			m_expectedPacketsInStatsInterval += msg->msgs.size();

			m_incompleteMessages.erase(it);
		}
	}
}

}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<nimbro_topic_transport::UDPReceiver>();
	node->run();

	rclcpp::shutdown();

	return 0;
}
