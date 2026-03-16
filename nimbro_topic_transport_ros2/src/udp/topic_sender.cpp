// Sends a single topic (ROS2)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_sender.h"
#include "udp_sender.h"
#include "udp_packet.h"

#include <bzlib.h>

#include <random>
#include <algorithm>
#include <chrono>
#include <cstring>

#if WITH_OPENFEC
extern "C"
{
#include <of_openfec_api.h>
}
#endif

namespace nimbro_topic_transport
{

TopicSender::TopicSender(UDPSender* sender, rclcpp::Node* node,
	const std::string& topic, double rate, bool resend, int flags,
	bool enable, const std::string& type)
 : m_sender(sender)
 , m_node(node)
 , m_flags(flags)
 , m_durationBetweenPackets(rclcpp::Duration::from_seconds(rate == 0.0 ? 0.0 : 1.0 / rate))
 , m_lastTime(0, 0, RCL_ROS_TIME)
 , m_updateBuf(true)
 , m_msgCounter(0)
 , m_inputMsgCounter(0)
 , m_enabled(enable)
 , m_directTransmission(true)
{
	std::string sub_type = type.empty() ? "*" : type;

	m_subscriber = node->create_generic_subscription(
		topic, sub_type,
		rclcpp::SensorDataQoS(),
		[this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
			this->handleData(msg);
		}
	);

	m_topicName = topic;

	if(resend && rate > 0.0)
	{
		auto duration = std::chrono::duration<double>(1.0 / rate);
		m_resendTimer = node->create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(duration),
			[this]() { this->resend(); }
		);
	}

	RCLCPP_INFO(node->get_logger(), "Subscribed to '%s' (type: %s, compress: %s, rate: %.1f)",
		topic.c_str(), sub_type.c_str(),
		(flags & UDP_FLAG_COMPRESSED) ? "yes" : "no", rate);
}

TopicSender::~TopicSender()
{
	RCLCPP_DEBUG(m_node->get_logger(), "Topic '%s': Sent %d messages",
		m_topicName.c_str(), m_msgCounter);
}

void TopicSender::send()
{
	if(!m_enabled)
		return;

	if(m_updateBuf)
	{
		std::lock_guard<std::mutex> lock(m_dataMutex);

		if(m_buf.empty())
			return;

		if(m_flags & UDP_FLAG_COMPRESSED)
		{
			unsigned int len = m_buf.size() + m_buf.size() / 100 + 1200;
			m_compressionBuf.resize(len);
			int ret = BZ2_bzBuffToBuffCompress(
				(char*)m_compressionBuf.data(), &len,
				(char*)m_buf.data(), m_buf.size(), 3, 0, 30);
			if(ret == BZ_OK)
			{
				m_buf.swap(m_compressionBuf);
				m_buf.resize(len);
			}
			else
			{
				RCLCPP_ERROR(m_node->get_logger(), "Could not compress data, sending uncompressed");
			}
		}

		m_updateBuf = false;
	}

	// Do we want to do forward error correction?
	if(m_sender->fec() != 0.0)
	{
		sendWithFEC();
	}
	else
	{
		sendWithoutFEC();
	}

	m_msgCounter++;
}

inline uint64_t div_round_up(uint64_t a, uint64_t b)
{
	return (a + b - 1) / b;
}

void TopicSender::sendWithFEC()
{
#if WITH_OPENFEC
	uint16_t msg_id = m_sender->allocateMessageID();
	uint64_t dataSize = sizeof(FECHeader) + m_buf.size();

	uint64_t symbolSize;
	uint64_t sourceSymbols;

	if(dataSize <= FECPacket::MaxDataSize)
	{
		sourceSymbols = 1;
		symbolSize = dataSize;
	}
	else
	{
		sourceSymbols = div_round_up(dataSize, FECPacket::MaxDataSize);
		symbolSize = FECPacket::MaxDataSize;
	}

	uint64_t packetSize = sizeof(FECPacket::Header) + symbolSize;
	uint64_t repairSymbols = std::ceil(m_sender->fec() * sourceSymbols);
	uint64_t numPackets = sourceSymbols + repairSymbols;

	of_session_t* ses = 0;
	uint32_t prng_seed = rand();
	if(sourceSymbols >= MIN_PACKETS_LDPC)
	{
		if(of_create_codec_instance(&ses, OF_CODEC_LDPC_STAIRCASE_STABLE, OF_ENCODER, 1) != OF_STATUS_OK)
		{
			RCLCPP_ERROR(m_node->get_logger(), "%s: Could not create LDPC codec instance", m_topicName.c_str());
			return;
		}

		of_ldpc_parameters_t params;
		params.nb_source_symbols = sourceSymbols;
		params.nb_repair_symbols = repairSymbols;
		params.encoding_symbol_length = symbolSize;
		params.prng_seed = prng_seed;
		params.N1 = 7;

		if(of_set_fec_parameters(ses, (of_parameters_t*)&params) != OF_STATUS_OK)
		{
			RCLCPP_ERROR(m_node->get_logger(), "%s: Could not set FEC parameters", m_topicName.c_str());
			of_release_codec_instance(ses);
			return;
		}
	}
	else
	{
		if(of_create_codec_instance(&ses, OF_CODEC_REED_SOLOMON_GF_2_M_STABLE, OF_ENCODER, 0) != OF_STATUS_OK)
		{
			RCLCPP_ERROR(m_node->get_logger(), "%s: Could not create REED_SOLOMON codec instance", m_topicName.c_str());
			return;
		}

		of_rs_2_m_parameters params;
		params.nb_source_symbols = sourceSymbols;
		params.nb_repair_symbols = repairSymbols;
		params.encoding_symbol_length = symbolSize;
		params.m = 8;

		if(of_set_fec_parameters(ses, (of_parameters_t*)&params) != OF_STATUS_OK)
		{
			RCLCPP_ERROR(m_node->get_logger(), "%s: Could not set FEC parameters", m_topicName.c_str());
			of_release_codec_instance(ses);
			return;
		}
	}

	std::vector<uint8_t> packetBuffer(numPackets * packetSize);
	std::vector<void*> symbols(sourceSymbols + repairSymbols);

	uint64_t writtenData = 0;

	for(uint64_t i = 0; i < sourceSymbols; ++i)
	{
		uint8_t* packetPtr = packetBuffer.data() + i * packetSize;
		FECPacket::Header* header = reinterpret_cast<FECPacket::Header*>(packetPtr);

		header->msg_id = msg_id;
		header->symbol_id = i;
		header->symbol_length = symbolSize;
		header->source_symbols = sourceSymbols;
		header->repair_symbols = repairSymbols;
		header->prng_seed = prng_seed;

		uint8_t* dataPtr = packetPtr + sizeof(FECPacket::Header);
		uint64_t remainingSpace = symbolSize;

		symbols[i] = dataPtr;

		if(i == 0)
		{
			FECHeader* msgHeader = reinterpret_cast<FECHeader*>(dataPtr);

			msgHeader->flags = m_flags;
			msgHeader->topic_msg_counter = m_inputMsgCounter;

			strncpy(msgHeader->topic_name, m_topicName.c_str(), sizeof(msgHeader->topic_name));
			if(msgHeader->topic_name[sizeof(msgHeader->topic_name)-1] != 0)
			{
				RCLCPP_ERROR(m_node->get_logger(), "Topic '%s' is too long", m_topicName.c_str());
				msgHeader->topic_name[sizeof(msgHeader->topic_name)-1] = 0;
			}

			strncpy(msgHeader->topic_type, m_topicType.c_str(), sizeof(msgHeader->topic_type));
			if(msgHeader->topic_type[sizeof(msgHeader->topic_type)-1] != 0)
			{
				RCLCPP_ERROR(m_node->get_logger(), "Topic type '%s' is too long", m_topicType.c_str());
				msgHeader->topic_type[sizeof(msgHeader->topic_type)-1] = 0;
			}

			dataPtr += sizeof(FECHeader);
			remainingSpace -= sizeof(FECHeader);
		}

		uint64_t chunkSize = std::min(remainingSpace, m_buf.size() - writtenData);
		memcpy(dataPtr, m_buf.data() + writtenData, chunkSize);
		writtenData += chunkSize;

		if(chunkSize < remainingSpace)
			memset(dataPtr + chunkSize, 0, remainingSpace - chunkSize);
	}

	for(uint64_t i = sourceSymbols; i < sourceSymbols + repairSymbols; ++i)
	{
		uint8_t* packetPtr = packetBuffer.data() + i * packetSize;
		FECPacket::Header* header = reinterpret_cast<FECPacket::Header*>(packetPtr);

		header->msg_id = msg_id;
		header->symbol_id = i;
		header->symbol_length = symbolSize;
		header->source_symbols = sourceSymbols;
		header->repair_symbols = repairSymbols;
		header->prng_seed = prng_seed;

		uint8_t* dataPtr = packetPtr + sizeof(FECPacket::Header);
		symbols[i] = dataPtr;
	}
	for(uint64_t i = sourceSymbols; i < sourceSymbols + repairSymbols; ++i)
	{
		if(of_build_repair_symbol(ses, symbols.data(), i) != OF_STATUS_OK)
		{
			RCLCPP_ERROR(m_node->get_logger(), "%s: Could not build repair symbol", m_topicName.c_str());
			of_release_codec_instance(ses);
			return;
		}
	}

	of_release_codec_instance(ses);

	std::vector<unsigned int> packetOrder(numPackets);
	std::iota(packetOrder.begin(), packetOrder.end(), 0);

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 mt(seed);
	std::shuffle(packetOrder.begin(), packetOrder.end(), mt);

	for(unsigned int idx : packetOrder)
	{
		if(!m_sender->send(packetBuffer.data() + idx * packetSize, packetSize, m_topicName))
			return;
	}
#else
	(void)m_sender;
	throw std::runtime_error("Forward error correction requested, but not compiled with FEC support");
#endif
}

void TopicSender::sendWithoutFEC()
{
	uint32_t size = m_buf.size();

	uint8_t buf[PACKET_SIZE];
	uint32_t buf_size = std::min<uint32_t>(PACKET_SIZE, sizeof(UDPFirstPacket) + size);
	UDPFirstPacket* first = (UDPFirstPacket*)buf;

	uint16_t msg_id = m_sender->allocateMessageID();

	first->header.frag_id = 0;
	first->header.msg_id = msg_id;
	first->header.flags = m_flags;
	first->header.topic_msg_counter = m_inputMsgCounter;

	first->header.remaining_packets = std::max<uint32_t>(0,
		(size - UDPFirstPacket::MaxDataSize + (UDPDataPacket::MaxDataSize-1)) / UDPDataPacket::MaxDataSize
	);

	strncpy(first->header.topic_name, m_topicName.c_str(), sizeof(first->header.topic_name));
	if(first->header.topic_name[sizeof(first->header.topic_name)-1] != 0)
	{
		RCLCPP_ERROR(m_node->get_logger(), "Topic '%s' is too long", m_topicName.c_str());
		first->header.topic_name[sizeof(first->header.topic_name)-1] = 0;
	}

	strncpy(first->header.topic_type, m_topicType.c_str(), sizeof(first->header.topic_type));
	if(first->header.topic_type[sizeof(first->header.topic_type)-1] != 0)
	{
		RCLCPP_ERROR(m_node->get_logger(), "Topic type '%s' is too long", m_topicType.c_str());
		first->header.topic_type[sizeof(first->header.topic_type)-1] = 0;
	}

	uint8_t* rptr = m_buf.data();
	uint32_t psize = std::min<uint32_t>(UDPFirstPacket::MaxDataSize, size);
	memcpy(first->data, rptr, psize);
	rptr += psize;
	size -= psize;

	if(!m_sender->send(buf, buf_size, m_topicName))
		return;

	if(m_sender->duplicateFirstPacket())
	{
		if(!m_sender->send(buf, buf_size, m_topicName))
			return;
	}

	uint16_t frag_id = 1;
	while(size > 0)
	{
		buf_size = std::min<uint32_t>(PACKET_SIZE, sizeof(UDPDataPacket) + size);
		UDPDataPacket* next_packet = (UDPDataPacket*)buf;
		next_packet->header.frag_id = frag_id++;
		next_packet->header.msg_id = msg_id;

		psize = std::min<uint32_t>(UDPDataPacket::MaxDataSize, size);
		memcpy(next_packet->data, rptr, psize);
		rptr += psize;
		size -= psize;

		if(!m_sender->send(buf, buf_size, m_topicName))
			return;
	}
}

void TopicSender::handleData(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
	if(!m_enabled)
		return;

	{
		std::lock_guard<std::mutex> lock(m_dataMutex);

		auto& rcl_msg = msg->get_rcl_serialized_message();
		m_buf.assign(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);

		// Get the type from subscription (stored at construction)
		if(m_topicType.empty())
		{
			// Retrieve from parameter arrays
			auto topic_names = m_node->get_parameter("topic_names").as_string_array();
			auto topic_types = m_node->get_parameter("topic_types").as_string_array();
			for(size_t i = 0; i < topic_names.size(); ++i)
			{
				if(topic_names[i] == m_topicName && i < topic_types.size())
				{
					m_topicType = topic_types[i];
					break;
				}
			}
		}

		m_updateBuf = true;

		auto now = m_node->now();
		if(m_lastTime.nanoseconds() != 0 &&
			(now - m_lastTime) < m_durationBetweenPackets)
			return;

		m_lastTime = now;
		m_inputMsgCounter++;
	}

	if(m_directTransmission)
		send();
}

void TopicSender::resend()
{
	if(m_buf.empty())
		return;

	sendCurrentMessage();
}

void TopicSender::sendCurrentMessage()
{
	if(m_buf.empty())
		return;

	send();
}

void TopicSender::setDirectTransmissionEnabled(bool value)
{
	m_directTransmission = value;
}

}
