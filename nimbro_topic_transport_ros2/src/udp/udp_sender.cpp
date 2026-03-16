// UDP sender node (ROS2)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "udp_sender.h"
#include "topic_sender.h"
#include "udp_packet.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include <cstring>

namespace nimbro_topic_transport
{

UDPSender::UDPSender()
 : Node("udp_sender")
 , m_msgID(0)
 , m_sentBytesInStatsInterval(0)
{
	// Declare parameters
	this->declare_parameter<std::string>("destination_addr", "localhost");
	this->declare_parameter<int>("destination_port", 5050);
	this->declare_parameter<int>("source_port", -1);
	this->declare_parameter<bool>("relay_mode", false);
	this->declare_parameter<double>("relay_target_bitrate", 0.0);
	this->declare_parameter<double>("relay_control_rate", 100.0);
	this->declare_parameter<double>("fec", 0.0);
	this->declare_parameter<bool>("duplicate_first_packet", false);
	this->declare_parameter<std::string>("label", "");
	this->declare_parameter<std::vector<std::string>>("topic_names", std::vector<std::string>());
	this->declare_parameter<std::vector<bool>>("topic_compress", std::vector<bool>());
	this->declare_parameter<std::vector<double>>("topic_rates", std::vector<double>());
	this->declare_parameter<std::vector<bool>>("topic_resend", std::vector<bool>());
	this->declare_parameter<std::vector<bool>>("topic_enable", std::vector<bool>());
	this->declare_parameter<std::vector<std::string>>("topic_types", std::vector<std::string>());

	m_relayMode = this->get_parameter("relay_mode").as_bool();

	std::string dest_host = this->get_parameter("destination_addr").as_string();
	int dest_port = this->get_parameter("destination_port").as_int();
	std::string dest_port_str = std::to_string(dest_port);

	addrinfo* info = nullptr;
	if(getaddrinfo(dest_host.c_str(), dest_port_str.c_str(), nullptr, &info) != 0 || !info)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not lookup destination address '%s': %s",
			dest_host.c_str(), strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	m_fd = socket(info->ai_family, SOCK_DGRAM, 0);
	if(m_fd < 0)
	{
		freeaddrinfo(info);
		RCLCPP_FATAL(this->get_logger(), "Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) != 0)
	{
		freeaddrinfo(info);
		RCLCPP_FATAL(this->get_logger(), "Could not enable SO_BROADCAST flag: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
	m_addrLen = info->ai_addrlen;

	int source_port = this->get_parameter("source_port").as_int();
	if(source_port != -1)
	{
		std::string source_port_str = std::to_string(source_port);

		addrinfo hints;
		memset(&hints, 0, sizeof(hints));
		hints.ai_flags = AI_PASSIVE;
		hints.ai_family = info->ai_family;
		hints.ai_socktype = SOCK_DGRAM;

		addrinfo* localInfo = nullptr;
		if(getaddrinfo(nullptr, source_port_str.c_str(), &hints, &localInfo) != 0 || !localInfo)
		{
			freeaddrinfo(info);
			RCLCPP_FATAL(this->get_logger(), "Could not get local address: %s", strerror(errno));
			throw std::runtime_error("Could not get local address");
		}

		if(bind(m_fd, localInfo->ai_addr, localInfo->ai_addrlen) != 0)
		{
			freeaddrinfo(localInfo);
			freeaddrinfo(info);
			RCLCPP_FATAL(this->get_logger(), "Could not bind to source port: %s", strerror(errno));
			throw std::runtime_error(strerror(errno));
		}

		freeaddrinfo(localInfo);
	}

	freeaddrinfo(info);

	m_fec = this->get_parameter("fec").as_double();

	// Parse topic list from parallel arrays
	auto topic_names = this->get_parameter("topic_names").as_string_array();
	auto topic_compress = this->get_parameter("topic_compress").as_bool_array();
	auto topic_rates = this->get_parameter("topic_rates").as_double_array();
	auto topic_resend = this->get_parameter("topic_resend").as_bool_array();
	auto topic_enable = this->get_parameter("topic_enable").as_bool_array();
	auto topic_types = this->get_parameter("topic_types").as_string_array();

	for(size_t i = 0; i < topic_names.size(); ++i)
	{
		int flags = 0;
		double rate = 0.0;
		bool resend = false;
		bool enabled = true;
		std::string type;

		if(i < topic_compress.size() && topic_compress[i])
			flags |= UDP_FLAG_COMPRESSED;

		if(i < topic_rates.size())
			rate = topic_rates[i];

		if(i < topic_resend.size())
			resend = topic_resend[i];

		if(i < topic_enable.size())
			enabled = topic_enable[i];

		if(i < topic_types.size())
			type = topic_types[i];

		TopicSender* sender = new TopicSender(
			this, this, topic_names[i], rate, resend, flags, enabled, type);

		if(m_relayMode)
			sender->setDirectTransmissionEnabled(false);

		m_senders.push_back(sender);
	}

	m_duplicateFirstPacket = this->get_parameter("duplicate_first_packet").as_bool();

	if(m_relayMode)
	{
		double target_bitrate = this->get_parameter("relay_target_bitrate").as_double();
		if(target_bitrate == 0.0)
			throw std::runtime_error("relay mode needs relay_target_bitrate param");

		double relay_control_rate = this->get_parameter("relay_control_rate").as_double();

		m_relayTokens = 0;
		m_relayIndex = 0;
		m_relayTokensPerStep = target_bitrate / 8.0 / relay_control_rate;

		m_relayThreadShouldExit = false;
		m_relayRate = relay_control_rate;
		m_relayThread = std::thread(&UDPSender::relay, this);

		RCLCPP_INFO(this->get_logger(),
			"Relay mode: control rate %.1f, target bitrate %.0f bit/s, token increment %d",
			relay_control_rate, target_bitrate, m_relayTokensPerStep);
	}

	// Stats
	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = this->get_name();
	m_stats.protocol = "UDP";
	m_stats.host = hostnameBuf;
	m_stats.destination = dest_host;
	m_stats.destination_port = dest_port;
	m_stats.source_port = (source_port != -1) ? source_port : dest_port;
	m_stats.fec = m_fec != 0.0;
	m_stats.label = this->get_parameter("label").as_string();

	m_pub_stats = this->create_publisher<nimbro_topic_transport::msg::SenderStats>(
		"/network/sender_stats", 1);

	m_statsIntervalSec = 2.0;
	m_statsTimer = this->create_wall_timer(
		std::chrono::milliseconds(static_cast<int>(m_statsIntervalSec * 1000)),
		std::bind(&UDPSender::updateStats, this)
	);
}

UDPSender::~UDPSender()
{
	if(m_relayMode)
	{
		m_relayThreadShouldExit = true;
		if(m_relayThread.joinable())
			m_relayThread.join();
	}

	for(auto* sender : m_senders)
		delete sender;

	if(m_fd >= 0)
		close(m_fd);
}

uint16_t UDPSender::allocateMessageID()
{
	return m_msgID++;
}

bool UDPSender::send(const void* data, uint32_t size, const std::string& topic)
{
	if(m_relayMode)
	{
		std::vector<uint8_t> packet(size);
		memcpy(packet.data(), data, size);

		m_relayBuffer.emplace_back(std::move(packet));
		m_relayNameBuffer.push_back(topic);
		return true;
	}
	else
	{
		return internalSend(data, size, topic);
	}
}

bool UDPSender::internalSend(const void* data, uint32_t size, const std::string& topic)
{
	if(sendto(m_fd, data, size, 0, (sockaddr*)&m_addr, m_addrLen) != size)
	{
		RCLCPP_ERROR(this->get_logger(), "Could not send data of size %d: %s",
			size, strerror(errno));
		return false;
	}

	m_sentBytesInStatsInterval += size;
	if(!topic.empty())
		m_sentTopicBytesInStatsInterval[topic] += size;

	return true;
}

void UDPSender::relay()
{
	RCLCPP_INFO(this->get_logger(), "Relay thread starting...");

	auto interval = std::chrono::duration<double>(1.0 / m_relayRate);

	while(!m_relayThreadShouldExit)
	{
		m_relayTokens = std::min<uint64_t>(
			100 * m_relayTokensPerStep,
			m_relayTokens + m_relayTokensPerStep
		);

		if(m_senders.empty())
			throw std::runtime_error("No senders configured");

		while(true)
		{
			unsigned int tries = 0;
			bool noData = false;

			while(m_relayBuffer.empty())
			{
				if(tries++ == m_senders.size())
				{
					noData = true;
					break;
				}

				m_senders[m_relayIndex]->sendCurrentMessage();
				m_relayIndex = (m_relayIndex + 1) % m_senders.size();
			}

			if(noData)
				break;

			const std::vector<uint8_t>& packet = m_relayBuffer.front();
			const std::string& name = m_relayNameBuffer.front();
			std::size_t sizeOnWire = packet.size() + 20 + 8;

			if(sizeOnWire > m_relayTokens)
				break;

			if(!internalSend(packet.data(), packet.size(), name))
			{
				RCLCPP_ERROR(this->get_logger(), "Could not send packet");
				break;
			}

			m_relayTokens -= sizeOnWire;
			m_relayBuffer.pop_front();
			m_relayNameBuffer.pop_front();
		}

		std::this_thread::sleep_for(interval);
	}

	RCLCPP_INFO(this->get_logger(), "Relay thread exiting...");
}

void UDPSender::updateStats()
{
	m_stats.header.stamp = this->now();
	m_stats.bandwidth = 8.0 * m_sentBytesInStatsInterval / m_statsIntervalSec;

	m_stats.topics.clear();
	for(auto& pair : m_sentTopicBytesInStatsInterval)
	{
		nimbro_topic_transport::msg::TopicBandwidth tp;
		tp.topic = pair.first;
		tp.bandwidth = 8.0 * pair.second / m_statsIntervalSec;
		pair.second = 0;
		m_stats.topics.emplace_back(tp);
	}

	m_pub_stats->publish(m_stats);
	m_sentBytesInStatsInterval = 0;
}

}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<nimbro_topic_transport::UDPSender>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
