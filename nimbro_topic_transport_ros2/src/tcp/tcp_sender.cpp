// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// TCP sender (ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#include "tcp_sender.h"

#include <bzlib.h>

#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>

namespace nimbro_topic_transport
{

TCPSender::TCPSender()
 : Node("tcp_sender")
 , m_fd(-1)
 , m_sentBytesInStatsInterval(0)
{
	RCLCPP_INFO(this->get_logger(), "nimbro_topic_transport v2.0.0 — TCP sender");
	RCLCPP_INFO(this->get_logger(), "Based on nimbro_network by Max Schwarz, University of Bonn");
	RCLCPP_INFO(this->get_logger(), "ROS2 port by Guy Rodnay, Skana Robotics LTD");

	// Declare parameters
	this->declare_parameter<std::string>("destination_addr", "");
	this->declare_parameter<int>("destination_port", 0);
	this->declare_parameter<int>("source_port", -1);
	this->declare_parameter<std::string>("label", "");
	this->declare_parameter<std::vector<std::string>>("topic_names", std::vector<std::string>());
	this->declare_parameter<std::vector<bool>>("topic_compress", std::vector<bool>());
	this->declare_parameter<std::vector<std::string>>("topic_types", std::vector<std::string>());

	// Get destination address
	std::string addr = this->get_parameter("destination_addr").as_string();
	if(addr.empty())
	{
		RCLCPP_FATAL(this->get_logger(), "tcp_sender needs a 'destination_addr' parameter!");
		throw std::runtime_error("tcp_sender needs a 'destination_addr' parameter!");
	}

	int port = this->get_parameter("destination_port").as_int();
	if(port == 0)
	{
		RCLCPP_FATAL(this->get_logger(), "tcp_sender needs a 'destination_port' parameter!");
		throw std::runtime_error("tcp_sender needs a 'destination_port' parameter!");
	}

	m_sourcePort = this->get_parameter("source_port").as_int();

	std::string portStr = std::to_string(port);

	addrinfo* info = nullptr;
	if(getaddrinfo(addr.c_str(), portStr.c_str(), nullptr, &info) != 0 || !info)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not lookup host name '%s'", addr.c_str());
		throw std::runtime_error("Could not lookup hostname");
	}

	m_addrFamily = info->ai_family;
	memcpy(&m_addr, info->ai_addr, info->ai_addrlen);
	m_addrLen = info->ai_addrlen;

	freeaddrinfo(info);

	// Parse topic list from parallel arrays
	auto topic_names = this->get_parameter("topic_names").as_string_array();
	auto topic_compress = this->get_parameter("topic_compress").as_bool_array();
	auto topic_types = this->get_parameter("topic_types").as_string_array();

	for(size_t i = 0; i < topic_names.size(); ++i)
	{
		std::string topic = topic_names[i];
		int flags = 0;

		if(i < topic_compress.size() && topic_compress[i])
			flags |= TCP_FLAG_COMPRESSED;

		std::string type = (i < topic_types.size() && !topic_types[i].empty())
			? topic_types[i] : "*";

		auto sub = this->create_generic_subscription(
			topic, type,
			rclcpp::SensorDataQoS(),
			[this, topic, flags](std::shared_ptr<rclcpp::SerializedMessage> msg) {
				this->send(topic, flags, msg);
			}
		);

		m_subs.push_back(sub);
		RCLCPP_INFO(this->get_logger(), "Subscribed to '%s' (type: %s, compress: %s)",
			topic.c_str(), type.c_str(), (flags & TCP_FLAG_COMPRESSED) ? "yes" : "no");
	}

	// Stats setup
	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = this->get_name();
	m_stats.protocol = "TCP";
	m_stats.host = hostnameBuf;
	m_stats.destination = addr;
	m_stats.destination_port = port;
	m_stats.source_port = m_sourcePort;
	m_stats.fec = false;
	m_stats.label = this->get_parameter("label").as_string();

	m_pub_stats = this->create_publisher<nimbro_topic_transport::msg::SenderStats>(
		"/network/sender_stats", 1);

	m_statsIntervalSec = 2.0;
	m_statsTimer = this->create_wall_timer(
		std::chrono::milliseconds(static_cast<int>(m_statsIntervalSec * 1000)),
		std::bind(&TCPSender::updateStats, this)
	);

	RCLCPP_INFO(this->get_logger(), "Configuration: destination=%s:%d, topics=%zu, source_port=%d",
		addr.c_str(), port, topic_names.size(), m_sourcePort);
}

TCPSender::~TCPSender()
{
	if(m_fd >= 0)
		close(m_fd);
}

bool TCPSender::connect()
{
	m_fd = socket(m_addrFamily, SOCK_STREAM, 0);
	if(m_fd < 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Could not create socket: %s", strerror(errno));
		return false;
	}

	if(m_sourcePort != -1)
	{
		std::string source_port_str = std::to_string(m_sourcePort);

		addrinfo hints;
		memset(&hints, 0, sizeof(hints));

		hints.ai_flags = AI_PASSIVE;
		hints.ai_family = m_addrFamily;
		hints.ai_socktype = SOCK_STREAM;

		addrinfo* localInfo = nullptr;
		if(getaddrinfo(nullptr, source_port_str.c_str(), &hints, &localInfo) != 0 || !localInfo)
		{
			RCLCPP_FATAL(this->get_logger(), "Could not get local address: %s", strerror(errno));
			throw std::runtime_error("Could not get local address");
		}

		if(bind(m_fd, localInfo->ai_addr, localInfo->ai_addrlen) != 0)
		{
			RCLCPP_FATAL(this->get_logger(), "Could not bind to source port: %s", strerror(errno));
			freeaddrinfo(localInfo);
			throw std::runtime_error(strerror(errno));
		}

		freeaddrinfo(localInfo);
	}

	if(::connect(m_fd, (sockaddr*)&m_addr, m_addrLen) != 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Could not connect: %s", strerror(errno));
		close(m_fd);
		m_fd = -1;
		return false;
	}

	if(m_sourcePort == -1)
	{
		sockaddr_storage sa;
		socklen_t salen = sizeof(sa);

		char serviceBuf[256];

		if(getsockname(m_fd, (sockaddr*)&sa, &salen) == 0)
		{
			if(getnameinfo((sockaddr*)&sa, salen, nullptr, 0, serviceBuf, sizeof(serviceBuf), NI_NUMERICSERV) == 0)
			{
				m_stats.source_port = atoi(serviceBuf);
			}
		}
	}

#ifdef TCP_USER_TIMEOUT
	int timeout = 8000;
	if(setsockopt(m_fd, SOL_TCP, TCP_USER_TIMEOUT, &timeout, sizeof(timeout)) != 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Could not set TCP_USER_TIMEOUT: %s", strerror(errno));
		return false;
	}
#else
	RCLCPP_WARN(this->get_logger(), "Not setting TCP_USER_TIMEOUT");
#endif

	RCLCPP_INFO(this->get_logger(), "Connected to %s:%d",
		m_stats.destination.c_str(), m_stats.destination_port);

	return true;
}

void TCPSender::send(const std::string& topic, int flags,
	const std::shared_ptr<rclcpp::SerializedMessage>& msg)
{
	auto& rcl_msg = msg->get_rcl_serialized_message();
	const uint8_t* payload = rcl_msg.buffer;
	uint32_t size = rcl_msg.buffer_length;

	// We need the type string from the subscription. Since GenericSubscription
	// gives us serialized bytes but not the type, we embed the type in the
	// subscription's topic_type. For now, use the topic name to look it up
	// from our subscription list. The receiver uses the type to create
	// GenericPublisher.
	std::string type;
	for(size_t i = 0; i < m_subs.size(); ++i)
	{
		if(m_subs[i]->get_topic_name() == topic)
		{
			type = m_subs[i]->get_topic_name();
			break;
		}
	}
	// Fall back: receiver will use wildcard matching
	if(type.empty())
		type = "unknown";

	// Get actual type from the subscription
	for(size_t i = 0; i < m_subs.size(); ++i)
	{
		// GenericSubscription stores the type we passed at creation
		// We'll retrieve it from parameters instead
	}

	// Retrieve the type from parameters
	auto topic_names = this->get_parameter("topic_names").as_string_array();
	auto topic_types = this->get_parameter("topic_types").as_string_array();
	type = "";
	for(size_t i = 0; i < topic_names.size(); ++i)
	{
		if(topic_names[i] == topic && i < topic_types.size())
		{
			type = topic_types[i];
			break;
		}
	}

	uint32_t maxDataSize = size;
	if(flags & TCP_FLAG_COMPRESSED)
		maxDataSize = size + size / 100 + 1200;

	m_packet.resize(
		sizeof(TCPHeader) + topic.length() + type.length() + maxDataSize
	);

	TCPHeader* header = (TCPHeader*)m_packet.data();

	uint8_t* wptr = m_packet.data() + sizeof(TCPHeader);

	memcpy(wptr, topic.c_str(), topic.length());
	wptr += topic.length();

	memcpy(wptr, type.c_str(), type.length());
	wptr += type.length();

	if(flags & TCP_FLAG_COMPRESSED)
	{
		unsigned int len = m_packet.size() - (wptr - m_packet.data());

		m_compressionBuf.assign(payload, payload + size);

		if(BZ2_bzBuffToBuffCompress((char*)wptr, &len,
			(char*)m_compressionBuf.data(), m_compressionBuf.size(), 7, 0, 30) == BZ_OK)
		{
			header->data_len = len;
			wptr += len;
			size = len;
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Could not compress with bzip2, sending uncompressed");
			flags &= ~TCP_FLAG_COMPRESSED;
			memcpy(wptr, m_compressionBuf.data(), m_compressionBuf.size());
			header->data_len = m_compressionBuf.size();
			wptr += m_compressionBuf.size();
		}
	}
	else
	{
		memcpy(wptr, payload, size);
		header->data_len = size;
		wptr += size;
	}

	header->topic_len = topic.length();
	header->type_len = type.length();
	header->data_len = size;
	header->flags = flags;

	m_packet.resize(wptr - m_packet.data());

	// Try to send
	for(int tries = 0; tries < 10; ++tries)
	{
		if(m_fd == -1)
		{
			if(!connect())
			{
				RCLCPP_WARN(this->get_logger(), "Connection failed, trying again");
				continue;
			}
		}

		if(write(m_fd, m_packet.data(), m_packet.size()) != (ssize_t)m_packet.size())
		{
			RCLCPP_WARN(this->get_logger(), "Could not send data, trying again");
			close(m_fd);
			m_fd = -1;
			continue;
		}
		m_sentBytesInStatsInterval += m_packet.size();
		m_topicSendBytesInStatsInterval[topic] += m_packet.size();

		// Read ACK
		uint8_t ack;
		if(read(m_fd, &ack, 1) != 1)
		{
			RCLCPP_WARN(this->get_logger(), "Could not read ACK, sending again");
			close(m_fd);
			m_fd = -1;
			continue;
		}

		return;
	}

	RCLCPP_ERROR(this->get_logger(), "Could not send TCP packet. Dropping message from topic %s!",
		topic.c_str());
}

void TCPSender::updateStats()
{
	m_stats.header.stamp = this->now();
	m_stats.bandwidth = 8.0 * m_sentBytesInStatsInterval / m_statsIntervalSec;
	m_stats.topics.clear();

	for(auto& pair : m_topicSendBytesInStatsInterval)
	{
		nimbro_topic_transport::msg::TopicBandwidth tp;
		tp.topic = pair.first;
		tp.bandwidth = pair.second / m_statsIntervalSec;
		pair.second = 0;
		m_stats.topics.push_back(tp);
	}

	m_pub_stats->publish(m_stats);
	m_sentBytesInStatsInterval = 0;
}

}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<nimbro_topic_transport::TCPSender>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
