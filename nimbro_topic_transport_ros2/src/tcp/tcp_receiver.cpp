// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// TCP receiver (ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#include "tcp_receiver.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include "tcp_packet.h"

#include <bzlib.h>

#include <nimbro_topic_transport/msg/compressed_msg.hpp>

namespace nimbro_topic_transport
{

static bool sureRead(int fd, void* dest, ssize_t size)
{
	uint8_t* destWPtr = (uint8_t*)dest;
	while(size != 0)
	{
		ssize_t ret = read(fd, destWPtr, size);

		if(ret < 0)
		{
			fprintf(stderr, "Could not read(): %s\n", strerror(errno));
			return false;
		}

		if(ret == 0)
			return false;

		size -= ret;
		destWPtr += ret;
	}

	return true;
}

TCPReceiver::TCPReceiver()
 : Node("tcp_receiver")
 , m_receivedBytesInStatsInterval(0)
{
	RCLCPP_INFO(this->get_logger(), "nimbro_topic_transport v2.0.0 — TCP receiver");
	RCLCPP_INFO(this->get_logger(), "Based on nimbro_network by Max Schwarz, University of Bonn");
	RCLCPP_INFO(this->get_logger(), "ROS2 port by Guy Rodnay, Skana Robotics LTD");

	this->declare_parameter<int>("port", 5050);
	this->declare_parameter<bool>("keep_compressed", false);
	this->declare_parameter<std::string>("label", "");
	this->declare_parameter<std::string>("topic_prefix", "");

	int port = this->get_parameter("port").as_int();

	m_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(m_fd < 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not create socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	int on = 1;
	if(setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) != 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not enable SO_REUSEADDR: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	RCLCPP_DEBUG(this->get_logger(), "Binding to :%d", port);

	if(bind(m_fd, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not bind socket: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	if(listen(m_fd, 10) != 0)
	{
		RCLCPP_FATAL(this->get_logger(), "Could not listen: %s", strerror(errno));
		throw std::runtime_error(strerror(errno));
	}

	m_keepCompressed = this->get_parameter("keep_compressed").as_bool();

	char hostnameBuf[256];
	gethostname(hostnameBuf, sizeof(hostnameBuf));
	hostnameBuf[sizeof(hostnameBuf)-1] = 0;

	m_stats.node = this->get_name();
	m_stats.protocol = "TCP";
	m_stats.host = hostnameBuf;
	m_stats.local_port = port;
	m_stats.fec = false;
	m_stats.label = this->get_parameter("label").as_string();

	m_pub_stats = this->create_publisher<nimbro_topic_transport::msg::ReceiverStats>(
		"/network/receiver_stats", 1);

	m_statsIntervalSec = 2.0;
	m_statsTimer = this->create_wall_timer(
		std::chrono::milliseconds(static_cast<int>(m_statsIntervalSec * 1000)),
		std::bind(&TCPReceiver::updateStats, this)
	);

	m_topicPrefix = this->get_parameter("topic_prefix").as_string();

	RCLCPP_INFO(this->get_logger(), "Configuration: port=%d, keep_compressed=%s, topic_prefix='%s'",
		port, m_keepCompressed ? "true" : "false", m_topicPrefix.c_str());
}

TCPReceiver::~TCPReceiver()
{
	for(auto* handler : m_handlers)
		delete handler;
	if(m_fd >= 0)
		close(m_fd);
}

void TCPReceiver::run()
{
	fd_set fds;

	while(rclcpp::ok())
	{
		rclcpp::spin_some(shared_from_this());

		// Clean up exited client threads
		auto it = m_handlers.begin();
		while(it != m_handlers.end())
		{
			if(!(*it)->isRunning())
			{
				delete *it;
				it = m_handlers.erase(it);
			}
			else
				++it;
		}

		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);

		timeval timeout;
		timeout.tv_usec = 0;
		timeout.tv_sec = 1;

		int ret = select(m_fd+1, &fds, nullptr, nullptr, &timeout);
		if(ret < 0)
		{
			if(errno == EINTR || errno == EAGAIN)
				continue;

			RCLCPP_ERROR(this->get_logger(), "Could not select(): %s", strerror(errno));
			throw std::runtime_error("Could not select");
		}
		if(ret == 0)
			continue;

		sockaddr_storage remoteAddr;
		socklen_t remoteAddrLen = sizeof(remoteAddr);

		int client_fd = accept(m_fd, (sockaddr*)&remoteAddr, &remoteAddrLen);

		{
			char nameBuf[256];
			char serviceBuf[256];

			auto startLookup = std::chrono::steady_clock::now();
			if(getnameinfo((sockaddr*)&remoteAddr, remoteAddrLen, nameBuf, sizeof(nameBuf),
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
		}

		auto handler = new ClientHandler(client_fd, m_topicPrefix, shared_from_this());
		handler->setKeepCompressed(m_keepCompressed);

		m_handlers.push_back(handler);
	}
}

TCPReceiver::ClientHandler::ClientHandler(int fd, const std::string& topicPrefix,
	rclcpp::Node::SharedPtr node)
 : m_fd(fd)
 , m_uncompressBuf(1024)
 , m_running(true)
 , m_keepCompressed(false)
 , m_bytesReceived(0)
 , m_topicPrefix(topicPrefix)
 , m_node(node)
{
	m_thread = std::thread(&ClientHandler::run, this);
	m_thread.detach();
}

TCPReceiver::ClientHandler::~ClientHandler()
{
	close(m_fd);
}

void TCPReceiver::ClientHandler::run()
{
	while(true)
	{
		TCPHeader header;

		if(!sureRead(m_fd, &header, sizeof(header)))
		{
			m_running = false;
			return;
		}

		std::vector<char> buf(header.topic_len + 1);
		if(!sureRead(m_fd, buf.data(), header.topic_len))
		{
			m_running = false;
			return;
		}
		buf[buf.size()-1] = 0;
		std::string topic(buf.data());

		buf.resize(header.type_len + 1);
		if(!sureRead(m_fd, buf.data(), header.type_len))
		{
			m_running = false;
			return;
		}
		buf[buf.size()-1] = 0;
		std::string type(buf.data());

		std::vector<uint8_t> data(header.data_len);
		if(!sureRead(m_fd, data.data(), header.data_len))
		{
			m_running = false;
			return;
		}

		m_bytesReceived += sizeof(header) + header.topic_len + header.type_len + header.data_len;

		RCLCPP_DEBUG(m_node->get_logger(), "Got msg with flags: %d", header.flags());

		if(m_keepCompressed && (header.flags() & TCP_FLAG_COMPRESSED))
		{
			nimbro_topic_transport::msg::CompressedMsg compressed;
			compressed.type = type;
			compressed.data.swap(data);

			auto it = m_compressedPub.find(topic);
			if(it == m_compressedPub.end())
			{
				auto pub = m_node->create_publisher<nimbro_topic_transport::msg::CompressedMsg>(
					m_topicPrefix + topic, 2);
				m_compressedPub[topic] = pub;
				pub->publish(compressed);
			}
			else
				it->second->publish(compressed);
		}
		else
		{
			const uint8_t* cdr_data = nullptr;
			uint32_t cdr_len = 0;

			if(header.flags() & TCP_FLAG_COMPRESSED)
			{
				int ret = 0;
				unsigned int len = m_uncompressBuf.size();

				while(true)
				{
					ret = BZ2_bzBuffToBuffDecompress(
						(char*)m_uncompressBuf.data(), &len,
						(char*)data.data(), data.size(), 0, 0);

					if(ret == BZ_OUTBUFF_FULL)
					{
						len = 4 * m_uncompressBuf.size();
						RCLCPP_INFO(m_node->get_logger(),
							"Increasing buffer size to %d KiB", (int)len / 1024);
						m_uncompressBuf.resize(len);
						continue;
					}
					else
						break;
				}

				if(ret != BZ_OK)
				{
					RCLCPP_ERROR(m_node->get_logger(),
						"Could not decompress bz2 data (reason %d), dropping msg", ret);
					continue;
				}

				cdr_data = m_uncompressBuf.data();
				cdr_len = len;
			}
			else
			{
				cdr_data = data.data();
				cdr_len = data.size();
			}

			RCLCPP_DEBUG(m_node->get_logger(),
				"Got message from topic '%s' (type '%s')", topic.c_str(), type.c_str());

			// Create publisher if needed
			auto it = m_pub.find(topic);
			if(it == m_pub.end())
			{
				RCLCPP_INFO(m_node->get_logger(),
					"Advertising new topic '%s' (type '%s')",
					(m_topicPrefix + topic).c_str(), type.c_str());

				auto pub = m_node->create_generic_publisher(
					m_topicPrefix + topic, type, rclcpp::QoS(10));

				m_pub[topic] = pub;
				it = m_pub.find(topic);

				// Give subscribers time to discover
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}

			// Publish CDR bytes directly
			rclcpp::SerializedMessage serialized_msg(cdr_len);
			auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
			memcpy(rcl_msg.buffer, cdr_data, cdr_len);
			rcl_msg.buffer_length = cdr_len;

			it->second->publish(serialized_msg);
		}

		uint8_t ack = 1;
		if(write(m_fd, &ack, 1) != 1)
		{
			RCLCPP_ERROR(m_node->get_logger(), "Could not write(): %s", strerror(errno));
			m_running = false;
			return;
		}
	}
}

bool TCPReceiver::ClientHandler::isRunning() const
{
	return m_running;
}

void TCPReceiver::updateStats()
{
	m_stats.header.stamp = this->now();

	uint64_t totalBytes = 0;
	for(auto handler : m_handlers)
	{
		totalBytes += handler->bytesReceived();
		handler->resetByteCounter();
	}

	m_stats.bandwidth = totalBytes / m_statsIntervalSec;
	m_stats.drop_rate = 0;

	if(m_handlers.empty())
		return;

	m_pub_stats->publish(m_stats);
}

}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<nimbro_topic_transport::TCPReceiver>();

	node->run();

	rclcpp::shutdown();

	return 0;
}
