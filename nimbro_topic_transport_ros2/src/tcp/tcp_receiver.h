// TCP receiver (ROS2)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef TCP_RECEIVER_H
#define TCP_RECEIVER_H

#include <list>
#include <map>
#include <string>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <nimbro_topic_transport/msg/compressed_msg.hpp>
#include <nimbro_topic_transport/msg/receiver_stats.hpp>

namespace nimbro_topic_transport
{

class TCPReceiver : public rclcpp::Node
{
public:
	TCPReceiver();
	~TCPReceiver();

	void run();
private:
	class ClientHandler
	{
	public:
		ClientHandler(int fd, const std::string& topicPrefix,
			rclcpp::Node::SharedPtr node);
		~ClientHandler();
		void run();

		void setKeepCompressed(bool keep)
		{ m_keepCompressed = keep; }

		bool isRunning() const;

		inline uint64_t bytesReceived() const
		{ return m_bytesReceived; }

		inline void resetByteCounter()
		{ m_bytesReceived = 0; }
	private:
		int m_fd;
		std::thread m_thread;
		std::map<std::string, rclcpp::GenericPublisher::SharedPtr> m_pub;
		std::map<std::string, rclcpp::Publisher<nimbro_topic_transport::msg::CompressedMsg>::SharedPtr> m_compressedPub;
		std::vector<uint8_t> m_uncompressBuf;
		std::atomic<bool> m_running;
		bool m_keepCompressed;
		uint64_t m_bytesReceived;
		std::string m_topicPrefix;
		rclcpp::Node::SharedPtr m_node;
	};

	void updateStats();

	int m_fd;
	std::list<ClientHandler*> m_handlers;

	bool m_keepCompressed;

	nimbro_topic_transport::msg::ReceiverStats m_stats;
	uint64_t m_receivedBytesInStatsInterval;
	rclcpp::Publisher<nimbro_topic_transport::msg::ReceiverStats>::SharedPtr m_pub_stats;
	rclcpp::TimerBase::SharedPtr m_statsTimer;
	double m_statsIntervalSec;

	std::string m_topicPrefix;
};

}

#endif
