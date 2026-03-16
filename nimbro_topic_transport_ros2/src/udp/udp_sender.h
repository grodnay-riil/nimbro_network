// UDP sender node (ROS2)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef UDP_SENDER_H
#define UDP_SENDER_H

#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <map>
#include <thread>
#include <atomic>

#include <nimbro_topic_transport/msg/sender_stats.hpp>

namespace nimbro_topic_transport
{

class TopicSender;

class UDPSender : public rclcpp::Node
{
public:
	UDPSender();
	~UDPSender();

	uint16_t allocateMessageID();
	bool send(const void* data, uint32_t size, const std::string& topic);

	inline bool duplicateFirstPacket() const
	{ return m_duplicateFirstPacket; }

	inline double fec() const
	{ return m_fec; }
private:
	void relay();
	bool internalSend(const void* data, uint32_t size, const std::string& topic);

	void updateStats();

	uint16_t m_msgID;
	int m_fd;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;
	bool m_duplicateFirstPacket;
	std::vector<TopicSender*> m_senders;

	bool m_relayMode;
	double m_relayRate;
	int m_relayTokensPerStep;
	uint64_t m_relayTokens;

	std::deque<std::vector<uint8_t>> m_relayBuffer;
	std::deque<std::string> m_relayNameBuffer;
	unsigned int m_relayIndex;

	std::thread m_relayThread;
	std::atomic<bool> m_relayThreadShouldExit;

	double m_fec;

	nimbro_topic_transport::msg::SenderStats m_stats;
	rclcpp::Publisher<nimbro_topic_transport::msg::SenderStats>::SharedPtr m_pub_stats;
	rclcpp::TimerBase::SharedPtr m_statsTimer;
	double m_statsIntervalSec;
	uint64_t m_sentBytesInStatsInterval;

	std::map<std::string, uint64_t> m_sentTopicBytesInStatsInterval;
};

}

#endif
