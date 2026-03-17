// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// TCP sender (ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#ifndef TCP_SENDER_H
#define TCP_SENDER_H

#include <rclcpp/rclcpp.hpp>
#include <arpa/inet.h>

#include "tcp_packet.h"

#include <map>
#include <vector>
#include <string>

#include <nimbro_topic_transport/msg/sender_stats.hpp>

namespace nimbro_topic_transport
{

class TCPSender : public rclcpp::Node
{
public:
	TCPSender();
	~TCPSender();

	bool connect();

	void send(const std::string& topic, int flags,
		const std::shared_ptr<rclcpp::SerializedMessage>& msg);
private:
	void updateStats();

	int m_fd;

	int m_addrFamily;
	sockaddr_storage m_addr;
	socklen_t m_addrLen;

	int m_sourcePort;
	std::vector<rclcpp::GenericSubscription::SharedPtr> m_subs;
	std::vector<uint8_t> m_packet;
	std::vector<uint8_t> m_compressionBuf;

	nimbro_topic_transport::msg::SenderStats m_stats;
	rclcpp::Publisher<nimbro_topic_transport::msg::SenderStats>::SharedPtr m_pub_stats;
	rclcpp::TimerBase::SharedPtr m_statsTimer;
	double m_statsIntervalSec;
	uint64_t m_sentBytesInStatsInterval;
	std::map<std::string, uint64_t> m_topicSendBytesInStatsInterval;
};

}

#endif
