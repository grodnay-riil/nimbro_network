// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// UDP receiver node (ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <map>
#include <vector>
#include <string>
#include <list>
#include <memory>

#include <sys/socket.h>

#include <rclcpp/rclcpp.hpp>

#include <nimbro_topic_transport/msg/receiver_stats.hpp>
#include <nimbro_topic_transport/msg/compressed_msg.hpp>

#include "udp_packet.h"
#include "topic_receiver.h"

namespace nimbro_topic_transport
{

class UDPReceiver : public rclcpp::Node
{
public:
	UDPReceiver();
	~UDPReceiver();

	void run();
private:
	typedef std::map<std::string, std::shared_ptr<TopicReceiver>> TopicMap;
	typedef std::list<Message> MessageBuffer;

	void handleMessagePacket(MessageBuffer::iterator it, std::vector<uint8_t>* buf, std::size_t size);

	template<class HeaderType>
	void handleFinishedMessage(Message* msg, HeaderType* header);

	void pruneMessages();

	void updateStats();

	int m_fd;
	MessageBuffer m_incompleteMessages;
	TopicMap m_topics;

	bool m_dropRepeatedMessages;
	bool m_warnDropIncomplete;
	bool m_keepCompressed;

	bool m_fec;

	nimbro_topic_transport::msg::ReceiverStats m_stats;
	uint64_t m_receivedBytesInStatsInterval;
	uint64_t m_expectedPacketsInStatsInterval;
	uint64_t m_missingPacketsInStatsInterval;
	rclcpp::Publisher<nimbro_topic_transport::msg::ReceiverStats>::SharedPtr m_pub_stats;
	rclcpp::TimerBase::SharedPtr m_statsTimer;
	double m_statsIntervalSec;

	sockaddr_storage m_remoteAddr;
	socklen_t m_remoteAddrLen;

	std::string m_topicPrefix;
};

}

#endif
