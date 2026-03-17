// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// Sends a single topic (ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#ifndef TOPIC_SENDER_H
#define TOPIC_SENDER_H

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <vector>
#include <string>

namespace nimbro_topic_transport
{

class UDPSender;

class TopicSender
{
public:
	TopicSender(UDPSender* sender, rclcpp::Node* node,
		const std::string& topic, double rate, bool resend, int flags,
		bool enable, const std::string& type);
	~TopicSender();

	void handleData(std::shared_ptr<rclcpp::SerializedMessage> msg);

	bool isDirectTransmissionEnabled() const
	{ return m_directTransmission; }
	void setDirectTransmissionEnabled(bool value);

	void sendCurrentMessage();
private:
	void send();
	void resend();

	void sendWithFEC();
	void sendWithoutFEC();

	UDPSender* m_sender;
	rclcpp::Node* m_node;
	rclcpp::GenericSubscription::SharedPtr m_subscriber;
	std::string m_topicName;
	std::string m_topicType;
	int m_flags;
	std::vector<uint8_t> m_buf;

	std::vector<uint8_t> m_compressionBuf;
	rclcpp::Duration m_durationBetweenPackets;
	rclcpp::Time m_lastTime;
	rclcpp::TimerBase::SharedPtr m_resendTimer;
	bool m_updateBuf;
	unsigned int m_msgCounter;
	unsigned int m_inputMsgCounter;

	bool m_enabled;
	bool m_directTransmission;

	std::mutex m_dataMutex;
};

}

#endif
