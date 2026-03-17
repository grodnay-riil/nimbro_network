// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// Topic receiver (part of udp_receiver, ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#ifndef TOPIC_RECEIVER_H
#define TOPIC_RECEIVER_H

#include <stdint.h>
#include <stdlib.h>

#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <nimbro_topic_transport/msg/compressed_msg.hpp>

#if WITH_OPENFEC
extern "C"
{
#include <of_openfec_api.h>
}
#endif

#include "udp_packet.h"

namespace nimbro_topic_transport
{

struct Message
{
	Message(uint16_t id)
	 : id(id)
	 , size(0)
	 , complete(false)
	{}

	Message()
	 : id(0), size(0), complete(false)
	{}

	~Message() {}

	uint32_t getLength() const
	{ return size; }

	uint8_t* getData()
	{ return payload.data(); }

	bool decompress(Message* dest);

	uint16_t id;
	UDPFirstPacket::Header header;
	std::vector<uint8_t> payload;
	size_t size;
	std::vector<bool> msgs;

	bool complete;

#if WITH_OPENFEC
	std::shared_ptr<of_session_t> decoder;
	std::shared_ptr<of_parameters_t> params;
	unsigned int received_symbols;
	std::vector<std::shared_ptr<std::vector<uint8_t>>> fecPackets;
#endif
};

struct TopicReceiver
{
	TopicReceiver(rclcpp::Node::SharedPtr node);
	~TopicReceiver();

	rclcpp::GenericPublisher::SharedPtr publisher;
	rclcpp::Publisher<nimbro_topic_transport::msg::CompressedMsg>::SharedPtr compressed_publisher;

	std::string topic_type;
	bool compressed;

	int last_message_counter;

	void takeForDecompression(const std::shared_ptr<Message>& compressed);

	void publish(const uint8_t* cdr_data, size_t cdr_len);
	void publishCompressed(const nimbro_topic_transport::msg::CompressedMsg& msg);

private:
	std::shared_ptr<Message> m_compressedMsg;
	std::condition_variable m_cond;
	std::mutex m_mutex;

	std::thread m_decompressionThread;
	bool m_decompressionThreadRunning;
	std::atomic<bool> m_decompressionThreadShouldExit;

	rclcpp::Node::SharedPtr m_node;

	void decompress();
};

}

#endif
