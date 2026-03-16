// Topic receiver (part of udp_receiver, ROS2)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "topic_receiver.h"

#include <bzlib.h>
#include <cstring>

namespace nimbro_topic_transport
{

bool Message::decompress(Message* dest)
{
	unsigned int destLen = 1024;
	dest->payload.resize(destLen);

	while(true)
	{
		int ret = BZ2_bzBuffToBuffDecompress(
			(char*)dest->payload.data(), &destLen,
			(char*)payload.data(), payload.size(), 0, 0);

		if(ret == BZ_OUTBUFF_FULL)
		{
			destLen *= 2;
			dest->payload.resize(destLen);
			continue;
		}

		if(ret != BZ_OK)
		{
			fprintf(stderr, "Could not decompress message\n");
			return false;
		}

		break;
	}

	dest->payload.resize(destLen);
	dest->header = header;
	dest->id = id;
	dest->size = destLen;
	return true;
}

TopicReceiver::TopicReceiver(rclcpp::Node::SharedPtr node)
 : compressed(false)
 , last_message_counter(-1)
 , m_decompressionThreadRunning(false)
 , m_decompressionThreadShouldExit(false)
 , m_node(node)
{
}

TopicReceiver::~TopicReceiver()
{
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_decompressionThreadShouldExit = true;
	}
	m_cond.notify_one();
	if(m_decompressionThread.joinable())
		m_decompressionThread.join();
}

void TopicReceiver::takeForDecompression(const std::shared_ptr<Message>& msg)
{
	if(!m_decompressionThreadRunning)
	{
		m_decompressionThreadShouldExit = false;
		m_decompressionThread = std::thread(&TopicReceiver::decompress, this);
		m_decompressionThreadRunning = true;
	}

	std::unique_lock<std::mutex> lock(m_mutex);
	m_compressedMsg = msg;
	m_cond.notify_one();
}

void TopicReceiver::decompress()
{
	while(true)
	{
		std::shared_ptr<Message> currentMessage;

		{
			std::unique_lock<std::mutex> lock(m_mutex);

			while(!m_compressedMsg && !m_decompressionThreadShouldExit)
				m_cond.wait(lock);

			if(m_decompressionThreadShouldExit)
				return;

			currentMessage = m_compressedMsg;
			m_compressedMsg.reset();
		}

		Message decompressed;
		if(!currentMessage->decompress(&decompressed))
			continue;

		publish(decompressed.payload.data(), decompressed.size);
	}
}

void TopicReceiver::publish(const uint8_t* cdr_data, size_t cdr_len)
{
	if(!publisher)
		return;

	rclcpp::SerializedMessage serialized_msg(cdr_len);
	auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
	memcpy(rcl_msg.buffer, cdr_data, cdr_len);
	rcl_msg.buffer_length = cdr_len;

	publisher->publish(serialized_msg);
}

void TopicReceiver::publishCompressed(const nimbro_topic_transport::msg::CompressedMsg& msg)
{
	if(!compressed_publisher)
		return;

	compressed_publisher->publish(msg);
}

}
