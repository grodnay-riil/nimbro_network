// Copyright (c) 2015, University of Bonn, Autonomous Intelligent Systems.
// Copyright (c) 2026, Skana Robotics LTD.
// All rights reserved. BSD 3-Clause License — see LICENSE file.
//
// TCP packet definition (ROS2)
// Original author: Max Schwarz <max.schwarz@uni-bonn.de>
// ROS2 port: Guy Rodnay, Skana Robotics LTD <grodnay@skanarobotics.com>

#ifndef TCP_PACKET_H
#define TCP_PACKET_H

#include "../le_value.h"

namespace nimbro_topic_transport
{

enum TCPFlag
{
	TCP_FLAG_COMPRESSED = (1 << 0)
};

struct TCPHeader
{
	LEValue<2> topic_len;
	LEValue<2> type_len;
	LEValue<4> data_len;
	LEValue<4> flags;
} __attribute__((packed));

}

#endif
