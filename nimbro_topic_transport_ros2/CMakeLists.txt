cmake_minimum_required(VERSION 3.8)
project(nimbro_topic_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedMsg.msg"
  "msg/SenderStats.msg"
  "msg/ReceiverStats.msg"
  "msg/TopicBandwidth.msg"
  DEPENDENCIES std_msgs
)

# BZ2 compression
find_library(BZ2_LIBRARY bz2 REQUIRED)

# TCP sender
add_executable(tcp_sender
  src/tcp/tcp_sender.cpp
)
ament_target_dependencies(tcp_sender rclcpp)
target_link_libraries(tcp_sender ${BZ2_LIBRARY})
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(tcp_sender "${typesupport_target}")

# TCP receiver
add_executable(tcp_receiver
  src/tcp/tcp_receiver.cpp
)
ament_target_dependencies(tcp_receiver rclcpp)
target_link_libraries(tcp_receiver ${BZ2_LIBRARY})
target_link_libraries(tcp_receiver "${typesupport_target}")

# Install
install(TARGETS tcp_sender tcp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
