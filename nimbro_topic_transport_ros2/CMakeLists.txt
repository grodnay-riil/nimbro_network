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

# Optional: OpenFEC for forward error correction
set(DEFAULT_OPENFEC_PATH /opt/openfec_v1.4.2)
set(OPENFEC_PATH "" CACHE PATH "Path to OpenFEC (optional)")
if(NOT OPENFEC_PATH AND IS_DIRECTORY ${DEFAULT_OPENFEC_PATH})
  set(OPENFEC_PATH ${DEFAULT_OPENFEC_PATH})
endif()
if(OPENFEC_PATH)
  set(OPENFEC_INCLUDE ${OPENFEC_PATH}/src/lib_common)
  set(OPENFEC_LIBRARY ${OPENFEC_PATH}/bin/Release/libopenfec.so)
  set(OPENFEC_DEFINITIONS -DWITH_OPENFEC=1)
  message(STATUS "Found and using OpenFEC at: ${OPENFEC_PATH}")
else()
  set(OPENFEC_INCLUDE "")
  set(OPENFEC_LIBRARY "")
  set(OPENFEC_DEFINITIONS "")
endif()

# UDP sender
add_executable(udp_sender
  src/udp/udp_sender.cpp
  src/udp/topic_sender.cpp
)
ament_target_dependencies(udp_sender rclcpp)
target_link_libraries(udp_sender ${BZ2_LIBRARY} ${OPENFEC_LIBRARY})
target_link_libraries(udp_sender "${typesupport_target}")
target_include_directories(udp_sender PRIVATE ${OPENFEC_INCLUDE})
target_compile_definitions(udp_sender PRIVATE ${OPENFEC_DEFINITIONS})

# UDP receiver
add_executable(udp_receiver
  src/udp/udp_receiver.cpp
  src/udp/topic_receiver.cpp
)
ament_target_dependencies(udp_receiver rclcpp)
target_link_libraries(udp_receiver ${BZ2_LIBRARY} ${OPENFEC_LIBRARY})
target_link_libraries(udp_receiver "${typesupport_target}")
target_include_directories(udp_receiver PRIVATE ${OPENFEC_INCLUDE})
target_compile_definitions(udp_receiver PRIVATE ${OPENFEC_DEFINITIONS})

# Install executables
install(TARGETS tcp_sender tcp_receiver udp_sender udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
