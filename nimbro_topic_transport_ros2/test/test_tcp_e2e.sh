#!/bin/bash
# End-to-end test for TCP sender/receiver
set -e

echo "=== Starting tcp_receiver on port 5050 ==="
ros2 run nimbro_topic_transport tcp_receiver \
    --ros-args -p port:=5050 &
RECEIVER_PID=$!
sleep 1

echo "=== Starting tcp_sender → localhost:5050 ==="
ros2 run nimbro_topic_transport tcp_sender \
    --ros-args \
    -p destination_addr:=localhost \
    -p destination_port:=5050 \
    -p "topic_names:=['/chatter']" \
    -p "topic_types:=['std_msgs/msg/String']" &
SENDER_PID=$!
sleep 1

echo "=== Listing topics ==="
ros2 topic list

echo ""
echo "=== Publishing a test message to /chatter ==="
ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'hello from sender'}" &
PUB_PID=$!

echo "=== Waiting for message on /chatter (receiver side re-publishes it) ==="
# The receiver will republish on /chatter — but since we're on the same ROS graph,
# we need to check the receiver's advertised topic. Give it time to discover + republish.
sleep 3

echo ""
echo "=== Checking active topics ==="
ros2 topic list -t

echo ""
echo "=== Echoing /chatter (timeout 5s) ==="
timeout 5 ros2 topic echo /chatter std_msgs/msg/String --once 2>&1 || echo "(timeout — see notes below)"

echo ""
echo "=== Node info ==="
ros2 node info /tcp_sender
echo "---"
ros2 node info /tcp_receiver

echo ""
echo "=== Checking stats topics ==="
timeout 3 ros2 topic echo /network/sender_stats --once 2>&1 || echo "(no sender stats yet)"
timeout 3 ros2 topic echo /network/receiver_stats --once 2>&1 || echo "(no receiver stats yet)"

echo ""
echo "=== Cleanup ==="
kill $SENDER_PID $RECEIVER_PID $PUB_PID 2>/dev/null || true
wait 2>/dev/null

echo "=== Done ==="
