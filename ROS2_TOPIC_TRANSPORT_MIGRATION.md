# Migrating nimbro_topic_transport to Native ROS2

## Scope

Port **only** `nimbro_topic_transport` to ROS2. No cross-compatibility with ROS1 senders/receivers is required — the wire protocol can change freely.

## Target Platform

- **ROS2 Jazzy** (Ubuntu 24.04 Noble)
- Build and test in Docker (`ros:jazzy` base image)
- C++17 (Jazzy default)

---

## Current Architecture

The package implements transparent topic relay over unreliable networks, with UDP (streaming) and TCP (guaranteed delivery) variants.

**Data flow:**

```
Local Publisher → ros::Subscriber (ShapeShifter) → Serialize → Compress (optional) → UDP/TCP socket
                                                                                          ↓
Local Subscriber ← ros::Publisher (dynamic) ← Deserialize ← Decompress (optional) ← UDP/TCP socket
```

**Source files involved:**

| File | Role |
|------|------|
| `src/udp/udp_sender.cpp/.h` | UDP sender node: socket setup, topic list parsing, relay mode, stats |
| `src/udp/topic_sender.cpp/.h` | Per-topic logic: subscribe, serialize, compress, fragment, send |
| `src/udp/udp_receiver.cpp/.h` | UDP receiver node: socket listen, packet assembly, FEC decode, stats |
| `src/udp/topic_receiver.cpp/.h` | Per-topic logic: decompress, deserialize, publish |
| `src/udp/udp_packet.h` | UDP packet structs (`UDPFirstPacket`, `UDPDataPacket`, `FECPacket`, `FECHeader`) |
| `src/tcp/tcp_sender.cpp/.h` | TCP sender node: connect, serialize, compress, send with ACK |
| `src/tcp/tcp_receiver.cpp/.h` | TCP receiver node: accept, read, decompress, deserialize, publish |
| `src/tcp/tcp_packet.h` | TCP packet struct (`TCPHeader`) |
| `src/topic_info.cpp/.h` | Message metadata: forks `rosmsg md5`/`rosmsg show` to get MD5 and definitions |
| `src/le_value.h` | Little-endian value template (pure C++, no ROS dependency) |
| `msg/CompressedMsg.msg` | Custom message for keeping data compressed on the local bus |
| `msg/SenderStats.msg` | Bandwidth/connection stats published by senders |
| `msg/ReceiverStats.msg` | Bandwidth/connection stats published by receivers |
| `msg/TopicBandwidth.msg` | Per-topic bandwidth sub-message |

---

## What Stays Unchanged

These components have **zero ROS dependency** and can be reused as-is:

- **Socket I/O** — all `socket()`, `sendto()`, `recvfrom()`, `connect()`, `accept()`, `select()` calls are pure POSIX
- **BZ2 compression/decompression** — direct `libbz2` calls
- **FEC encoding/decoding** — OpenFEC library calls
- **`le_value.h`** — pure C++ template for little-endian wire values
- **Packet fragmentation logic** — `sendWithoutFEC()` / `sendWithFEC()` / `handleMessagePacket()` work on raw byte buffers
- **Relay mode** — token bucket + buffer, no ROS API involvement
- **Connection retry logic** (TCP sender) — pure socket reconnection

---

## ROS1 API Touchpoints That Must Change

### 1. Generic Subscribe: `topic_tools::ShapeShifter`

**Current (ROS1):**
```cpp
// topic_sender.cpp:42-52
ros::SubscribeOptions ops;
boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> func =
    boost::bind(&TopicSender::handleData, this, _1);
ops.initByFullCallbackType(topic, 20, func);
ops.datatype = type;
ops.md5sum = topic_info::getMd5Sum(type);
m_subscriber = nh->subscribe(ops);
```

Also in `tcp_sender.cpp:82-103`, same pattern with `ros::MessageEvent<ShapeShifter const>`.

**ROS2 replacement: `rclcpp::GenericSubscription`**

```cpp
auto subscription = node->create_generic_subscription(
    topic,
    type,  // e.g. "sensor_msgs/msg/Image"
    qos,
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        handleData(msg);
    }
);
```

Key differences:
- Callback receives `rclcpp::SerializedMessage` — raw CDR bytes, not a ShapeShifter
- No MD5 sum needed for subscription
- Type string uses ROS2 format: `"pkg/msg/Type"` instead of `"pkg/Type"`
- The serialized bytes are already in CDR format — can be sent over the wire directly

### 2. Message Serialization (Sender)

**Current (ROS1):**
```cpp
// topic_sender.cpp:97-98
m_buf.resize(m_lastData->size());
m_lastData->write(*this);  // ShapeShifter writes ROS1-serialized bytes to m_buf

// tcp_sender.cpp:300-301
PtrStream stream(wptr);
shifter->write(stream);
```

**ROS2 replacement:**

With `GenericSubscription`, the callback already provides CDR-serialized bytes:

```cpp
void handleData(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    auto& rcl_msg = msg->get_rcl_serialized_message();
    // rcl_msg.buffer contains the CDR bytes
    // rcl_msg.buffer_length is the size
    // Copy directly to send buffer — no serialization step needed
    m_buf.assign(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}
```

This is actually **simpler** than the ROS1 code — no custom stream classes (`PtrStream`) needed.

### 3. Dynamic Publish (Receiver)

**Current (ROS1):**
```cpp
// udp_receiver.cpp:220-232
ros::AdvertiseOptions options(
    m_topicPrefix + header->topic_name,
    1,
    topic->md5_str,
    header->topic_type,
    topic->msg_def,
    boost::bind(&TopicReceiver::handleSubscriber, topic)
);
topic->publisher = m_nh.advertise(options);

// then later:
shapeShifter->morph(md5_str, type, msg_def, "");
shapeShifter->read(decompressed);  // deserialize from raw bytes
publisher.publish(shapeShifter);
```

Also in `tcp_receiver.cpp:330-359`, same pattern.

**ROS2 replacement: `rclcpp::GenericPublisher`**

```cpp
auto publisher = node->create_generic_publisher(
    topic_prefix + topic_name,
    type,  // "pkg/msg/Type"
    qos
);

// Publish raw CDR bytes directly — no deserialization needed
auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
serialized_msg->reserve(data_length);
memcpy(
    serialized_msg->get_rcl_serialized_message().buffer,
    payload_data, data_length
);
serialized_msg->get_rcl_serialized_message().buffer_length = data_length;
publisher->publish(*serialized_msg);
```

Again **simpler** — no ShapeShifter morph/read, no message definition lookup, no MD5 verification. The CDR bytes go straight through.

### 4. `topic_info.cpp` — Message Metadata Lookup

**Current (ROS1):**
```cpp
// topic_info.cpp:51 — forks "rosmsg md5 <type>" subprocess
execlp("rosmsg", "rosmsg", cmd.c_str(), type.c_str(), 0);

// Used for:
// - MD5 sums: sender subscription matching, receiver type verification
// - Message definitions: receiver advertise options, ShapeShifter morph
```

**ROS2 replacement:**

With `GenericSubscription`/`GenericPublisher` and CDR pass-through, **most of `topic_info.cpp` becomes unnecessary**:

- **MD5 sums**: ROS2 does not use MD5 sums. Type matching uses type name + type hash from `rosidl`. Since we transmit CDR bytes directly and both sides must have the same message packages installed, the type name string is sufficient for verification.
- **Message definitions**: Not needed for `GenericPublisher` — it only needs the type name string.

`topic_info.cpp` can be **deleted entirely** or reduced to a simple type name validation utility.

### 5. Wire Protocol: Packet Headers

**Current packet headers carry MD5 sums:**

```cpp
// udp_packet.h — UDPFirstPacket::Header
LEValue<4> topic_md5[4];  // 16 bytes of MD5 sum

// tcp_packet.h — TCPHeader
LEValue<4> topic_md5sum[4];  // 16 bytes of MD5 sum
```

**ROS2 change:**

Two options:

**Option A (recommended): Replace MD5 with type hash**
```cpp
// New: use rosidl type hash (SHA-256 based, 32 bytes)
// Available via rosidl_typesupport_cpp::get_message_type_support_handle()
uint8_t type_hash[32];
```

**Option B (simpler): Drop type verification from wire protocol**

Since both sender and receiver must have the same ROS2 message packages installed, the type name string (already in the header) is sufficient. Remove the MD5 fields entirely and save 16 bytes per packet.

The `topic_md5` fields in `UDPFirstPacket::Header`, `FECHeader`, and `TCPHeader` can be replaced or removed. Either way, this is a wire-protocol-breaking change (acceptable since we don't need ROS1 compatibility).

### 6. Node Lifecycle and Parameters

**Current (ROS1):**
```cpp
// Initialization
ros::init(argc, argv, "udp_sender");
ros::NodeHandle nh("~");
nh.param("destination_addr", dest_host, std::string("localhost"));
nh.param("destination_port", dest_port, 5050);

// Topic config via XmlRpc
XmlRpc::XmlRpcValue list;
nh.getParam("topics", list);  // parsed from YAML via rosparam

// Timers
m_statsTimer = nh.createWallTimer(m_statsInterval, callback);
m_resendTimer = nh.createTimer(duration, callback);

// Spin
ros::spin();  // or manual ros::spinOnce() in event loop
```

**ROS2 replacement:**
```cpp
// Initialization
rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("udp_sender");
node->declare_parameter("destination_addr", "localhost");
auto dest_host = node->get_parameter("destination_addr").as_string();

// Topic config — ROS2 supports structured parameters natively
// YAML loaded via launch file, accessed as:
auto topics = node->get_parameter("topics").as_string_array();
// Or use nested parameters: topics.0.name, topics.0.compress, etc.

// Timers
auto timer = node->create_wall_timer(2s, callback);

// Spin
rclcpp::spin(node);  // or manual rclcpp::spin_some(node) in event loop
```

**Topic config YAML** stays almost identical; just the loading mechanism changes (launch file `parameters` directive instead of `rosparam load`).

**XmlRpc topic list parsing** needs rework — ROS2 doesn't have `XmlRpc::XmlRpcValue`. Use ROS2's structured parameter API or parse YAML directly (e.g., with `yaml-cpp`, already a dependency for the bandwidth GUI).

### 7. Logging

**Current:** `ROS_INFO(...)`, `ROS_WARN(...)`, `ROS_ERROR(...)`, `ROS_FATAL(...)`, `ROS_DEBUG(...)`

**ROS2:** `RCLCPP_INFO(node->get_logger(), ...)`, etc.

This is a mechanical find-and-replace. Consider a macro shorthand:
```cpp
#define LOG_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
```

### 8. Time API

**Current:**
```cpp
ros::Time::now()
ros::WallTime::now()
ros::Duration(1.0)
ros::WallDuration(2.0)
ros::WallRate(100.0)
m_statsInterval.toSec()
```

**ROS2:**
```cpp
node->now()                    // or rclcpp::Clock().now()
rclcpp::Clock(RCL_STEADY_TIME).now()  // wall time equivalent
rclcpp::Duration::from_seconds(1.0)
std::chrono::seconds(2)        // for wall timers
rclcpp::WallRate(100.0)
m_statsInterval.seconds()
```

### 9. Threading

**Current:** `boost::thread`, `boost::mutex`, `boost::condition_variable`, `boost::bind`

**ROS2 (C++17):** `std::thread`, `std::mutex`, `std::condition_variable`, `std::bind` or lambdas

Mechanical replacement. The threading model is the same.

### 10. Build System

**Current (`CMakeLists.txt`):**
```cmake
find_package(catkin REQUIRED COMPONENTS roscpp topic_tools message_generation)
add_message_files(FILES CompressedMsg.msg ...)
generate_messages(DEPENDENCIES std_msgs)
catkin_package()
```

**ROS2:**
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedMsg.msg"
  "msg/SenderStats.msg"
  "msg/ReceiverStats.msg"
  "msg/TopicBandwidth.msg"
  DEPENDENCIES std_msgs
)

ament_target_dependencies(udp_sender rclcpp)
ament_package()
```

**`package.xml`** changes from format 2 to format 3:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>rclcpp</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### 11. Launch Files

**Current:** `.launch` XML files with `rosparam load`

**ROS2:** `.launch.py` Python files (or `.launch.xml` / `.launch.yaml`)

```python
# launch/udp_sender.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nimbro_topic_transport',
            executable='udp_sender',
            name='udp_sender',
            parameters=[
                'config/topics.yaml',
                {'destination_addr': 'remote_host', 'destination_port': 17001}
            ],
        ),
    ])
```

### 12. `ros::MessageEvent` (TCP sender only)

**Current:**
```cpp
// tcp_sender.cpp:82-83
boost::function<void(const ros::MessageEvent<topic_tools::ShapeShifter const>&)> func;
// Used to access event.getConnectionHeader()["callerid"] for ignored_publishers filtering
```

**ROS2:** `rclcpp::MessageInfo` provides similar metadata, but `GenericSubscription` callbacks don't include it. The `ignored_publishers` feature would need reimplementation — possibly by checking `rmw_message_info_t` or by filtering at the subscription level using content filtering or intra-process checks.

Alternatively, drop this feature or replace it with a topic remapping approach.

---

## Simplified Data Path with ROS2

Because `GenericSubscription`/`GenericPublisher` work with raw CDR bytes, the ROS2 data path is simpler:

```
ROS2 Publisher
    ↓
GenericSubscription callback → receives SerializedMessage (CDR bytes)
    ↓
Copy CDR bytes to send buffer (NO serialization needed)
    ↓
Optional BZ2 compress
    ↓
Add packet header (topic name, type string, flags)
    ↓
UDP sendto() / TCP write()
    ↓ (network)
    ↓
UDP recvfrom() / TCP read()
    ↓
Strip packet header
    ↓
Optional BZ2 decompress
    ↓
Wrap CDR bytes in SerializedMessage (NO deserialization needed)
    ↓
GenericPublisher.publish(serialized_msg)
    ↓
ROS2 Subscriber
```

Compared to ROS1, two costly steps are eliminated:
- **Sender**: no `ShapeShifter::write()` re-serialization
- **Receiver**: no `ShapeShifter::read()` + `morph()` + message definition lookup

---

## Migration Order

Recommended order, starting with the simplest path to a working system:

### Phase 1: TCP (simpler, no fragmentation)

1. **tcp_sender.cpp** — Replace `ros::NodeHandle`, params, `ShapeShifter` subscription with `rclcpp::Node` + `GenericSubscription`. Replace `ShapeShifter::write()` with direct CDR byte copy. Update `TCPHeader` (drop or replace MD5 fields).
2. **tcp_receiver.cpp** — Replace dynamic `ros::Publisher` with `GenericPublisher`. Remove `ShapeShifter::morph()`/`read()`, publish CDR bytes directly. Replace `boost::thread` with `std::thread`.
3. **tcp_packet.h** — Update `TCPHeader` struct.
4. Delete or gut `topic_info.cpp` (no longer needed for TCP path).

### Phase 2: UDP (adds fragmentation + FEC complexity)

5. **topic_sender.cpp** — Same subscription change as TCP. Update `UDPFirstPacket` header fields.
6. **udp_sender.cpp** — Replace `ros::NodeHandle`, params, XmlRpc topic list parsing.
7. **udp_receiver.cpp** — Replace dynamic publishers, remove ShapeShifter usage. Update packet header parsing.
8. **topic_receiver.cpp** — Replace `boost::thread`/`boost::mutex` with `std::thread`/`std::mutex`. Remove ShapeShifter from decompression path.
9. **udp_packet.h** — Update `UDPFirstPacket::Header` and `FECHeader` (drop/replace MD5).

### Phase 3: Build + Config

10. **CMakeLists.txt** — catkin → ament_cmake, message generation → rosidl.
11. **package.xml** — Update to format 3 with ROS2 dependencies.
12. **Launch files** — Convert to `.launch.py`.
13. **msg/*.msg** — Should work as-is with `rosidl_generate_interfaces()` (verify `std_msgs` dependency resolves).

### Phase 4: Optional Cleanup

14. Remove `topic_info.cpp/.h` entirely.
15. Remove `boost` dependency where `std::` equivalents suffice.
16. Replace `XmlRpc` topic config parsing with `yaml-cpp` or ROS2 structured parameters.
17. Decide fate of `config_server` integration (drop or replace with ROS2 dynamic reconfigure).
18. Port GUI plugins (separate effort — Qt4 → Qt5, RViz plugin API changed).

---

## GUI: Dropped

The Qt4-based `rqt` plugins (`TopicGUI`, `BandwidthGui`) are **not being ported**. They were read-only stats visualizations. Use `ros2 topic echo`, `rqt_plot`, or Foxglove Studio instead.

**Files excluded from ROS2 port:**
- `src/gui/` (entire directory)
- All Qt4/rqt build rules from `CMakeLists.txt`
- Any `plugin.xml` for rqt plugin registration

---

## Risks and Considerations

| Risk | Mitigation |
|------|------------|
| CDR overhead larger than ROS1 serialization for some types | Benchmark; BZ2 compression already handles large messages |
| `GenericSubscription` QoS must match publisher | Use `rclcpp::SystemDefaultsQoS()` or make QoS configurable per topic |
| ROS2 structured parameter API is more verbose than XmlRpc | Use `yaml-cpp` to parse topic list YAML directly, or flatten to `topics.0.name`, `topics.0.compress`, etc. |
| `ignored_publishers` feature (TCP) harder to implement | Low priority; can defer or drop |
| OpenFEC library availability | Same as ROS1 — optional compile-time feature, no change needed |
| `config_server` (NimbRo-specific) | Drop or replace with ROS2 parameter events for runtime enable/disable |

---

## Summary of Effort

| Category | Estimated Change |
|----------|-----------------|
| Lines of ROS API to replace | ~200 lines across 8 source files |
| Lines of socket/compression code unchanged | ~800+ lines |
| New code (build, launch) | ~100 lines |
| Code to delete (`topic_info.cpp`, ShapeShifter glue) | ~120 lines |
| Net complexity change | **Simpler** — CDR pass-through eliminates serialize/deserialize steps |
