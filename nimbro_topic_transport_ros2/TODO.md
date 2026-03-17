# TODO — Future Enhancements

1. **Multi-sender support**: Allow multiple senders to connect to a single receiver instance (currently 1:1 TCP, single-source UDP).

2. **Automatic namespace prefix on egress/ingress**: Sender automatically strips a configurable namespace prefix before transmitting; receiver re-adds it on the other side. Avoids manual topic remapping.

3. **QoS options**: Make publisher/subscriber QoS configurable per-topic (reliability, durability, history depth) instead of hardcoded defaults. Expose as parameters in topic config YAML.

4. **Ignored publishers / message source filtering**: ROS2's `GenericSubscription` doesn't expose `MessageInfo` (publisher GID). To re-implement the ROS1 `ignored_publishers` feature, options are:
   - Use `get_publishers_info_by_topic()` to map GIDs to node names, then filter via `rmw_message_info_t.publisher_gid` in a typed subscription.
   - Or rely on automatic namespace prefixing (TODO #2) to avoid feedback loops entirely, making this feature unnecessary.

5. **Runtime enable/disable per topic**: The ROS1 version used `config_server` to toggle individual topics on/off at runtime without restarting the node. ROS2 equivalent: use dynamic parameter callbacks (`add_on_set_parameters_callback`) with a `topic_enable` parameter array, or expose a service per topic.

6. **Action proxy**: Forward ROS2 actions (goal/result/feedback) over nimbro. ROS2 actions use 3 services + 2 topics under the hood, so this requires bidirectional service forwarding. Best to implement alongside the `nimbro_service_transport` ROS2 migration — the service proxy infrastructure is a prerequisite for action forwarding.
