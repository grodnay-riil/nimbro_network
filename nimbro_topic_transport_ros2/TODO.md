# TODO — Future Enhancements

1. **Multi-sender support**: Allow multiple senders to connect to a single receiver instance (currently 1:1 TCP, single-source UDP).

2. **Automatic namespace prefix on egress/ingress**: Sender automatically strips a configurable namespace prefix before transmitting; receiver re-adds it on the other side. Avoids manual topic remapping.

3. **QoS options**: Make publisher/subscriber QoS configurable per-topic (reliability, durability, history depth) instead of hardcoded defaults. Expose as parameters in topic config YAML.

4. **Ignored publishers / message source filtering**: ROS2's `GenericSubscription` doesn't expose `MessageInfo` (publisher GID). To re-implement the ROS1 `ignored_publishers` feature, options are:
   - Use `get_publishers_info_by_topic()` to map GIDs to node names, then filter via `rmw_message_info_t.publisher_gid` in a typed subscription.
   - Or rely on automatic namespace prefixing (TODO #2) to avoid feedback loops entirely, making this feature unnecessary.
