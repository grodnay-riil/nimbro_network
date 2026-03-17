# TODO — Future Enhancements

1. **Multi-sender support**: Allow multiple senders to connect to a single receiver instance (currently 1:1 TCP, single-source UDP).

2. **Automatic namespace prefix on egress/ingress**: Sender automatically strips a configurable namespace prefix before transmitting; receiver re-adds it on the other side. Avoids manual topic remapping.

3. **QoS options**: Make publisher/subscriber QoS configurable per-topic (reliability, durability, history depth) instead of hardcoded defaults. Expose as parameters in topic config YAML.
