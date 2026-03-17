# E2E Test Plan: Docker Compose with Traffic Shaping

## Infrastructure

Two containers on a custom Docker bridge network:
- `talker` (hostname: `talker`) — runs sender + 201 publishers
- `listener` (hostname: `listener`) — runs receiver + verification

Each container is its own ROS2 island. Nimbro's raw TCP/UDP sockets are the only bridge.

**Ports:** `17001/tcp`, `17002/udp` on listener.

**Nimbro stats topics:**
- `/network/sender_stats` (on talker) — bandwidth, per-topic bandwidth
- `/network/receiver_stats` (on listener) — bandwidth, drop_rate

**Extra tools:** iftop, iproute2 (tc), iputils-ping, net-tools

## Traffic Pattern

- **200 noise topics** `/chatter1`..`/chatter200` at 10 Hz each on talker — NOT forwarded.
- **1 main topic** `/main_chatter` with 1KB payload at 100 Hz publish, forwarded at 50 Hz relay.
- Verification on listener: only `/main_chatter` arrives at ~50 Hz with correct data.

## Scenarios

| # | Protocol | Compress | Loss | FEC | Notes |
|---|----------|----------|------|-----|-------|
| 1 | UDP | no | 0% | no | Baseline |
| 2 | TCP | no | 0% | no | Baseline |
| 3 | UDP | yes | 0% | no | Compression savings |
| 4 | TCP | yes | 0% | no | Compression savings |
| 5 | UDP | no | 2% | no | Loss impact |
| 6 | UDP | yes | 2% | no | Loss + compression |
| 7 | UDP | no | 2% | yes | FEC recovery |
| 8 | UDP | yes | 2% | yes | FEC + compression |

## Per-Scenario Measurements

- Message count received in 10 seconds
- Effective receive rate (Hz)
- Sender bandwidth (from sender_stats)
- Receiver bandwidth (from receiver_stats)
- Drop rate (from receiver_stats, UDP only)
- Pass/fail: main_chatter arrived? Noise topics did NOT leak?
