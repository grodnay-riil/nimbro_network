#!/usr/bin/env python3
"""
Verification node that runs on the listener side.
- Subscribes to /main_chatter and counts messages + measures rate
- Checks that NO noise topics (/chatter1..200) leak through
- Collects nimbro stats from /network/receiver_stats
- Writes results to a JSON file
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time
import os


class VerifyNode(Node):
    def __init__(self, duration=10.0, scenario_name='unknown'):
        super().__init__('verify_node')

        self.duration = duration
        self.scenario_name = scenario_name
        self.start_time = None
        self.msg_count = 0
        self.first_seq = None
        self.last_seq = None
        self.valid_payload = True
        self.expected_payload = '0123456789' * 100

        # Noise leak detection
        self.noise_received = 0

        # Stats from nimbro
        self.last_receiver_stats = None

        # Subscribe to main topic (default RELIABLE QoS matches nimbro's publisher)
        self.main_sub = self.create_subscription(
            String, '/main_chatter', self.main_callback, 10)

        # Subscribe to a sample of noise topics to detect leaks
        for i in [1, 50, 100, 150, 200]:
            self.create_subscription(
                String, f'/chatter{i}', self.noise_callback, 10)

        # Subscribe to nimbro stats
        try:
            from nimbro_topic_transport.msg import ReceiverStats
            self.stats_sub = self.create_subscription(
                ReceiverStats, '/network/receiver_stats',
                self.receiver_stats_callback, 10)
        except Exception:
            self.get_logger().warn('Could not subscribe to ReceiverStats')
            self.stats_sub = None

        # Timer to check if we're done
        self.check_timer = self.create_timer(0.5, self.check_done)

        self.get_logger().info(
            f'Verify node started: scenario={scenario_name}, duration={duration}s')

    def main_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('First message received on /main_chatter')

        self.msg_count += 1

        # Parse seq:payload
        parts = msg.data.split(':', 1)
        if len(parts) == 2:
            try:
                seq = int(parts[0])
            except ValueError:
                return
            payload = parts[1]

            if self.first_seq is None:
                self.first_seq = seq
            self.last_seq = seq

            if payload != self.expected_payload:
                self.valid_payload = False

    def noise_callback(self, msg):
        self.noise_received += 1

    def receiver_stats_callback(self, msg):
        self.last_receiver_stats = {
            'bandwidth': msg.bandwidth,
            'drop_rate': msg.drop_rate,
            'protocol': msg.protocol,
        }

    def check_done(self):
        if self.start_time is None:
            return

        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.msg_count / elapsed if elapsed > 0 else 0

        seq_expected = (self.last_seq - self.first_seq + 1) if (
            self.first_seq is not None and self.last_seq is not None) else 0
        seq_loss = max(0, seq_expected - self.msg_count) if seq_expected > 0 else 0
        msg_loss_pct = (seq_loss / seq_expected * 100) if seq_expected > 0 else 0

        result = {
            'scenario': self.scenario_name,
            'duration_sec': round(elapsed, 1),
            'msg_count': self.msg_count,
            'rate_hz': round(rate, 1),
            'first_seq': self.first_seq,
            'last_seq': self.last_seq,
            'msg_loss_pct': round(msg_loss_pct, 2),
            'payload_valid': self.valid_payload,
            'noise_leaked': self.noise_received,
            'receiver_stats': self.last_receiver_stats,
            'pass': self.msg_count > 0 and self.noise_received == 0 and self.valid_payload,
        }

        self.get_logger().info(f'Results: {json.dumps(result, indent=2)}')

        # Write to file
        out_path = f'/results/{self.scenario_name}.json'
        os.makedirs('/results', exist_ok=True)
        with open(out_path, 'w') as f:
            json.dump(result, f, indent=2)

        self.get_logger().info(f'Results written to {out_path}')

        # Signal done
        self._done = True


def main():
    rclpy.init()

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    scenario = sys.argv[2] if len(sys.argv) > 2 else 'unknown'

    node = VerifyNode(duration=duration, scenario_name=scenario)
    node._done = False

    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
