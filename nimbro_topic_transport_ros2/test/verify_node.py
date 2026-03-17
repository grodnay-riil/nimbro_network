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
        self.wall_start = time.time()  # for no-message timeout
        self.msg_count = 0
        self.seq_numbers = []
        self.valid_payload = True
        self.expected_payload = '0123456789' * 100
        self._done = False

        # Noise leak detection
        self.noise_received = 0

        # Stats from nimbro — collect multiple samples
        self.receiver_stats_samples = []

        # Subscribe to main topic (RELIABLE — matches nimbro receiver's QoS)
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
                self.seq_numbers.append(seq)
            except ValueError:
                pass
            payload = parts[1]
            if payload != self.expected_payload:
                self.valid_payload = False

    def noise_callback(self, msg):
        self.noise_received += 1

    def receiver_stats_callback(self, msg):
        self.receiver_stats_samples.append({
            'bandwidth': msg.bandwidth,
            'drop_rate': msg.drop_rate,
            'protocol': msg.protocol,
        })

    def check_done(self):
        # No-message timeout: if 20s pass with no messages, give up
        if self.start_time is None:
            if time.time() - self.wall_start > 20.0:
                self.get_logger().warn('No messages received after 20s, giving up')
                self.finish()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        if self._done:
            return
        self._done = True

        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.msg_count / elapsed if elapsed > 0 else 0

        # Calculate loss from sequence numbers
        if len(self.seq_numbers) >= 2:
            # Sequences should be monotonically increasing
            # Count how many are missing in the range we received
            min_seq = min(self.seq_numbers)
            max_seq = max(self.seq_numbers)
            expected_count = max_seq - min_seq + 1
            # But rate limiting means sender skips sequences, so
            # actual loss = received unique seqs vs expected unique seqs
            unique_seqs = len(set(self.seq_numbers))
            seq_loss_pct = max(0.0, (1.0 - unique_seqs / expected_count) * 100) if expected_count > 0 else 0.0
        else:
            expected_count = 0
            unique_seqs = len(self.seq_numbers)
            seq_loss_pct = 0.0

        # Use the last few stats samples (skip first which may be stale)
        stats = None
        if len(self.receiver_stats_samples) >= 2:
            # Average the last 3 samples
            recent = self.receiver_stats_samples[-3:]
            stats = {
                'bandwidth': sum(s['bandwidth'] for s in recent) / len(recent),
                'drop_rate': sum(s['drop_rate'] for s in recent) / len(recent),
                'protocol': recent[-1]['protocol'],
            }
        elif self.receiver_stats_samples:
            stats = self.receiver_stats_samples[-1]

        result = {
            'scenario': self.scenario_name,
            'duration_sec': round(elapsed, 1),
            'msg_count': self.msg_count,
            'unique_seqs': unique_seqs,
            'rate_hz': round(rate, 1),
            'seq_loss_pct': round(seq_loss_pct, 2),
            'payload_valid': self.valid_payload,
            'noise_leaked': self.noise_received,
            'receiver_stats': stats,
            'pass': self.msg_count > 0 and self.noise_received == 0 and self.valid_payload,
        }

        self.get_logger().info(f'Results: {json.dumps(result, indent=2)}')

        # Write to file
        out_path = f'/results/{self.scenario_name}.json'
        os.makedirs('/results', exist_ok=True)
        with open(out_path, 'w') as f:
            json.dump(result, f, indent=2)

        self.get_logger().info(f'Results written to {out_path}')


def main():
    rclpy.init()

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    scenario = sys.argv[2] if len(sys.argv) > 2 else 'unknown'

    node = VerifyNode(duration=duration, scenario_name=scenario)

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
