#!/usr/bin/env python3
"""
Publishes 200 noise topics at 10 Hz and 1 main topic at 100 Hz.
The noise topics exist only on the talker side to prove they don't
interfere with the forwarded main_chatter.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class NoisePublisher(Node):
    def __init__(self):
        super().__init__('noise_publisher')

        # 200 noise topics at 10 Hz
        self.noise_pubs = []
        self.noise_count = 200
        for i in range(1, self.noise_count + 1):
            pub = self.create_publisher(String, f'/chatter{i}', 10)
            self.noise_pubs.append(pub)

        self.noise_timer = self.create_timer(0.1, self.noise_callback)  # 10 Hz
        self.noise_seq = 0
        self.get_logger().info(f'Publishing {self.noise_count} noise topics at 10 Hz')

    def noise_callback(self):
        self.noise_seq += 1
        for i, pub in enumerate(self.noise_pubs):
            msg = String()
            msg.data = f'noise_{i+1}_seq_{self.noise_seq}'
            pub.publish(msg)


class MainPublisher(Node):
    def __init__(self):
        super().__init__('main_publisher')

        # Main topic: 1KB payload at 100 Hz
        self.main_pub = self.create_publisher(String, '/main_chatter', 10)
        self.main_timer = self.create_timer(0.01, self.main_callback)  # 100 Hz
        self.main_seq = 0
        self.payload = '0123456789' * 100  # 1000 bytes
        self.get_logger().info('Publishing /main_chatter (1KB) at 100 Hz')

    def main_callback(self):
        self.main_seq += 1
        msg = String()
        msg.data = f'{self.main_seq}:{self.payload}'
        self.main_pub.publish(msg)


def main():
    rclpy.init()

    noise = NoisePublisher()
    main_pub = MainPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(noise)
    executor.add_node(main_pub)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        noise.destroy_node()
        main_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
