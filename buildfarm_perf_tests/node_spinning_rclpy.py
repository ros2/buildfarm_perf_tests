"""Minimal rclpy node used by the spinning overhead benchmark."""

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()