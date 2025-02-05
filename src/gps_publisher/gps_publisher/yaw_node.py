import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class YawNode(Node):
    def __init__(self):
        super().__init__('yaw_node')
        self.subscription = self.create_subscription(Float32, 'yaw_data', self.yaw_callback, 10)

    def yaw_callback(self, msg):
        yaw_value = msg.data
        self.get_logger().info(f"Yaw 데이터 수신 및 활용: {yaw_value}")

def main(args=None):
    rclpy.init(args=args)
    node = YawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("YawNode 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
