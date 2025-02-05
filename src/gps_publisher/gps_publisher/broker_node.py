import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import json

SERIAL_PORT = "/dev/Arduino_serial"
BAUD_RATE = 115200

class BrokerNode(Node):
    def __init__(self):
        super().__init__('broker_node')
        self.gps_publisher = self.create_publisher(String, 'gps_data', 10)
        self.yaw_publisher = self.create_publisher(Float32, 'yaw_data', 10)
        self.get_logger().info("BrokerNode 시작")

        try:
            self.serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"시리얼 포트 {SERIAL_PORT} 연결 성공")
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 연결 실패: {e}")
            rclpy.shutdown()

        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        line = self.serial_connection.readline().decode('utf-8').strip()
        if not line:
            return

        try:
            # Yaw 데이터 처리
            if "Yaw:" in line:
                yaw_value = float(line.split(":")[1].strip())
                msg = Float32()
                msg.data = yaw_value
                self.yaw_publisher.publish(msg)
                self.get_logger().info(f"Yaw 퍼블리시: {yaw_value}")

            # GPS 데이터 처리
            elif "," in line:
                try:
                    lat, lng = map(float, line.split(","))
                    gps_data = {"lat": lat, "lng": lng}
                    json_data = json.dumps(gps_data)

                    msg = String()
                    msg.data = json_data
                    self.gps_publisher.publish(msg)
                    self.get_logger().info(f"GPS 퍼블리시: {json_data}")

                except ValueError as ve:
                    self.get_logger().error(f"GPS 데이터 파싱 오류: {ve}")

        except Exception as e:
            self.get_logger().error(f"데이터 처리 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BrokerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("BrokerNode 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
