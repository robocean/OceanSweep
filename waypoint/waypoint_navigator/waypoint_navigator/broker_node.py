import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial
import threading
import time

class BrokerNode(Node):
    def __init__(self):
        super().__init__('broker_node')

        # 시리얼 포트 설정
        self.serial_port = '/dev/ttyACM0'      # 사용중인 시리얼 포트
        self.baud_rate = 115200                # 시리얼 통신 속도 (baudrate)

        # 시리얼 포트 연결 시도
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 열기 실패: {e}")
            return

        # 아두이노 초기화(리셋)
        self.reset_arduino()

        # ROS 2 퍼블리셔 생성
        # yaw_data: IMU로부터 수신한 Yaw 값 (Float32)
        # gps_data: 원본 GPS 문자열 데이터 (String, RAW_LAT, RAW_LNG 형식)
        self.yaw_publisher = self.create_publisher(Float32, 'yaw_data', 10)
        self.gps_string_publisher = self.create_publisher(String, 'gps_data', 10)

        # 시리얼 읽기용 백그라운드 스레드 시작
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

        self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud.")

    def reset_arduino(self):
        """
        아두이노 DTR 신호를 이용하여 아두이노 보드 리셋 수행
        """
        try:
            self.ser.setDTR(False)
            time.sleep(0.5)
            self.ser.setDTR(True)
            self.get_logger().info("Arduino has been reset.")
        except Exception as e:
            self.get_logger().error(f"Arduino 리셋 실패: {e}")

    def read_serial(self):
        """
        시리얼 포트에서 데이터를 지속적으로 읽고, 적절한 ROS 2 토픽으로 퍼블리시
        - "Yaw:" → yaw_data (Float32)
        - "RAW_LAT:" 및 "RAW_LNG:" → gps_data (String)
        """
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    # Yaw 데이터 처리
                    if "Yaw:" in line:
                        try:
                            yaw_value = float(line.split(":")[1].strip())
                            msg = Float32()
                            msg.data = yaw_value
                            self.yaw_publisher.publish(msg)
                            self.get_logger().info(f"Published Yaw: {yaw_value:.2f}")
                        except ValueError:
                            self.get_logger().warning(f"Yaw 데이터 형식 오류: {line}")

                    # GPS 데이터 처리
                    elif "RAW_LAT:" in line and "RAW_LNG:" in line:
                        try:
                            parts = line.split(",")
                            lat_str = parts[0].split(":")[1].strip()
                            lng_str = parts[1].split(":")[1].strip()

                            gps_msg = String()
                            gps_msg.data = f"RAW_LAT={lat_str},RAW_LNG={lng_str}"
                            self.gps_string_publisher.publish(gps_msg)

                            self.get_logger().info(f"Published /gps_data: {gps_msg.data}")
                        except (IndexError, ValueError) as e:
                            self.get_logger().warning(f"GPS 데이터 파싱 오류: {line}")
            except Exception as e:
                self.get_logger().error(f"시리얼 수신 중 오류: {e}")

    def destroy_node(self):
        """
        노드 종료 시 시리얼 포트 정상적으로 닫기
        """
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            self.get_logger().warning(f"시리얼 포트 닫기 실패: {e}")
        super().destroy_node()

def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = BrokerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

