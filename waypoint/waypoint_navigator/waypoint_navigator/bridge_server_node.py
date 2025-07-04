import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import socket
import threading
import re

# 릴레이 서버 설정 (실제 사용 시 IP/포트 입력 필요)
RELAY_SERVER_IP = 'YOUR (D)DNS OR IP'
RELAY_SERVER_PORT = YOURPORT

class BridgeServerNode(Node):
    def __init__(self):
        super().__init__('bridge_server_node')

        # GPS 보정값 퍼블리셔 생성
        # 토픽명: gps_correction_lat, gps_correction_lon
        # 다른 노드에서 보정된 위도/경도 값을 사용할 수 있도록 퍼블리시
        self.pub_lat = self.create_publisher(Float64, 'gps_correction_lat', 10)
        self.pub_lng = self.create_publisher(Float64, 'gps_correction_lon', 10)

        # 별도의 스레드에서 릴레이 서버와 TCP 연결 수행
        threading.Thread(target=self.connect_to_relay_server, daemon=True).start()

    def connect_to_relay_server(self):
        """
        릴레이 서버에 TCP 연결을 수행하고
        GPS 보정 데이터를 수신하여 ROS 2 토픽으로 퍼블리시하는 함수
        """
        try:
            # TCP 소켓 생성 및 서버 연결
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((RELAY_SERVER_IP, RELAY_SERVER_PORT))
            self.get_logger().info(f"Connected to relay server: {RELAY_SERVER_IP}:{RELAY_SERVER_PORT}")

            # 지속적으로 데이터 수신 루프
            while True:
                data = sock.recv(1024)
                if not data:
                    # 서버 측에서 연결 종료 시
                    self.get_logger().warn("Connection closed by server")
                    break

                # 수신 데이터 디코딩 및 전처리
                message = data.decode(errors='ignore').strip()
                self.get_logger().info(f"Received raw: {message}")

                # 데이터 형식 예시: ERR_LAT=0.00010,ERR_LNG=0.00008
                match = re.match(r'ERR_LAT=([-0-9.]+),ERR_LNG=([-0-9.]+)', message)
                if match:
                    # 추출한 보정값을 ROS 2 토픽으로 퍼블리시
                    err_lat = float(match.group(1))
                    err_lng = float(match.group(2))

                    self.pub_lat.publish(Float64(data=err_lat))
                    self.pub_lng.publish(Float64(data=err_lng))

                    self.get_logger().info(f"Published GPS Error → lat: {err_lat}, lon: {err_lng}")
                else:
                    # 예외적인 포맷 처리
                    self.get_logger().warn("Unrecognized data format")

        except Exception as e:
            # 예외 처리 및 에러 로그 출력
            self.get_logger().error(f"Error in TCP connection: {e}")

def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = BridgeServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

