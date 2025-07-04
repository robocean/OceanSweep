import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import re
from threading import Lock

class DGPSNode(Node):
    def __init__(self):
        super().__init__('dgps_node')

        # 원본 GPS 데이터 (위도, 경도)
        self.raw_gps = None  # (lat, lon)

        # 수신한 GPS 오차값 (DGPS 보정값)
        self.error_lat = 0.0
        self.error_lon = 0.0

        # 멀티스레드 접근 보호용 락
        self.lock = Lock()

        # 구독자 등록
        # - gps_data: 원본 GPS 문자열 데이터 수신 (String)
        # - gps_correction_lat: DGPS 보정값 (위도, Float64)
        # - gps_correction_lon: DGPS 보정값 (경도, Float64)
        self.gps_sub = self.create_subscription(String, 'gps_data', self.gps_callback, 10)
        self.lat_sub = self.create_subscription(Float64, 'gps_correction_lat', self.error_lat_callback, 10)
        self.lon_sub = self.create_subscription(Float64, 'gps_correction_lon', self.error_lon_callback, 10)

        # 보정된 GPS 데이터를 퍼블리시 (CORRECTED_LAT, CORRECTED_LNG 형식)
        self.publisher_ = self.create_publisher(String, 'corrected_gps', 10)

    def gps_callback(self, msg):
        """
        gps_data 토픽에서 수신한 원본 GPS 문자열 데이터를 파싱하여 저장
        """
        try:
            match = re.match(r'RAW_LAT=([-0-9.]+),RAW_LNG=([-0-9.]+)', msg.data)
            if match:
                lat = float(match.group(1))
                lon = float(match.group(2))
                with self.lock:
                    self.raw_gps = (lat, lon)
                self.publish_corrected()
            else:
                self.get_logger().warn(f"GPS 데이터 형식이 올바르지 않음: {msg.data}")
        except Exception as e:
            self.get_logger().warn(f"GPS 파싱 오류: {e}")

    def error_lat_callback(self, msg):
        """
        gps_correction_lat 토픽에서 수신한 위도 오차값 저장 및 보정값 퍼블리시
        """
        with self.lock:
            self.error_lat = msg.data
        self.publish_corrected()

    def error_lon_callback(self, msg):
        """
        gps_correction_lon 토픽에서 수신한 경도 오차값 저장 및 보정값 퍼블리시
        """
        with self.lock:
            self.error_lon = msg.data
        self.publish_corrected()

    def publish_corrected(self):
        """
        현재 원본 GPS 값과 보정값을 이용하여
        보정된 GPS 데이터를 계산한 후 corrected_gps 토픽으로 퍼블리시
        """
        with self.lock:
            if self.raw_gps is None:
                # 아직 원본 GPS 수신 전인 경우 처리 안함
                return

            raw_lat, raw_lon = self.raw_gps
            corrected_lat = raw_lat - self.error_lat
            corrected_lon = raw_lon - self.error_lon

        msg = String()
        msg.data = f"CORRECTED_LAT={corrected_lat:.5f},CORRECTED_LNG={corrected_lon:.5f}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"보정 GPS 퍼블리시: {msg.data}")

def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = DGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

