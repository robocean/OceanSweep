import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from pyproj import CRS, Transformer
import re

# 좌표계 설정
# WGS84 (EPSG:4326, 위경도 좌표계) → TM (EPSG:5175, 중부원점 기반 TM 좌표계)
crs_wgs84 = CRS.from_epsg(4326)
crs_tm = CRS.from_epsg(5175)
transformer = Transformer.from_crs(crs_wgs84, crs_tm, always_xy=True)

class TMConverterNode(Node):
    def __init__(self):
        super().__init__('tm_converter_node')

        # corrected_gps 토픽에서 보정된 GPS 데이터 수신
        self.subscription = self.create_subscription(
            String,
            'corrected_gps',
            self.listener_callback_corrected,
            10
        )

        # TM 좌표 퍼블리셔 등록
        self.x_publisher = self.create_publisher(Float64, 'tm_x', 10)
        self.y_publisher = self.create_publisher(Float64, 'tm_y', 10)

    def listener_callback_corrected(self, msg):
        """
        corrected_gps 토픽에서 보정된 GPS 데이터를 수신하여
        TM 좌표계로 변환 후 tm_x, tm_y 토픽으로 퍼블리시
        """
        try:
            match = re.match(r'CORRECTED_LAT=([-0-9.]+),CORRECTED_LNG=([-0-9.]+)', msg.data)
            if match:
                lat = float(match.group(1))
                lon = float(match.group(2))
                tm_x, tm_y = transformer.transform(lon, lat)
                self.x_publisher.publish(Float64(data=tm_x))
                self.y_publisher.publish(Float64(data=tm_y))
                self.get_logger().info(f"TM 좌표 퍼블리시: X={tm_x:.2f}, Y={tm_y:.2f}")
            else:
                self.get_logger().warn(
                    f"CORRECTED 형식 오류: {msg.data} | 예상 형식: CORRECTED_LAT=<float>,CORRECTED_LNG=<float>"
                )
        except Exception as e:
            self.get_logger().error(f"TM 변환 중 오류: {e}")

def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = TMConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

