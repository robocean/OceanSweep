import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from pyproj import Proj

class GPSToUTMConverter(Node):
    def __init__(self):
        super().__init__('gps_to_utm_converter')

        # GPS 데이터 구독
        self.subscription = self.create_subscription(
            String,   # GPS 데이터 토픽 (JSON 문자열)
            'gps_data',  
            self.gps_callback,
            10
        )
        
        # xy_data 토픽 발행 (결과를 JSON으로 발행)
        self.publisher = self.create_publisher(
            String,
            'xy_data',
            10
        )

        self.get_logger().info('GPS to UTM Converter Node Started.')

        # UTM 변환 설정 (예: zone=52, WGS84)
        self.utm_proj = Proj(proj="utm", zone=52, ellps="WGS84", south=False)
        
        # isFirst 플래그 초기값 1로 설정
        # 1이면 다음 수신되는 GPS 데이터는 무시
        # 0이면 다음 수신되는 GPS 데이터는 처리
        self.isFirst = 1

    def gps_callback(self, msg):
        try:
            gps_data = json.loads(msg.data)

            # lat, lng 필드 확인
            if 'lat' not in gps_data or 'lng' not in gps_data:
                self.get_logger().warn("Received data does not contain 'lat' or 'lng'. Ignoring.")
                return

            latitude = gps_data['lat']
            longitude = gps_data['lng']

            if self.isFirst == 1:
                # 첫 번째 데이터 수신 시: 무시하고 isFirst=0으로 설정
                self.get_logger().info("First data received, ignoring and set isFirst=0.")
                self.isFirst = 0
            else:
                # isFirst=0 상태에서 들어온 데이터는 처리
                x, y = self.utm_proj(longitude, latitude)

                # 터미널 출력
                self.get_logger().info(f"Processing GPS Data: Latitude={latitude}, Longitude={longitude}")
                self.get_logger().info(f"Converted UTM Coordinates: x={x:.6f}, y={y:.6f}")

                # xy_data 발행
                xy_data = {'x': x, 'y': y}
                self.publisher.publish(String(data=json.dumps(xy_data)))

                # 처리 후 isFirst=1로 다시 설정
                self.isFirst = 1

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON data received. Ignoring.")
        except Exception as e:
            self.get_logger().error(f"Error processing GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSToUTMConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
