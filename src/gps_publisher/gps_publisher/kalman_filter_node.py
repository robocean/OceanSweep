import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # xy_data 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'xy_data',  # 구독할 토픽 이름
            self.xy_callback,
            10
        )

        # kalman_xy 토픽 발행 (예측된 위치)
        self.publisher_xy = self.create_publisher(
            String,  # JSON 형식으로 발행
            'kalman_xy',
            10
        )

        # kalman_v 토픽 발행 (예측된 속도)
        self.publisher_v = self.create_publisher(
            String,  # JSON 형식으로 발행
            'kalman_v',
            10
        )

        self.get_logger().info('Kalman Filter Node Started.')

        # Kalman 필터 초기화
        self.dt = 1.0  # 시간 간격 (1초)
        self.F = np.array([[1, 0, self.dt, 0],  # 상태 전이 행렬
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0],  # 관측 행렬
                           [0, 1, 0, 0]])
        self.R = np.array([[1, 0],  # 관측 잡음 공분산
                           [0, 1]])
        self.Q = np.array([[1, 0, 0, 0],  # 프로세스 잡음 공분산
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.P = np.eye(4)  # 초기 오차 공분산 행렬
        self.x = np.array([[0],  # 초기 상태 벡터 (x 위치, y 위치, x 속도, y 속도)
                           [0],
                           [0],
                           [0]])

    def xy_callback(self, msg):
        try:
            # xy_data 토픽에서 JSON 파싱
            xy_data = json.loads(msg.data)
            if 'x' not in xy_data or 'y' not in xy_data:
                self.get_logger().warn("Received data does not contain 'x' or 'y'. Ignoring.")
                return

            z = np.array([[xy_data['x']], [xy_data['y']]])  # 측정값 (위치)

            # 칼만 필터 예측 및 업데이트
            self.kalman_filter(z)

            # 예측된 위치와 속도를 발행
            self.publish_results()

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON data received. Ignoring message.")
        except Exception as e:
            self.get_logger().error(f"Error processing data: {e}")

    def kalman_filter(self, z):
        # 예측 단계
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        # 업데이트 단계
        y = z - np.dot(self.H, self.x)  # 잔차 계산
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # 칼만 이득 계산
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)  # 공분산 업데이트

    def publish_results(self):
        # 예측된 위치 발행
        kalman_xy = {
            'x': self.x[0, 0],
            'y': self.x[1, 0]
        }
        self.publisher_xy.publish(String(data=json.dumps(kalman_xy)))

        # 예측된 속도 발행
        kalman_v = {
            'vx': self.x[2, 0],
            'vy': self.x[3, 0]
        }
        self.publisher_v.publish(String(data=json.dumps(kalman_v)))

        # 터미널 출력
        self.get_logger().info(f"Predicted Position: x={self.x[0, 0]:.6f}, y={self.x[1, 0]:.6f}")
        self.get_logger().info(f"Predicted Velocity: vx={self.x[2, 0]:.6f}, vy={self.x[3, 0]:.6f}")

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
