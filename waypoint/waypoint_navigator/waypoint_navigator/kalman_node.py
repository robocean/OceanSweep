import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

class KalmanFilter:
    """
    2차원 위치 (x, y)와 속도 (vx, vy)를 추정하는 칼만 필터 클래스
    - 상태 벡터: [x, y, vx, vy]
    - 측정값: [x, y] (TM 좌표계 기준)
    """

    def __init__(self, dt):
        self.dt = dt

        # 상태 벡터 초기화: [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        # 상태 전이 행렬 A
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 측정 행렬 H (위치만 측정 가능)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # 프로세스 노이즈 공분산 Q (모델 불확실성)
        self.Q = np.diag([1.3739e-10, 3.8002e-10, 1.1838e-10, 5.9331e-10])

        # 측정 오차 공분산 R (측정 불확실성)
        self.R = np.diag([5.6017e-10, 1.7659e-10])

        # 초기 공분산 행렬 P (초기 상태 추정 오차)
        self.P = np.eye(4)

    def set_initial_position(self, x0, y0):
        """초기 위치 설정"""
        self.x[0, 0] = x0
        self.x[1, 0] = y0

    def predict(self):
        """예측 단계"""
        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, z):
        """갱신 단계"""
        z = z.reshape((2, 1))
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.A.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

    def process(self, z):
        """
        칼만 필터 전체 단계 수행 (예측 + 갱신)
        입력 z: [x, y]
        출력: 필터링된 x, y 값
        """
        self.predict()
        self.update(z)
        return self.x[0, 0], self.x[1, 0]


class KalmanNode(Node):
    """
    칼만 필터 기반 위치 추정 ROS 2 노드
    입력:
        - tm_x (Float64): TM 좌표계 x 위치
        - tm_y (Float64): TM 좌표계 y 위치
    출력:
        - kalman_x (Float64): 필터링된 x 위치
        - kalman_y (Float64): 필터링된 y 위치
    """

    def __init__(self):
        super().__init__('kalman_node')
        self.kf = KalmanFilter(dt=0.1)

        self.latest_x = None
        self.latest_y = None
        self.initialized = False  # 초기화 여부 플래그

        # 구독자 설정
        self.create_subscription(Float64, 'tm_x', self.x_callback, 10)
        self.create_subscription(Float64, 'tm_y', self.y_callback, 10)

        # 퍼블리셔 설정
        self.x_pub = self.create_publisher(Float64, 'kalman_x', 10)
        self.y_pub = self.create_publisher(Float64, 'kalman_y', 10)

        # 주기적 타이머 (0.1초 주기)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def x_callback(self, msg):
        """tm_x 토픽 수신 시 최신 x 값 저장"""
        self.latest_x = msg.data

    def y_callback(self, msg):
        """tm_y 토픽 수신 시 최신 y 값 저장"""
        self.latest_y = msg.data

    def timer_callback(self):
        """주기적으로 칼만 필터 실행 후 결과 퍼블리시"""
        if self.latest_x is None or self.latest_y is None:
            return

        # 초기 위치 설정 (최초 1회)
        if not self.initialized:
            self.kf.set_initial_position(self.latest_x, self.latest_y)
            self.initialized = True
            self.get_logger().info(f"칼만 필터 초기 위치 설정: x={self.latest_x:.3f}, y={self.latest_y:.3f}")
            return

        # 칼만 필터 업데이트 수행
        filtered_x, filtered_y = self.kf.process(np.array([self.latest_x, self.latest_y]))

        # 결과 퍼블리시
        self.x_pub.publish(Float64(data=filtered_x))
        self.y_pub.publish(Float64(data=filtered_y))

        self.get_logger().info(f"칼만 필터 출력: x={filtered_x:.3f}, y={filtered_y:.3f}")


def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = KalmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

