import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32
import math
import json
from collections import deque
import os

class WaypointNode(Node):
    """
    Waypoint 기반 직선 경로 추종 노드
    - 입력:
        - kalman_x (Float64): 칼만 필터 적용된 TM 좌표계 x 위치
        - kalman_y (Float64): 칼만 필터 적용된 TM 좌표계 y 위치
        - yaw_data (Float32): 현재 yaw 각도 (degree 단위)
    - 출력:
        - v_r (Float64): 선속도 명령
        - w_r (Float64): 각속도 명령
    주요 제어 방식:
    - delta_y (경로 선분에 대한 수직 거리)와 theta_error (현재 yaw - 경로 각도)를 기반으로 P제어기 적용
    - theta_error가 클 경우 v_r = 0으로 감속
    - waypoint 도착 시 다음 waypoint로 자동 전환
    """

    def __init__(self):
        super().__init__('waypoint_node')

        # 현재 위치 및 자세
        self.x = None
        self.y = None
        self.yaw = None  # degree 단위

        # 제어 파라미터
        self.k1 = 0.7
        self.k2 = 0.7
        self.v_r_val = 0.3
        self.arrival_thresh = 1.5  # waypoint 도착 거리 기준

        # 내부 상태
        self.arrival_time = None
        self.timer_period = 0.1

        # waypoint 로드 및 큐 구성
        self.waypoint_queue = deque()
        self.load_waypoints()

        # 구독자 설정
        self.create_subscription(Float64, 'kalman_x', self.x_callback, 10)
        self.create_subscription(Float64, 'kalman_y', self.y_callback, 10)
        self.create_subscription(Float32, 'yaw_data', self.yaw_callback, 10)

        # 퍼블리셔 설정
        self.v_r_pub = self.create_publisher(Float64, 'v_r', 10)
        self.w_r_pub = self.create_publisher(Float64, 'w_r', 10)

        # 제어 루프 타이머
        self.create_timer(self.timer_period, self.control_loop)

    def load_waypoints(self):
        """waypoints.json 파일에서 waypoint 목록 로드"""
        filepath = os.path.expanduser('~/ros2_ws2/waypoints.json')
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            coords = data["coordinates"]
            self.tm_waypoints = [(pt["x"], pt["y"]) for pt in coords]
        except Exception as e:
            self.get_logger().error(f'Waypoint 파일 로딩 실패: {e}')
            return

        if len(self.tm_waypoints) < 2:
            self.get_logger().error('Waypoint가 2개 미만입니다.')
            return

        self.waypoint_queue.append(self.tm_waypoints[0])
        self.waypoint_queue.append(self.tm_waypoints[1])
        self.current_index = 1

    def x_callback(self, msg):
        """kalman_x 토픽 수신"""
        self.x = msg.data

    def y_callback(self, msg):
        """kalman_y 토픽 수신"""
        self.y = msg.data

    def yaw_callback(self, msg):
        """yaw_data 토픽 수신 (degree → float)"""
        self.yaw = msg.data

    def control_loop(self):
        """
        메인 제어 루프
        - 현재 waypoint 선분에 대해 delta_y, theta_error 계산
        - P제어기 기반으로 v_r, w_r 결정 후 퍼블리시
        - waypoint 도착 판정 및 업데이트
        """
        if None in (self.x, self.y, self.yaw):
            return
        if len(self.waypoint_queue) < 2:
            return

        p1 = self.waypoint_queue[0]
        p2 = self.waypoint_queue[1]
        x1, y1 = p1
        x2, y2 = p2
        x_t, y_t = self.x, self.y

        dx = x2 - x1
        dy = y2 - y1
        l = math.hypot(dx, dy)
        psi = math.atan2(dy, dx)  # 라디안

        # 현재 yaw → 라디안 변환
        yaw_rad = math.radians(self.yaw)
        theta_error = (yaw_rad - psi + math.pi) % (2 * math.pi) - math.pi

        # 도착 판정
        dist_to_p2 = math.hypot(x2 - x_t, y2 - y_t)
        if dist_to_p2 < self.arrival_thresh:
            if self.arrival_time is None:
                self.arrival_time = self.get_clock().now()
            else:
                duration = (self.get_clock().now() - self.arrival_time).nanoseconds * 1e-9
                if duration >= 1.0:
                    if self.current_index + 1 < len(self.tm_waypoints):
                        self.waypoint_queue.popleft()
                        self.waypoint_queue.append(self.tm_waypoints[self.current_index + 1])
                        self.get_logger().info(f'Waypoint {self.current_index} 도착. 다음으로 이동.')
                        self.current_index += 1
                        self.arrival_time = None
                    else:
                        self.get_logger().info_once('모든 waypoint 도착 완료')
                        self.publish_vel(0.0, 0.0)
                        return
        else:
            self.arrival_time = None

        # delta_y 계산
        dot = ((x_t - x1) * dx + (y_t - y1) * dy) / (l * l)
        if dot < 0:
            delta_y = math.hypot(x1 - x_t, y1 - y_t)
        elif dot > 1:
            delta_y = math.hypot(x2 - x_t, y2 - y_t)
        else:
            numerator = abs((x2 - x1)*(y1 - y_t) - (x1 - x_t)*(y2 - y1))
            delta_y = numerator / l

        # cross product → delta_y 부호 결정
        cross = (x2 - x1)*(y_t - y1) - (y2 - y1)*(x_t - x1)
        if cross < 0:
            delta_y = -delta_y

        # delta_y saturation
        delta_y = max(min(delta_y, 2.0), -2.0)

        # 속도 제어
        v_val = 0.0 if abs(theta_error) > 0.3 else self.v_r_val
        w_r_val = -self.k1 * theta_error - self.k2 * delta_y
        self.publish_vel(v_val, w_r_val)

    def publish_vel(self, v, w):
        """v_r, w_r 명령 퍼블리시"""
        v_msg = Float64()
        v_msg.data = v
        self.v_r_pub.publish(v_msg)

        w_msg = Float64()
        w_msg.data = w
        self.w_r_pub.publish(w_msg)
        self.get_logger().info(f"제어 입력 퍼블리시: v_r = {v:.3f} m/s, w_r = {w:.3f} rad/s")

def main(args=None):
    # ROS 2 노드 실행
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

