import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import math
import threading
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Dynamixel 모터 설정
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME_1 = '/dev/left_wheel'
DEVICENAME_2 = '/dev/right_wheel'
DXL_ID_1 = 1
DXL_ID_2 = 2

class WaypointControllerNode(Node):
    def __init__(self):
        super().__init__('waypoint_controller_node')

        # 상태 퍼블리셔
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)

        # Dynamixel 초기화
        self.portHandler_1 = PortHandler(DEVICENAME_1)
        self.portHandler_2 = PortHandler(DEVICENAME_2)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_motor(self.portHandler_1, DXL_ID_1)
        self.init_motor(self.portHandler_2, DXL_ID_2)

        # kalman_xy, yaw_data 토픽 구독
        self.create_subscription(String, 'kalman_xy', self.coordinates_callback, 10)
        self.create_subscription(Float32, 'yaw_data', self.yaw_callback, 10)

        # 웨이포인트 좌표 파일을 읽어 초기화
        self.read_waypoint_from_file('/home/ubuntu/utm_coordinates.json')  # JSON 파일 경로 설정

        # 로봇 상태 초기화
        self.current_position = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.goal_index = 0
        self.goal_radius = 0.7  # 목표 반경
        self.delta_y_threshold = 0.3  # 허용 가능한 delta_y 값

        self.stop_requested = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Waypoint Controller Node가 시작되었습니다.")

        # 키보드 입력 감지 스레드 시작
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    def init_motor(self, port_handler, motor_id):
        """모터 초기화"""
        if not port_handler.openPort():
            self.get_logger().error(f"모터 {motor_id} 포트를 열지 못했습니다.")
        port_handler.setBaudRate(BAUDRATE)
        self.packetHandler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)

    def coordinates_callback(self, msg):
        """kalman_xy 토픽에서 받은 좌표값을 처리하는 콜백"""
        try:
            data = json.loads(msg.data)
            self.current_position["x"] = data["x"]
            self.current_position["y"] = data["y"]
            self.get_logger().info(f"현재 좌표: {self.current_position}")
        except json.JSONDecodeError:
            self.get_logger().error("좌표 데이터 파싱 오류.")
        except Exception as e:
            self.get_logger().error(f"좌표 데이터 처리 오류: {e}")

    def yaw_callback(self, msg):
        """yaw_data 토픽에서 받은 yaw 값을 처리하는 콜백"""
        self.current_position["yaw"] = msg.data
        self.get_logger().info(f"현재 yaw: {self.current_position['yaw']}")

    def read_waypoint_from_file(self, file_path):
        """웨이포인트 좌표를 JSON 파일에서 읽어오는 함수"""
        try:
            with open(file_path, 'r') as file:
                waypoint_data = json.load(file)
                if len(waypoint_data["utm_coordinates"]) >= 2:
                    self.start_position = waypoint_data["utm_coordinates"][0]  # 첫 번째 좌표
                    self.end_position = waypoint_data["utm_coordinates"][1]  # 두 번째 좌표
                    self.get_logger().info(f"웨이포인트 좌표: 시작 위치 = {self.start_position}, 끝 위치 = {self.end_position}")
                else:
                    self.get_logger().error("웨이포인트 좌표가 2개 이상 존재하지 않습니다.")
        except FileNotFoundError:
            self.get_logger().error(f"파일을 찾을 수 없습니다: {file_path}")
        except json.JSONDecodeError:
            self.get_logger().error("JSON 파일 파싱 오류.")
        except Exception as e:
            self.get_logger().error(f"웨이포인트 파일 처리 오류: {e}")

    def calculate_cross_product(self, current_position, start_position, end_position):
        """현재 위치와 path(두 좌표 간 직선) 사이의 외적 계산"""
        x0, y0 = current_position["x"], current_position["y"]
        x1, y1 = start_position["x"], start_position["y"]
        x2, y2 = end_position["x"], end_position["y"]

        # 벡터 외적 계산
        cross_product = (x2 - x1) * (y0 - y1) - (y2 - y1) * (x0 - x1)
        return cross_product

    def control_loop(self):
        """로봇 제어 루프"""
        if self.stop_requested:
            self.get_logger().info("모터 정지 요청 상태: 제어 루프를 스킵합니다.")
            return

        goal_position = {"x": self.end_position["x"], "y": self.end_position["y"]}  # 목표는 end_position

        # start_position과 end_position을 사용하여 외적을 계산
        cross_product = self.calculate_cross_product(self.current_position, self.start_position, goal_position)

        # 상태 퍼블리싱
        state_message = json.dumps({
            "current_position": self.current_position,
            "goal_position": goal_position,
            "cross_product": cross_product
        })
        self.state_publisher.publish(String(data=state_message))

        # 경로 벗어남 보정
        if abs(cross_product) > self.delta_y_threshold:
            self.get_logger().info(f"경로 벗어남: cross_product={cross_product:.2f}. 경로 복귀 중...")
            self.correct_path(cross_product)
        else:
            self.get_logger().info("직진 중...")
            self.move_forward()

        # 목표 위치에 도달했는지 확인
        dx = goal_position["x"] - self.current_position["x"]
        dy = goal_position["y"] - self.current_position["y"]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance <= self.goal_radius:
            self.get_logger().info(f"목표 {self.goal_index}에 도달했습니다: {goal_position}")
            self.stop_motors()
            self.goal_index += 1

    def correct_path(self, cross_product):
        """경로로 복귀"""
        if cross_product > 0:  # 오른쪽으로 회전
            self.set_motor_speed(800, -300)
        else:  # 왼쪽으로 회전
            self.set_motor_speed(300, -800)

    def move_forward(self):
        """로봇 직진"""
        self.set_motor_speed(700, -700)

    def stop_motors(self):
        """모터 정지"""
        self.set_motor_speed(0, 0)

    def set_motor_speed(self, left_speed, right_speed):
        """모터 속도 설정"""
        self.packetHandler.write4ByteTxRx(self.portHandler_1, DXL_ID_1, ADDR_GOAL_VELOCITY, left_speed)
        self.packetHandler.write4ByteTxRx(self.portHandler_2, DXL_ID_2, ADDR_GOAL_VELOCITY, right_speed)

    def destroy_node(self):
        """노드 종료 시 모터 정리"""
        self.stop_motors()
        self.portHandler_1.closePort()
        self.portHandler_2.closePort()
        super().destroy_node()

    def keyboard_listener(self):
        """키보드 입력을 감지하여 동작을 수행"""
        while True:
            try:
                key = input("키 입력 (b: 정지, q: 종료): ")
                if key == 'b':
                    self.stop_requested = True
                    self.get_logger().info("'b' 키 입력 감지: 모터 정지 요청")
                    self.stop_motors()
                elif key == 'q':
                    self.get_logger().info("'q' 키 입력 감지: 모터 및 노드 종료 요청")
                    self.stop_motors()
                    rclpy.shutdown()
                    break
            except Exception as e:
                self.get_logger().error(f"키보드 입력 처리 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()