import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import PortHandler, PacketHandler

# 다이나믹셀 설정
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME_LEFT = '/dev/left_wheel'
DEVICENAME_RIGHT = '/dev/right_wheel'
DXL_ID_LEFT = 1
DXL_ID_RIGHT = 2

# 로봇 기구학 파라미터
R = 0.075   # 바퀴 반지름 (m)
L = 0.310   # 로봇 중심 <-> 바퀴 간 거리 (m)
LEFT_GAIN = 40.21
RIGHT_GAIN = 41.02

class DiffDriveNode(Node):
    """
    차동구동 로봇 제어 노드 (DiffDriveNode)
    - 입력:
        - v_r (Float64): 선속도 명령
        - w_r (Float64): 각속도 명령
    - 출력:
        - 다이나믹셀 모터 제어 명령 (좌/우 바퀴 속도 설정)
    주요 기능:
    - v_r, w_r 명령을 수신하여 좌/우 바퀴 각속도로 변환 (차동구동 기구학 사용)
    - 계산된 각속도를 다이나믹셀 모터에 write4ByteTxRx 명령어로 전송
    - 음수 속도는 2's complement 처리하여 전송
    """

    def __init__(self):
        super().__init__('diff_drive_node')

        # 다이나믹셀 포트 핸들 설정
        self.port_left = PortHandler(DEVICENAME_LEFT)
        self.port_right = PortHandler(DEVICENAME_RIGHT)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # 포트 오픈
        if not self.port_left.openPort() or not self.port_right.openPort():
            self.get_logger().error("포트 열기 실패")
            return
        self.port_left.setBaudRate(BAUDRATE)
        self.port_right.setBaudRate(BAUDRATE)

        # 구독자 설정
        self.create_subscription(Float64, 'v_r', self.v_callback, 10)
        self.create_subscription(Float64, 'w_r', self.w_callback, 10)

        # 내부 상태 저장용
        self.v_r = None
        self.w_r = None

        self.get_logger().info("DiffDriveNode 시작됨.")

    def v_callback(self, msg):
        """v_r 토픽 수신"""
        self.v_r = msg.data
        self.try_drive()

    def w_callback(self, msg):
        """w_r 토픽 수신"""
        self.w_r = msg.data
        self.try_drive()

    def try_drive(self):
        """
        좌/우 바퀴 속도 계산 후 모터 명령 전송
        - 차동구동 기구학 공식 사용:
            wl = (v_r - (L/2) * w_r) / R
            wr = (v_r + (L/2) * w_r) / R
        - 각속도(rad/s)를 다이나믹셀 속도 명령값으로 변환 후 전송
        - 음수 속도는 2's complement 처리하여 표현
        """
        if self.v_r is None or self.w_r is None:
            return

        v_r = self.v_r
        w_r = self.w_r

        # 좌/우 바퀴 속도 계산 (rad/s → RPM 단위 환산 필요시 추가 적용 가능)
        wl = (v_r - (L / 2) * w_r) / R
        wr = (v_r + (L / 2) * w_r) / R

        # 안전 클램핑 (음수 속도 허용 시 아래 처리 필요)
        wl = max(0.0, wl)
        wr = max(0.0, wr)

        # 다이나믹셀 명령 변환 (gain 적용)
        wl_cmd = int(wl * LEFT_GAIN)
        wr_cmd = int(wr * RIGHT_GAIN)

        # 음수 속도 2's complement 처리
        if wr_cmd > 0:
            wr_cmd = 2**32 - wr_cmd
        if wl_cmd < 0:
            wl_cmd = 2**32 + wl_cmd

        # 모터 명령 전송
        self.packetHandler.write4ByteTxRx(self.port_left, DXL_ID_LEFT, ADDR_GOAL_VELOCITY, wl_cmd)
        self.packetHandler.write4ByteTxRx(self.port_right, DXL_ID_RIGHT, ADDR_GOAL_VELOCITY, wr_cmd)

        # 디버그 출력
        self.get_logger().info(
            f"[v={v_r:.2f}, w={w_r:.2f}] → wl={wl:.2f}, wr={wr:.2f}, wl_cmd={wl_cmd}, wr_cmd={wr_cmd}"
        )

        # 명령 1회 처리 후 None으로 초기화
        self.v_r = None
        self.w_r = None

    def destroy_node(self):
        """노드 종료 시 모터 정지 및 포트 닫기"""
        self.packetHandler.write4ByteTxRx(self.port_left, DXL_ID_LEFT, ADDR_GOAL_VELOCITY, 0)
        self.packetHandler.write4ByteTxRx(self.port_right, DXL_ID_RIGHT, ADDR_GOAL_VELOCITY, 0)
        self.port_left.closePort()
        self.port_right.closePort()
        super().destroy_node()

def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
