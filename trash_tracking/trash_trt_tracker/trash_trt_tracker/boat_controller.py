# boat_controller_node.py
# 목적: camera_node나 테스트 노드에서 받은 v_r, w_r 값을 바탕으로 바퀴 제어 계산 수행

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import PortHandler, PacketHandler

# 다이나믹셀 설정
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME_LEFT = '/dev/ttyUSB1'
DEVICENAME_RIGHT = '/dev/ttyUSB0'
DXL_ID_LEFT = 1
DXL_ID_RIGHT = 2

# 로봇 기구학
R = 0.075   # 바퀴 반지름 (m)
L = 0.310   # 로봇 중심 <-> 바퀴 중심 거리 (m)
LEFT_GAIN = 38.20
RIGHT_GAIN = 38.20

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_controller')

        # 포트 및 패킷 핸들러 설정
        self.port_left = PortHandler(DEVICENAME_LEFT)
        self.port_right = PortHandler(DEVICENAME_RIGHT)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_left.openPort() or not self.port_right.openPort():
            self.get_logger().error("포트 열기 실패")
            return

        self.port_left.setBaudRate(BAUDRATE)
        self.port_right.setBaudRate(BAUDRATE)

        # 토크 활성화
        self.enable_torque(self.port_left, DXL_ID_LEFT)
        self.enable_torque(self.port_right, DXL_ID_RIGHT)

        self.v_r = None
        self.w_r = None

        # 서브스크립션 설정
        self.create_subscription(Float64, 'v_r', self.v_callback, 10)
        self.create_subscription(Float64, 'w_r', self.w_callback, 10)

        self.get_logger().info("BoatController 노드가 시작되었습니다.")

    def enable_torque(self, port, dxl_id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != 0 or dxl_error != 0:
            self.get_logger().warn(f"[ID {dxl_id}] 토크 ON 실패: "
                                   f"{self.packetHandler.getTxRxResult(dxl_comm_result)}, "
                                   f"{self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            self.get_logger().info(f"[ID {dxl_id}] 토크 ON 성공")


    def v_callback(self, msg):
        self.v_r = msg.data
        self.try_drive()

    def w_callback(self, msg):
        self.w_r = msg.data
        self.try_drive()

    def try_drive(self):
        if self.v_r is None or self.w_r is None:
            return

        v_r = self.v_r
        w_r = self.w_r

        # 바퀴 각속도 (rad/s)
        wl = (v_r - L * w_r) / R
        wr = (v_r + L * w_r) / R

        # 다이나믹셀 속도 명령값으로 변환
        wl_cmd = int(wl * LEFT_GAIN)
        wr_cmd = int(wr * RIGHT_GAIN)

        wr_cmd *= -1
        
        # 다이나믹셀 방향 설정
        # 방향 보정
        if wl_cmd >= 0:
            wl_cmd_value = wl_cmd
        else:
            wl_cmd_value = 2**32 + wl_cmd  # 음수 속도는 2의 보수 처리

        if wr_cmd >= 0:
            wr_cmd_value = wr_cmd            # 양수 → 그대로 (정방향)
        else:
            wr_cmd_value = 2**32 + wr_cmd    # 음수 → 2의 보수 처리

       # if wl_cmd < 0:
       #     wl_cmd = 2**32 + wl_cmd
       # if wr_cmd > 0:
       #     wr_cmd = 2**32 - wr_cmd

        # 명령 전송
        self.packetHandler.write4ByteTxRx(self.port_left, DXL_ID_LEFT, ADDR_GOAL_VELOCITY, wl_cmd)
        self.packetHandler.write4ByteTxRx(self.port_right, DXL_ID_RIGHT, ADDR_GOAL_VELOCITY, wr_cmd)

        self.get_logger().info('[v=%.2f, w=%.2f] → wl=%.2f, wr=%.2f, wl_cmd=%d, wr_cmd=%d' %
                               (v_r, w_r, wl, wr, wl_cmd, wr_cmd))

        # 값 초기화
        self.v_r = None
        self.w_r = None

    def shutdown(self):
        self.packetHandler.write4ByteTxRx(self.port_left, DXL_ID_LEFT, ADDR_GOAL_VELOCITY, 0)
        self.packetHandler.write4ByteTxRx(self.port_right, DXL_ID_RIGHT, ADDR_GOAL_VELOCITY, 0)
        self.port_left.closePort()
        self.port_right.closePort()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BoatController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
