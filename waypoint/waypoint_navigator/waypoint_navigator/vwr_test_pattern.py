import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import sys
import termios
import tty
import select

class VRWRPatternPublisher(Node):
    """
    테스트용 v_r, w_r 패턴 발행 노드
    - 일정한 v_r, w_r 값을 주기적으로 발행
    - 사용자가 키보드 'q' 입력 시 v_r, w_r = 0.0을 발행하고 종료
    사용 목적:
    - boat_controller_node 등에서 수신하여 테스트용으로 모터 동작 확인
    """

    def __init__(self):
        super().__init__('vwr_pattern_publisher')

        # 퍼블리셔 설정
        # - v_r: 선속도 명령 (Float64)
        # - w_r: 각속도 명령 (Float64)
        self.v_pub = self.create_publisher(Float64, 'v_r', 10)
        self.w_pub = self.create_publisher(Float64, 'w_r', 10)

        # 타이머 (1초 주기)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.stop_requested = False

        # 현재 터미널 상태 저장
        self.fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.fd)

        # 키보드 입력 감시 스레드 시작 (비동기적으로 'q' 감지용)
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    def timer_callback(self):
        """
        주기적으로 v_r, w_r 값을 퍼블리시
        - stop_requested가 True가 되면 0.0 명령을 발행하고 노드 종료
        """
        if self.stop_requested:
            # 종료 시 0.0 명령 발행
            self.v_pub.publish(Float64(data=0.0))
            self.w_pub.publish(Float64(data=0.0))
            self.get_logger().info("정지 명령 전송됨. 노드 종료 중...")
            rclpy.shutdown()
            return

        # 테스트용 고정 패턴 값
        v_r = 0.3
        w_r = 0.4

        # 퍼블리시
        self.v_pub.publish(Float64(data=v_r))
        self.w_pub.publish(Float64(data=w_r))
        self.get_logger().info(f"[{self.counter}s] v_r: {v_r:.2f}, w_r: {w_r:.2f}")
        self.counter += 1

    def keyboard_listener(self):
        """
        키보드 입력을 비동기적으로 감시하여 'q' 입력 시 종료 요청
        """
        try:
            tty.setcbreak(self.fd)
            while not self.stop_requested:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'q':
                        self.get_logger().info("Q 입력 감지. 종료합니다.")
                        self.stop_requested = True
                        break
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term_settings)

def main(args=None):
    # ROS 2 노드 초기화 및 실행
    rclpy.init(args=args)
    node = VRWRPatternPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 입력 시 정지 명령 발행
        node.get_logger().info("Ctrl+C 감지. 정지 명령 전송.")
        node.v_pub.publish(Float64(data=0.0))
        node.w_pub.publish(Float64(data=0.0))
    finally:
        # 터미널 상태 복원 후 노드 종료
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, node.old_term_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
