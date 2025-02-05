import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json

# WebSocket 서버 설정
WEBSOCKET_SERVER_URL = "ws://192.168.0.68:8000/robocean/gps/ws"

class WebSocketTestNode(Node):
    def __init__(self):
        super().__init__('websocket_test_node')
        self.get_logger().info("WebSocketTestNode 시작")

        # WebSocket 연결 태스크 생성
        self.loop = asyncio.get_event_loop()
        self.websocket = None
        self.loop.create_task(self.connect_websocket())

        # ROS 타이머로 데이터 전송
        self.timer = self.create_timer(2.0, self.timer_callback)

    async def connect_websocket(self):
        while True:
            try:
                self.get_logger().info("WebSocket 서버에 연결 중...")
                self.websocket = await websockets.connect(WEBSOCKET_SERVER_URL)
                self.get_logger().info("WebSocket 연결 성공")
            except Exception as e:
                self.get_logger().warn(f"WebSocket 연결 실패: {e}. 5초 후 재시도 중...")
                await asyncio.sleep(5)
            else:
                break

    async def send_test_data(self):
        if self.websocket is None or self.websocket.closed:
            self.get_logger().warn("WebSocket 연결이 닫혀 있음. 데이터를 전송하지 않습니다.")
            return

        try:
            test_data = {"lat": 35.0, "lng": 129.0}
            await self.websocket.send(json.dumps(test_data))
            self.get_logger().info(f"WebSocket 데이터 전송: {test_data}")
        except Exception as e:
            self.get_logger().error(f"WebSocket 데이터 전송 오류: {e}")

    def timer_callback(self):
        self.loop.create_task(self.send_test_data())

    def destroy_node(self):
        # WebSocket 연결 종료
        if self.websocket and not self.websocket.closed:
            self.loop.run_until_complete(self.websocket.close())
        self.get_logger().info("WebSocketTestNode 종료")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트로 노드 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())

