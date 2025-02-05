import asyncio
import websockets
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.websocket_url = "ws://192.168.0.68:8000/robocean/gps/ws"
        self.websocket = None
        self.loop = asyncio.new_event_loop()
        self.websocket_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.websocket_thread.start()

        self.subscription = self.create_subscription(
            String,
            'gps_data',
            self.listener_callback,
            10
        )

    def run_event_loop(self):
        """WebSocket 관리를 위한 asyncio 이벤트 루프 실행."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.manage_websocket_connection())

    async def manage_websocket_connection(self):
        """WebSocket 연결을 관리."""
        while True:
            try:
                self.websocket = await websockets.connect(
                    self.websocket_url,
                    ping_interval=20,  # 20초마다 Ping
                    ping_timeout=20    # Ping 응답 제한 시간
                )
                self.get_logger().info("WebSocket 연결 성공")
                break
            except Exception as e:
                self.get_logger().error(f"WebSocket 연결 실패: {e}. 5초 후 재시도...")
                await asyncio.sleep(5)

    async def send_to_websocket(self, data):
        """WebSocket으로 데이터를 전송."""
        if not self.websocket or self.websocket.closed:
            self.get_logger().warning("WebSocket 연결이 닫혀 있음. 데이터 전송 불가")
            return

        try:
            self.get_logger().info(f"전송할 데이터: {data}")
            await self.websocket.send(data)
            self.get_logger().info(f"WebSocket 전송 성공: {data}")
        except Exception as e:
            self.get_logger().error(f"WebSocket 전송 중 오류: {e}"
                    )
    
    async def close_websocket(self):
        """WebSocket 연결을 닫는 비동기 함수."""
        if self.websocket:
            try:
                await self.websocket.close()
                self.get_logger().info("WebSocket 연결 종료")
            except Exception as e:
                self.get_logger().error(f"WebSocket 종료 중 오류: {e}")

    def listener_callback(self, msg):
        """ROS 토픽에서 GPS 데이터를 수신하고 WebSocket으로 전송."""
        self.get_logger().info(f"수신한 GPS 데이터: {msg.data}")

        if self.websocket:
            try:
                # JSON 문자열 그대로 전송
                data = f'{{"gps_data": {msg.data}}}'
                self.get_logger().info(f"전송할 JSON 데이터: {data}")
    
                # WebSocket 전송 비동기 작업 실행
                future = asyncio.run_coroutine_threadsafe(
                    self.send_to_websocket(data),
                    self.loop
                )
                future.result(timeout=5)  # 작업 완료를 기다림
            except Exception as e:
                self.get_logger().error(f"WebSocket 전송 중 오류: {e}")
        else:
            self.get_logger().warning("WebSocket 연결이 닫혀 있음")
    
    def destroy_node(self):
        """ROS2 노드 종료 시 호출."""
        pending = asyncio.all_tasks(loop=self.loop)
        for task in pending:
            task.cancel()

        self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        self.loop.run_until_complete(self.close_websocket())
        self.loop.stop()
        self.loop.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSNode()

    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        gps_node.get_logger().info("GPSNode 종료")
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
