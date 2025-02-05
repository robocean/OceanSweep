import asyncio
import websockets
import json

# ROSbridge 서버 정보
ROSBRIDGE_URL = "ws://192.168.0.101:9090"
ROS2_TOPIC = "/gps_data"

# 백엔드 서버 정보
BACKEND_SERVER_URL = "ws://192.168.0.68:8000/robocean/gps/ws"

async def connect_and_forward():
    """ROSBridge와 백엔드 서버에 연결하고 GPS 데이터를 전달."""
    while True:  # 자동 재연결 루프
        try:
            # ROSBridge와 백엔드 서버에 연결
            async with websockets.connect(ROSBRIDGE_URL) as rosbridge_socket, \
                       websockets.connect(BACKEND_SERVER_URL) as backend_socket:

                # ROS2 토픽 구독 요청
                subscribe_message = {
                    "op": "subscribe",
                    "topic": ROS2_TOPIC,
                }
                await rosbridge_socket.send(json.dumps(subscribe_message))
                print(f"ROS2 토픽 {ROS2_TOPIC} 구독 시작")

                while True:
                    try:
                        # ROSBridge에서 메시지 수신
                        ros_message = await asyncio.wait_for(rosbridge_socket.recv(), timeout=10)
                        data = json.loads(ros_message)

                        # 유효한 메시지 확인 및 파싱
                        if "msg" in data and "data" in data["msg"]:
                            gps_data = json.loads(data["msg"]["data"])
                            print(f"파싱된 GPS 데이터: {gps_data}")

                            # 백엔드 서버로 데이터 전송
                            await backend_socket.send(json.dumps(gps_data))
                            print(f"백엔드 서버로 데이터 전송: {gps_data}")
                        else:
                            print("유효하지 않은 데이터 형식:", data)

                    except asyncio.TimeoutError:
                        print("ROSBridge 메시지 수신 대기 시간 초과. 재시도...")
                        continue  # 재시도

        except websockets.ConnectionClosed as e:
            print(f"WebSocket 연결 종료. 재연결 시도 중... ({e})")
            await asyncio.sleep(2)  # 재연결 대기 시간

        except Exception as e:
            print(f"오류 발생: {e}. 재연결 시도 중...")
            await asyncio.sleep(2)  # 재연결 대기 시간

if __name__ == "__main__":
    try:
        asyncio.run(connect_and_forward())
    except KeyboardInterrupt:
        print("프로그램 종료")
