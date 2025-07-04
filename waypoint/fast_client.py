import requests
import json

#fastapi 서버로부터waypoint를 받아오는 클라이언트 코드
#url의 IP부분은  fast api가 구동중인 서버의 ip로 대체할것

url = "http://YOUR_SERVER_IP_OR_DNS:8000/robocean/get_coordinates"
save_path = "/home/ubuntu/ros2_ws2/waypoints.json"

try:
    response = requests.get(url)

    if response.status_code == 200:
        # JSON 텍스트 저장
        with open(save_path, "w") as f:
            f.write(response.text)
        print(f"파일 저장 완료: {save_path}")
    else:
        print(f"요청 실패 - 상태 코드: {response.status_code}, 응답: {response.text}")

except Exception as e:
    print(f"오류 발생: {e}")
