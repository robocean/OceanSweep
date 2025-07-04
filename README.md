# GPS & Vision‑Based Amphibious Marine Debris Collection Robot

> GPS 및 영상 기반 P제어기를 적용한 수륙양용 해양쓰레기 수거로봇 개발

*(본 README는 개괄 정보만을 담고 있으며, 각 서브 프로젝트의 세부 사용법은 폴더별 **`README.md`**에서 확인해주세요.)*

---

## 📌 프로젝트 개요

- **목적**: 해양 쓰레기 수거 자동화를 위한 자율주행 로봇 시스템 설계·구현
- **핵심 기술**  
  - **위치 추정**: DGPS 보정 + TM(EPSG:5175) 좌표계 변환  
  - **경로 추종**: Waypoint 기반 P 제어기  
  - **쓰레기 추적**: YOLOv5 + TensorRT 실시간 추론  
  - **사용자 인터페이스**: Web 기반 Waypoint 입력 및 모니터링
- **주요 하드웨어**: Raspberry Pi, Jetson Nano, Arduino (Neo‑6M GPS), 수륙양용 구동 플랫폼

---

## 🗂️ 디렉터리 구조

| 폴더                | 플랫폼                      | 설명                                      |
|---------------------|-----------------------------|-------------------------------------------|
| `waypoint/`         | Raspberry Pi / ROS 2        | DGPS 보정 + Waypoint 추종 노드              |
| `trash_tracking/`   | Jetson Nano / Python        | 영상 기반 쓰레기 위치 인식 및 전송           |
| `server/`           | Windows PC                  | FastAPI 백엔드, Svelte 프론트 + TCP Relay 서버 |
| `arduino/`          | Arduino                     | GPS 오차 측정 및 릴레이 서버로 전송           |

> 각 폴더 내에 별도의 `README.md`를 통해 설치 방법 및 실행 방법을 안내할 예정입니다.

---

## ⚙️ 기술 스택

- **ROS 2 Dashing** (`rclpy`)
- **Jetson Nano** (PyTorch 1.10 + TensorRT 8.x)
- **FastAPI / Svelte** + DuckDNS + HTTPS (Let’s Encrypt)
- **Arduino IDE 2.3.2 / Neo-6M GPS**
- **좌표 변환**: WGS‑84 → TM(EPSG:5175) (`pyproj` 사용)

---

## 🚀 빠른 시작

```bash
# 1. 레포지토리 클론
git clone git@github.com:robocean/KMOUCapstone.git
cd KMOUCapstone

# 2. 예시: 라즈베리파이 환경 실행
cd waypoint
# → 자세한 내용은 waypoint/README.md 참고
