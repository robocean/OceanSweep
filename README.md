# GPS & Vision‑Based Amphibious Marine Debris Collection Robot

> GPS 및 영상 기반 P제어기를 적용한 수륙양용 해양쓰레기 수거로봇 개발

*(본 README.md 파일은 프로젝트의 전체 개요를 포함하고 있으며, 각 핵심 기술의 세부 사항은 하위 디렉터리별 README.md 파일을 통해 확인할 수 있습니다.)*

---

## 📌 프로젝트 개요

- **목적**: 해양 쓰레기 수거 자동화를 위한 자율주행 로봇 시스템 설계·구현
- **핵심 기술**  
  - **위치 추정**: DGPS 보정 + TM(EPSG:5175) 좌표계 변환  
  - **경로 추종**: Waypoint 기반 P 제어기  
  - **쓰레기 추적**: YOLOv5 + TensorRT 실시간 추론  
  - **사용자 인터페이스**: Web 기반 Waypoint 입력 및 모니터링
- **하드웨어**: Raspberry Pi, Jetson Nano, Arduino mega2560, Neo‑6M(GPS), MPU9250(IMU), 수륙양용 구동 플랫폼

이 문서는 수륙양용 해양쓰레기 수거로봇 플랫폼 개발에 관한 내용을 작성하였다. 수륙양용을 위해 로봇은 방수설계가 적용되었으며, 설계단계에서 부력계산을 통해 로봇의 수상운행 가능성을 검토하였다. 그러나 실제 해양환경에서의 실험은 기상문제와 안전상의 문제로 실험 환경을 육상 환경과 수조 환경으로 분리하여 구현하였다. 육상에서는 waypoint 기반 자율주행 기능을, 수조 환경에서는 인공지능 기반 영상처리를 통해 쓰레기 탐지 및 수거 기능을 검증하였다.

![flow chart](https://drive.google.com/uc?export=view&id=1SsmRRuB_ef9-9xoN4hEDy-MCKG0wIf5V)  
<sub>*육지환경 흐름도*</sub>

![flow chart](https://drive.google.com/uc?export=view&id=1xj7-aLJtd8UocgSyo67M9hwgq7LmD7An)  
<sub>*수조환경 흐름도*</sub>

---

## 🗂️ 디렉터리 구조

| 폴더                | 플랫폼                      | 설명                                      |
|---------------------|-----------------------------|-------------------------------------------|
| `waypoint/`         | Raspberry Pi / ROS 2        | DGPS 보정 + Waypoint 추종 노드              |
| `trash_tracking/`   | Jetson Nano / Python        | 영상 기반 쓰레기 위치 인식 및 전송           |
| `server/`           | Windows                     | FastAPI 백엔드, Svelte 프론트 + TCP Relay 서버 |
| `arduino/`          | Arduino                     | GPS및 IMU센서를 이용한 위치와 방위각 측정및 노이즈 제거|
| `DGPS/`             | Windows                     | GPS 위치 오차 측정 및 릴레이 서버로 전송           |

> 핵심기술의 세부사항은 각 폴더별 readme파일 참조.

---

## ⚙️ 기술 스택

- **ROS 2 Dashing / Foxy** (`rclpy`)
- **Jetson Nano** (PyTorch 1.10 + TensorRT 8.x)
- **FastAPI / Svelte**
- **Arduino IDE 2.3.2 / Neo-6M GPS**
- **좌표 변환**: WGS‑84 → TM(EPSG:5175) (`pyproj` 사용)

---

## 🛠️ 로봇 설계

![robot_photo](https://drive.google.com/uc?export=view&id=1js2hFgn9yw9kBC1bLLEw2Lgo3DgyNQmg)  
<sub>* 수륙양용 해양쓰레기 수거 로봇 사진*</sub>

본 프로젝트에서 제작된 수륙양용 로봇은 아크릴판을 주 재료로 사용하여 구성되었으며, 설계 초기 단계에서 로봇의 체적과 재료의 비중을 고려하여 전체 무게를 예측하였다. 이를 바탕으로 부력을 계산하고, 실제 무게보다 충분한 부력을 확보할 수 있도록 설계함으로써 수상 환경에서의 안정적인 운용을 가능하게 하였다.

무게는 다음 공식을 통해 계산되었다.

> $W = \rho_{\text{material}} \times V$  
> (여기서 $W$는 무게 [kg], $\rho_{\text{material}}$는 재료의 비중 [g/cm³], $V$는 로봇의 체적 [cm³] 이다.)

또한, 물에서 작용하는 부력은 다음 식을 이용하여 산출하였다.

> $F_b = \rho_{\text{water}} \times V$  
> (여기서 $F_b$는 부력 [kgf], $\rho_{\text{water}}$은 물의 밀도, $V$는 로봇의 체적 [cm³] 이다.)

위 공식을 이용해 로봇의 각 파트별 무게를 계산 하고, 내장되는 부품의 무게를 더하면 전체 무게가 어느정도인지 예측 가능하다.

이를 토대로 계산된 무게: 약 3.5kg \
계산된 부력: 9.5kg \
계산을 통해 부력이 충분하여 로봇이 물에 뜰 수 있다는것을 확인하였다.



로봇의 구동 방식은 **차동구동(Differential Drive)**으로, 좌우 바퀴에 각각 모터를 장착하여 회전 속도의 차이를 이용해 방향을 제어하도록 구성하였다. 또한, 쓰레기를 전진하면서 수거할 수 있도록 **쌍동선(Twin-hull)** 구조를 채택하였다.

※ 방수설계

로봇은 수상에서 활동하기 때문에 방수성능을 확보해야 한다. 다른 부분은 방수테이프나 에폭시등을 활용하여 수밀구조를 구현할 수 있으나, 로봇의 하우징과 모터축 사이의 빈틈은 일반적인 방법으로 수밀구조를 구현하기 어렵다.

바퀴축이 회전을 하기 위해서는 하우징에 빈 틈이 생기는데, 이 틈으로 물이 들어올 수 있기때문에 여기를 막아줘야 하지만, 바퀴축이 회전을 하기위해선 틈이 존재해야 한다는 모순이 발생한다. 이때 적용 가능한것이 메카니컬 씰이다. 메카니컬 씰은 수직된 섭동면 으로 구성되어 한 면이 회전축과 함께 회전하며 스프링의 장력 혹은 유체의 압력으로 회전부의 밀봉을 지속적 으로 유지하는 축 밀봉장치이다.

![robot_photo](https://drive.google.com/uc?export=view&id=1mJNYOKdsUvp4fko-kgF80gJ-L9xN7oeA)  
<sub>*↑ 메카니컬 씰 구조*</sub>

이를 통해 바퀴의 회전을 보장하면서 방수성능도 확보할 수 있었다.

---