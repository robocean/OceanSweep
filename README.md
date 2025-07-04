# KMOUCapstone – GPS & Vision-Based Amphibious Marine Debris Collection Robot

> **국립한국해양대학교 인공지능공학부**  
> GPS 및 영상 기반 P제어기를 적용한 수륙양용 해양쓰레기 수거로봇 개발

![robot_photo](https://drive.google.com/uc?export=view&id=1js2hFgn9yw9kBC1bLLEw2Lgo3DgyNQmg)  
<sub>*↑ 수륙양용 해양쓰레기 수거 로봇 사진 (샘플)*</sub>

---

## 🌊 프로젝트 개요

**KMOUCapstone** 프로젝트는 해양 쓰레기 문제에 대응하기 위해 **자율적으로 쓰레기를 탐지하고 수거할 수 있는 수륙양용 로봇 시스템**을 개발하는 것을 목표로 한다. 본 시스템은 육상환경에서 waypoint기반 자율주행을, 수상환경에서는 쓰레기 탐지및 추적을 수행하며, 핵심기술은 다음과 같이 구성되어 있다.

### ✅ 핵심 구성요소

- **위치 추정 및 경로 추종 제어**
  - DGPS 보정과 TM 좌표계(EPSG:5175) 기반 위치 추정
  - Waypoint 기반 자율주행 및 P제어기 적용
  - Kalman Filter를 통한 GPS noise 보정
- **실시간 영상 기반 쓰레기 인식 및 정렬**
  - YOLOv5 기반 객체 탐지
  - Jetson Nano에서 TensorRT 최적화를 통해 실시간 추론
  - 객체 중심 오차(Δx)에 따른 비례제어(P-Control) 방식 정렬
- **네트워크 통신 시스템**
  - TCP 기반 릴레이 서버를 통한 GPS 보정 정보 수신
  - Web 기반 사용자 인터페이스를 통한 경로 지정
- **하드웨어 설계**
  - 수륙양용 주행을 위한 쌍동선 구조 + 패들형 바퀴
  - 기계적 방수 구조 (메커니컬 씰 포함)
  - 중앙 유입 구조의 쓰레기 수거 메커니즘

### 🔬 실험 환경

- **육상**: DGPS 기반 waypoint 자율 주행 실험
- **수조**: 영상 기반 쓰레기 탐지 및 회수 실험
- 실험은 실제 해양 환경의 제약(파도, 기상, 안전성 등)을 고려하여 분리된 환경에서 수행됨

---

> 📌 *이 README는 전체 개요를 담고 있으며, 각 하위 폴더의 `README.md`에서 세부 설명을 확인하실 수 있습니다.*

