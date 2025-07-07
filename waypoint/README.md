# 🗺️ Waypoint Navigation Module

> DGPS 보정 및 운동학 기반 P제어기를 활용한 정밀 웨이포인트 추종 노드

*(본 README.md 파일은 Waypoint 내비게이션 모듈의 세부 사항을 포함하며, 전체 프로젝트 개요는 [여기](../README.md)에서 확인할 수 있다.)*

---

## 📌 모듈 개요

`Waypoint Navigation Module`은 'OceanSweep' 수륙양용 로봇의 **육상 환경 자율 주행을 위한 핵심 모듈**이다. 본 모듈은 GPS 및 IMU 기반 내비게이션을 통합하여 로봇의 정해진 경로 자율 이동 기능을 구현하였다. DGPS(Differential Global Positioning System) 보정 정보를 통해 로봇의 정확한 위치를 파악하고, IMU(Inertial Measurement Unit) 센서 데이터와 함께 운동학 기반 P제어기를 적용하여 미리 설정된 웨이포인트 경로를 정밀하게 추종한다. 이는 로봇이 육상 환경에서 정해진 임무 경로를 따라 안정적으로 이동하는 데 필수적인 역할을 수행한다.

## ✨ 주요 기능

* **DGPS 기반 위치 보정**: GPS 신호의 오차를 보정하기 위해 DGPS 방식을 적용하여, 정확한 위치 정보를 기반으로 로봇의 현재 위치를 파악한다.
* **센서 데이터 필터링 및 위치 추정**:
    * GPS 데이터에 포함된 순간적인 큰 노이즈를 제거하고 정확한 위치 추정을 위해 2차원 위치 및 속도를 상태로 갖는 **칼만 필터(Kalman Filter)**를 설계하여 적용한다.
    * IMU Yaw 데이터의 센서 노이즈 감소를 위해 **1차 저주파 통과 필터(Low-pass Filter)**를 적용하여 정밀한 방위각 측정을 가능하게 한다.
* **운동학 기반 P제어기**: 로봇의 위치 및 자세 정보를 활용하는 운동학 모델 기반의 경로 추종 제어 알고리즘을 구현한다.
* **차동 구동 시스템 제어**: 계산된 선속도 및 각속도 명령을 2륜 차동 구동 방식의 로봇에 맞게 좌우 바퀴의 각속도($\omega_l, \omega_r$)로 변환하여 로봇의 직진, 회전, 곡선 주행 등 다양한 기동을 가능하게 한다.

## ⚙️ 기술 스택 및 구현 환경

* **플랫폼**: Raspberry Pi 4 (라즈베리파이)
* **운영체제 및 프레임워크**: ROS 2 Dashing (`rclpy`)
* **센서**: GPS (Neo-6M GPS 모듈), IMU 센서
* **알고리즘**: 칼만 필터, 1차 저주파 통과 필터, 운동학 기반 P제어
* **좌표 변환**: WGS-84에서 TM(EPSG:5175) 좌표계로의 변환 (`pyproj` 라이브러리 활용)

## 🌐 Waypoint 흐름도

Waypoint 내비게이션 모듈은 GPS/IMU 센서로부터 데이터를 수신하고, 이를 정제 및 처리하여 로봇의 운동 제어 명령을 생성하는 일련의 과정을 거친다. 아래 흐름도는 본 모듈의 주요 데이터 흐름 및 알고리즘 연동 방식을 시각적으로 나타낸다.

![waypoint flow chart](https://drive.google.com/uc?export=view&id=15yX_5fkynSkE_WlR4fPlOoaUYC3oO4t1)  
<sub>*waypoint 모듈 흐름도*</sub>


**설명:**
로봇은 GPS 및 IMU 센서로부터 위치와 자세 데이터를 실시간으로 획득한다. GPS 데이터는 DGPS 보정 및 2차원 상태벡터 기반 칼만 필터를 통해 정제되어 로봇의 정확한 현재 위치를 추정하는 데 사용된다. IMU 데이터는 1차 저주파 통과 필터를 통해 노이즈가 제거되어 정밀한 자세 정보를 제공한다. 추정된 현재 위치 및 자세 정보는 미리 설정된 웨이포인트 경로와 비교되어 경로 오차를 계산한다. 이 오차를 기반으로 운동학 기반 P제어기가 로봇의 선속도 및 각속도 명령을 생성하며, 이 명령은 차동 구동 기구학을 통해 좌우 모터의 개별적인 회전 속도 명령으로 변환되어 로봇을 제어한다.

## 🛠️ 작동 원리 및 구현 상세

`Waypoint Navigation Module`은 로봇의 자율 육상 주행을 위한 핵심적인 경로 추종 기능을 수행한다.

### 1. Waypoint Tracking 알고리즘 개요

Waypoint 기반 자율주행은 로봇이 일련의 미리 정의된 지점(waypoint)들을 순차적으로 따라 이동하는 방식이다. 본 알고리즘은 로봇의 현재 위치와 목표 waypoint 간의 오차를 지속적으로 계산하고, 이 오차를 줄이는 방향으로 로봇의 운동을 제어한다. 이를 통해 로봇은 복잡한 경로를 자율적으로 추종하며 이동할 수 있다.

### 2. 정밀 위치 추정을 위한 2차원 상태벡터 기반 Kalman 필터 설계

GPS 신호는 일반적인 위치 오차 외에도 순간적인 큰 노이즈가 포함되는 특성이 있다. 이를 보정하고 정확한 위치 추정을 위해 2차원 위치 및 속도를 상태로 갖는 Kalman 필터를 설계하였다. 로봇의 상태벡터는 $x=[x~y~v_{x}v_{y}]^{T}$로 정의되며, 시스템 행렬 A 및 측정 행렬 H는 각각 다음과 같이 설정된다:

$$
A = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
H = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}
$$

시스템 오차 공분산 행렬 Q 및 측정 오차 공분산 행렬 R은 실측 데이터를 기반으로 구성되며, 이는 다음과 같다:

$$
Q = \begin{bmatrix}
1.3739 \times 10^{-10} & 0 & 0 & 0 \\
0 & 3.8002 \times 10^{-10} & 0 & 0 \\
0 & 0 & 1.1838 \times 10^{-10} & 0 \\
0 & 0 & 0 & 5.9331 \times 10^{-10}
\end{bmatrix}
$$

$$
R = \begin{bmatrix}
5.6017 \times 10^{-10} & 0 \\
0 & 1.7659 \times 10^{-10}
\end{bmatrix}
$$

Kalman 필터는 이러한 모델과 GPS 측정값을 융합하여 최적의 위치 및 속도 추정값을 제공함으로써 GPS 노이즈를 효과적으로 억제한다. IMU Yaw 데이터의 경우, 별도의 1차 저주파 통과 필터(`Arduino Module`에서 구현)를 적용하여 센서 노이즈를 감소시킨다.

### 3. 운동학 기반 P제어기 설계

본 연구에서는 설정된 경로를 따라 이동하기 위해, 로봇의 위치 및 자세 정보를 이용한 운동학 모델 기반의 경로 추종 제어 알고리즘을 구현한다. 2차원 평면상에서 이동체의 상태를 다음과 같이 정의한다.

![waypoint](https://drive.google.com/uc?export=view&id=1_k54-5XsRFqo55fLt7Z1yeiFYGKbR1MQ)  
<sub>*2차원 평면에서 이동체의 상태 정의*</sub>

이를 미분하면 이동체의 속도와 회전속도가 정의된다.

이동체가 waypoint 경로 선분을 추종하기 위해선 로봇의 방위각과 경로 선분의 방향각도 오차가 0으로 수렴하고, 경로선분과
로봇의 수직거리 오차가 0으로 수렴하도록 제어하면 된다.

![waypoint algorithm](https://drive.google.com/uc?export=view&id=19hCuSjvjcTaxqvItGsbjW2TSGMXuVvr5)  
<sub>*waypoint 알고리즘*</sub>

이를 위해 2차원 평면에서의 이동체의 운동학 모델을 이용한다.

![mobile kinematic model](https://drive.google.com/uc?export=view&id=1FmOfkLsCRbwWKBfaUvbBL023rtmBjCZ4)  

해당 운동학 모델을 이용하여 로봇의 선속도와 회전속도를 결정하면 로봇이 추종해야하는 waypoint경로를 따라 이동하도록 제어할 수 있다.

로봇의 선속도 $v_{ref}$는 $0.3 \text{ [m/s]}$로 정의하였으며, 각속도($\omega_{ref}$)는 로봇의 자세 오차($\theta-\psi$)와 경로부터의 수직 거리 오차($\Delta y$)에 대해 비례 제어 방식으로 계산된다. 로봇의 정확한 방위각을 측정하기 위해 IMU 센서의 캘리브레이션을 수행하였으며, 이 과정에서 국토지리정보원에서 제공하는 지자기 편각 정보를 반영한다.

**제어식:**
$\omega_{ref}=-k_{1}(\theta-\psi)-k_{2}\Delta y$

* $\theta$: 로봇의 방위각
* $\psi$: 경로 선분의 방향 각도


이때 수직거리오차를 계산할때 단순히 점과 직선사이의 거리를 계산하면 안된다. 

선분과 직선 사이의 거리를 계산할 경우, 로봇이 목표 선분을 지나쳤음에도 불구하고 해당 직선 상의 투영점으로 오차를 계산하여 잘못된 방향으로 제어될 수 있는 문제가 발생한다. 예를 들어, 로봇이 웨이포인트 A에서 B로 이어지는 선분을 추종할 때, B 지점을 지나쳐도 직선 AB 상에서 가장 가까운 점을 기준으로 오차를 계산하면 로봇이 불필요하게 이전 경로로 되돌아가려는 제어 행동을 보일 수 있다.

이를 방지하기 위해 **Point-to-segment distance formula** 방식을 채택하였다. 이 방식은 로봇의 현재 위치가 목표 선분의 시작점과 끝점 사이에 있을 경우에만 해당 선분까지의 수직 거리를 계산하고, 로봇이 선분의 끝점을 지나친 경우에는 다음 목표 선분으로의 전환을 고려하거나, 선분의 끝점을 기준으로 오차를 계산하도록 하여 로봇이 불필요하게 이전 경로로 돌아가지 않도록 한다.

#### 점과 선분 사이의 거리 계산 수식

![Point-to-segment distance formula](https://drive.google.com/uc?export=view&id=1o-AjQsZhQuBvOzW9wY8S7B0CzbScFonN)  
<sub>*로봇 위치에 따른 선분사이의 거리 판단*</sub>

로봇의 현재 위치 $P(x_p, y_p)$와 경로 상의 선분 $AB$ (시작점 $A(x_1, y_1)$, 끝점 $B(x_2, y_2)$ )가 주어졌을 때, 점 $P$에서 선분 $AB$까지의 최단 거리 $d$는 다음 단계에 따라 계산된다.

1.  **벡터 정의**:
    * 벡터 $\vec{AB} = (x_2 - x_1, y_2 - y_1)$
    * 벡터 $\vec{AP} = (x_p - x_1, y_p - y_1)$

2.  **$P$의 선분 $AB$에 대한 투영점 결정**:
    * 점 $P$에서 직선 $AB$에 내린 수선의 발이 선분 $AB$ 내에 위치하는지 여부를 판단하기 위해, 벡터 $\vec{AB}$와 벡터 $\vec{AP}$의 내적을 활용하여 투영 계수 $t$를 계산한다: \
        ![Point-to-segment distance formula](https://drive.google.com/uc?export=view&id=1367VZflangfO-nOE2dkW9Rkxs-Jyw5av)  

    * **$t$ 값에 따른 경우 분리**:
        * **$t < 0$**: 수선의 발이 선분 $AB$의 시작점 $A$ 바깥쪽에 위치하는 경우. 최단 거리는 점 $P$와 점 $A$ 사이의 거리이다.
            ![Point-to-segment distance formula](https://drive.google.com/uc?export=view&id=1bZCF_FI5FeWVwl5LjulxqnIXBJJIGjA3)  
        * **$t > 1$**: 수선의 발이 선분 $AB$의 끝점 $B$ 바깥쪽에 위치하는 경우. 최단 거리는 점 $P$와 점 $B$ 사이의 거리이다.
            ![Point-to-segment distance formula](https://drive.google.com/uc?export=view&id=1Jl4qqvja79le8lK8FQO6eOO8_F18se1I)  
        * **$0 \le t \le 1$**: 수선의 발이 선분 $AB$ 내에 위치하는 경우. 최단 거리는 점 $P$에서 직선 $AB$까지의 수직 거리이다. 직선 $AB$의 방정식 $Ax' + By' + C = 0$을 사용하여 계산할 수 있으며, 일반적인 형태는 다음과 같다: \
            ![Point-to-segment distance formula](https://drive.google.com/uc?export=view&id=1j1jf7HmAwQ34WdqGq6mXIrRvlzjrji2C)  

    (여기서 $\Delta y$는 이 $d$ 값과 로봇의 진행 방향 및 경로 선분의 방향을 고려하여 양수 또는 음수로 결정될 수 있다.)

이러한 점과 선분 사이의 거리 계산 방식은 로봇이 현재 추종해야 할 웨이포인트 선분을 정확히 파악하고, 그에 맞는 정밀한 제어를 수행하는 데 핵심적인 역할을 한다.

### 4. 차동 구동 기구학을 통한 선속도 및 회전속도 구현

본 연구에 사용된 로봇은 2륜 차동 구동(Differential Drive) 방식으로 구동된다. 이 방식은 각 바퀴에 독립적인 회전속도 명령을 전달함으로써 직진, 회전, 곡선 주행 등 다양한 기동이 가능하다. 로봇의 제어 입력은 상기 설명된 선속도 $v_{ref}$와 운동학 기반 P제어기 계산식으로 도출된 각속도 $\omega_{ref}$이다. 이 두 입력은 차동 구동 운동학에 따라 좌우 바퀴의 각속도($\omega_l, \omega_r$)로 변환된다.

![Differential Drive](https://drive.google.com/uc?export=view&id=1pv8RuIJiXgIhHn5OjNqhwO9M2KtFWXlG)  
<sub>*차동구동 기구학*</sub>

**차동 구동 기구학 역변환:** \
![Point-to-segment distance formula](https://drive.google.com/uc?export=view&id=1LsFLFlp6W4kt8lR6FCDlcMUPwrT39lVi)  

* $R$: 바퀴 반지름
* $L$: 바퀴간 거리

## 📈 Waypoint 결과

최종적으로 운동학모델을 기반으로 waypoint 경로추종을 위한 P제어기를 설계하였고, 실제로 경로를 추종하는것을 확인할 수 있었다.

![Differential Drive](https://drive.google.com/uc?export=view&id=1qoQtl9Hs8dwS3FIDRt5BZb6z7UIXnPh-)  
<sub>*waypoint 결과*</sub>

---