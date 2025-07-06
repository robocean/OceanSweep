#include "MPU9250.h"
#include <TinyGPS++.h>
#include <math.h>

//LPF를 사용하기위한 alpha값 설정
#define ALPHA 0.75

MPU9250 mpu;
TinyGPSPlus gps;

float last_lat = 0.0;
float last_lng = 0.0;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;              //통신 간격, 0.5초에 한번 gps 수신

float yawLPF = 0.0;
bool fMPUFirst = true;                                //mpu9250이 처음으로 yaw값을 수신받는다면 yaw_lpf에 yaw값 바로 대입하기 위한 flag 변수

float delta = 0.0, prevYaw = 0.0, linYaw = 0.0;       // Sawtooth 선형 보정을 위한 변수
float unwrapedYaw = 0.0;                              //선형 보정이 된 yaw값을 LPF에 활용하기 위해 저장할 변수

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);

    mpu.calibrateAccelGyro();
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);

    mpu.calibrateMag();
    print_calibration();

    Serial.println("set magnetic devlineation -8.58");
    mpu.setMagneticDeclination(-8.58);    //경남 거제 편각: -8도, 울산 언양편각: -9도
                                          //부산 지역은 그 중간이므로 -8.58도로 편각 설정
    Serial.println("set filter iteration 5");
    mpu.setFilterIterations(5);           //드래프트 개선을 위해 MADGWICK필터를 5번 반복
                                          //라이브러리 문서에서는 10~20정도 반복해야 안정적인 yaw값 추정이 가능하다 했지만, 
                                          //mcu 성능이 낮아 5번만 반복. 4번 반복하는것은 드래프트현상이 여전히 남아있음.
                                          //5번 반복시 노이즈가 커져서 LPF적용 필요.(보류)
    mpu.verbose(false);


    Serial1.begin(9600);          // GPS module baud rate
    Serial.println("Starting GPS reception...");
    delay(1000);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            unwrapedYaw = linear_yaw();
            printLPFYaw(unwrapedYaw);
            prev_ms = millis();
        }
    }
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());

      if (gps.location.isValid()) {
        last_lat = gps.location.lat();
        last_lng = gps.location.lng();
      }
    }

    if (millis() - lastPrintTime >= printInterval) {
      lastPrintTime = millis();

      if (gps.location.isValid()) {
        // Print valid GPS coordinates
        Serial.print("RAW_LAT:");
        Serial.print(last_lat, 5);
        Serial.print(",");
        Serial.print("RAW_LNG:");
        Serial.println(last_lng, 5);
      } else {
        // Print GPS unavailable message
        Serial.println("GPS signal not valid");
    }
  }

    //delay(100);     //loop문에 delay 넣지 말것, yaw값 갱신 안됨.
}

// IMU 센서의 yaw 값은 0도에서 359도까지 증가한 다음, 다시 0도로 돌아가는 Sawtooth 파형을 가짐.
// 이런 데이터는 그래프 상에서는 급격한 감소처럼 보이며,
// 로봇의 회전 방향이나 전체 회전량을 정확하게 파악하기 어렵게 만든다.
// 이를 해결하기 위해 Sawtooth 함수를 끊김 없이 이어지는 선형 함수로 변환할 필요가 있다.
// 이를 unwrap, 또는 선형 보정(linear correction) 이라고 부르며
// 아래의 함수는 이러한 기능을 수행한다.
float linear_yaw() {
    float yaw = mpu.getYaw();

    if (yaw < 0) yaw += 360;        //현재 yaw값은 -180~179도 범위로 출력됨. 0~359도 범위로 보정

    delta = yaw - prevYaw;          //현재값과 이전값을 비교하여 오차 계산.
    
    if (delta < -180.0) {           //ex) 350 → 10 → 실제로는 +20도인데 -340도 회전한것처럼 보임
        delta += 360.0;             //이에 대한 선형 보정
    }
    else if (delta > 180.0) {       //ex) 10 → 350 → 실제로는 -20도인데 +340도 회전한것처럼 보임
        delta -= 360.0;             //이에 대한 선형보정
    }

    linYaw += delta;
    //linYaw += 20;
    prevYaw = yaw;

    // Serial.print("linear_yaw: ");
    // Serial.println(linYaw);

    return linYaw + 20;
}

// LPF(지수가중 이동평균필터 사용)
// x_hat_k = alpha * prev_x_hat_k + (1 - alpha) * x_k
// LPF는 가중치 alpha를 조절하여 새로 측정되는 값과 기존에 저장했던 과거의 데이터를 적절히 조절하여 현재 값을 계산함.
// imu를 통해 읽어들이는 방위각은 주변 자기장의 영향을 너무 많이 받아 측정결과가 조금씩 달라짐.
// 센서의 값을 너무 신뢰하지 않는것이 좋을것으로 생각됨.
void printLPFYaw(float unwrapedYaw){
    float yaw;
    if (fMPUFirst) {
        yawLPF = unwrapedYaw;
        fMPUFirst = false;
    }
    else{
        yawLPF = ALPHA * yawLPF + (1 - ALPHA) * unwrapedYaw;
    }
    
    yaw = fmod(yawLPF, 360.0);  //선형보정된 yaw를 0~360범위로 변환
    if (yaw < 0) yaw += 360.0;  //yawLPF < 0인 경우 yaw는 -359 ~ 0 의 값을 가짐, 이를 양수로 변환

    Serial.print("Yaw: ");
    Serial.println(yaw, 2);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}