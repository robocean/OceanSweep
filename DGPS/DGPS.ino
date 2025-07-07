#include <TinyGPS++.h>

TinyGPSPlus gps;

// gps에서 받아온 좌표를 저장할 변수
float lat = 0.0;
float lng = 0.0;

// 오차를 저장할 변수
float e_lat = 0.0;
float e_lng = 0.0;

// 현재위치의 정확한 위경도값, 전자지도(카카오지도)를 통해 정확한 위치값을 알아낸다.
const float local_lat = YOUR_LOCATION;
const float local_lng = YOUR_LOCATION;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;  // 0.5 seconds

void setup() {
  Serial.begin(9600);           // Serial to PC or ROS
  Serial1.begin(9600);          // GPS module baud rate

  Serial.println("Starting GPS reception...");
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());

    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lng = gps.location.lng();

      e_lat = lat - local_lat;
      e_lng = lng - local_lng;
    }
  }

  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();

    if (gps.location.isValid()) {
      // Print error gps value
      Serial.print("ERR_LAT=");
      Serial.print(e_lat, 5);
      Serial.print(",ERR_LNG=");
      Serial.println(e_lng, 5);
    }
    else {
      // Print GPS unavailable message
      Serial.println("GPS signal not valid");
    }
  }
}