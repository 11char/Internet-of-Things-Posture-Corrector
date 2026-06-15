#include <NewPing.h>

int triggerPin = 42;
int echoPin = 42;
int maxDistance = 400; // 최대 측정 거리 (cm)

NewPing sensor(triggerPin, echoPin, maxDistance);
float initialDistance = 0.0f;
float currentDistance = 0.0f;
float distanceThreshold = 7.5f; // 초기 거리에서의 허용 오차 (cm)
float ultrasonicDistanceCm() {
  float distance = sensor.ping_cm();
    if (distance <= maxDistance && distance >= 2) {
      return distance;
    } else {
      return -1.0f; // 유효하지 않은 거리
    }
}

void calibrate_ultrasonic() {
    const int calibrationSamples = 10;
    float distanceSum = 0.0f;
    int validSamples = 0;
    
    for (int i = 0; i < calibrationSamples; ++i) {
        float distance = ultrasonicDistanceCm();
        if (distance > 0) { // 유효한 거리만 합산
        distanceSum += distance;
        validSamples++;
        }
        delay(100); // 샘플 간 짧은 지연
    }
    Serial.println("초기 거리 측정 완료.");
    if (validSamples > 0) {
        initialDistance = distanceSum / validSamples; // 평균 거리 계산
        Serial.print("초기 거리: ");
        Serial.print(initialDistance);
        Serial.println(" cm");
    } else {
        Serial.println("유효한 거리 샘플이 없습니다. 초기 거리를 설정할 수 없습니다.");
    }
}

bool withinThreshold(float distance) {
    return (initialDistance - distance) <= distanceThreshold;
}