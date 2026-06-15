#include <Arduino.h>


constexpr int recalibratePin = 1;
constexpr int piezoPin = 10;


void initMpu6050();
void calibrateMpu6050(int samples = 200);
void calibrateFrontBackDirection(int samples = 100);
bool readHeadTilt(float &pitchDeg, float &rollDeg);
bool isHeadTilted(float pitchDeg, float rollDeg);
bool readFrontBackDisplacementCm(float &frontBackDisplacementCm);

bool withinThreshold(float distance);
float ultrasonicDistanceCm();
void calibrate_ultrasonic();

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(recalibratePin, INPUT_PULLDOWN);
  pinMode(piezoPin, OUTPUT);

  initMpu6050();
  calibrateMpu6050();
  calibrateFrontBackDirection();
  calibrate_ultrasonic();

  Serial.println("MPU-6050 head tilt detector ready");
}

void loop() {
  static bool lastRecalibrateState = LOW;
  const bool recalibrateState = digitalRead(recalibratePin);

  if (recalibrateState == HIGH && lastRecalibrateState == LOW) {
    Serial.println("Recalibrating MPU-6050...");
    calibrateMpu6050();
    calibrate_ultrasonic();
  }

  lastRecalibrateState = recalibrateState;

  float pitchDeg = 0.0f;
  float rollDeg = 0.0f;
  float frontBackDisplacementCm = 0.0f;

  if (readHeadTilt(pitchDeg, rollDeg)) {
    const bool tilted = isHeadTilted(pitchDeg, rollDeg);
    readFrontBackDisplacementCm(frontBackDisplacementCm);

    // Serial.print("Pitch: ");
    // Serial.print(pitchDeg, 1);
    // Serial.print(" deg, Roll: ");
    // Serial.print(rollDeg, 1);
    // Serial.print(" deg -> ");
    // Serial.println(tilted ? "HEAD TILTED" : "OK");

    // Serial.print("Estimated front/back distance: ");
    // Serial.print(frontBackDisplacementCm, 2);
    // Serial.println(" cm");
  }
  Serial.println();
  Serial.print("Ultrasonic distance: ");
  float distance = ultrasonicDistanceCm();
  Serial.print(distance, 2);
  Serial.println(" cm");


  if(!withinThreshold(distance) or abs(frontBackDisplacementCm) > 5.0f) {
    tone(piezoPin, 1000);
  } else {
    noTone(piezoPin);
  }

  delay(100);
}