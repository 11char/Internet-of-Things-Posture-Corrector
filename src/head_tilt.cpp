#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

namespace {
Adafruit_MPU6050 mpu;

constexpr int I2C_SDA = 7;
constexpr int I2C_SCL = 6;

constexpr float kTiltThresholdDeg = 20.0f;
constexpr float kAccelerationDeadbandMps2 =0.60f;
constexpr float kGravityMps2 = 9.80665f;
constexpr unsigned long kShortTermHoldMs = 350;
constexpr float kVelocityDecayFactor = 0.85f;
constexpr float kPositionDecayFactor = 0.90f;

float accelOffsetX = 0.0f;
float accelOffsetY = 0.0f;
float accelOffsetZ = 0.0f;
float gravityRefX = 0.0f;
float gravityRefY = 0.0f;
float gravityRefZ = kGravityMps2;
bool sensorReady = false;
float frontBackVelocityMps = 0.0f;
float frontBackPositionM = 0.0f;
unsigned long lastFrontBackUpdateMs = 0;
unsigned long lastSignificantAccelMs = 0;

// Returns true if the sensor data was read successfully, false otherwise.
bool readAccelerationMps2(float &ax, float &ay, float &az) {
	if (!sensorReady) {
		return false;
	}

	sensors_event_t accel;
	sensors_event_t gyro;
	sensors_event_t temp;
	mpu.getEvent(&accel, &gyro, &temp);

	ax = accel.acceleration.x - gravityRefX;
	ay = accel.acceleration.y - gravityRefY;
	az = accel.acceleration.z - gravityRefZ;
	return true;
}

// Returns true if the sensor data was read successfully, false otherwise.
bool readRawAccelerationMps2Internal(float &ax, float &ay, float &az) {
	if (!sensorReady) {
		return false;
	}

	sensors_event_t accel;
	sensors_event_t gyro;
	sensors_event_t temp;
	mpu.getEvent(&accel, &gyro, &temp);

	ax = accel.acceleration.x;
	ay = accel.acceleration.y;
	az = accel.acceleration.z;
	return true;
}

// Returns true if the sensor data was read successfully, false otherwise.
void readAccelerationG(float &axG, float &ayG, float &azG) {
	float ax = 0.0f;
	float ay = 0.0f;
	float az = 0.0f;
	readAccelerationMps2(ax, ay, az);

	axG = ax / kGravityMps2;
	ayG = ay / kGravityMps2;
	azG = az / kGravityMps2;
}

void resetFrontBackEstimate() {
	frontBackVelocityMps = 0.0f;
	frontBackPositionM = 0.0f;
	lastFrontBackUpdateMs = millis();
	lastSignificantAccelMs = lastFrontBackUpdateMs;
}

void setGravityReferenceFromAverage(float avgX, float avgY, float avgZ) {
	const float magnitude = sqrtf((avgX * avgX) + (avgY * avgY) + (avgZ * avgZ));
	if (magnitude <= 0.0001f) {
		gravityRefX = 0.0f;
		gravityRefY = 0.0f;
		gravityRefZ = kGravityMps2;
		return;
	}

	const float downX = avgX / magnitude;
	const float downY = avgY / magnitude;
	const float downZ = avgZ / magnitude;
	gravityRefX = downX * kGravityMps2;
	gravityRefY = downY * kGravityMps2;
	gravityRefZ = downZ * kGravityMps2;
}

}  // namespace

void initMpu6050() {
	Wire.begin(I2C_SDA, I2C_SCL);
	sensorReady = mpu.begin();
	if (!sensorReady) {
		return;
	}

	mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
	mpu.setGyroRange(MPU6050_RANGE_250_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
	resetFrontBackEstimate();
}

void calibrateMpu6050(int samples) {
	if (!sensorReady) {
		return;
	}

	float sumX = 0.0f;
	float sumY = 0.0f;
	float sumZ = 0.0f;

	for (int i = 0; i < samples; ++i) {
		sensors_event_t accel;
		sensors_event_t gyro;
		sensors_event_t temp;
		mpu.getEvent(&accel, &gyro, &temp);
		sumX += accel.acceleration.x;
		sumY += accel.acceleration.y;
		sumZ += accel.acceleration.z;
		delay(5);
	}

	accelOffsetX = 0.0f;
	accelOffsetY = 0.0f;
	accelOffsetZ = 0.0f;
	setGravityReferenceFromAverage(sumX / samples, sumY / samples, sumZ / samples);
	resetFrontBackEstimate();
}

void calibrateFrontBackDirection(int samples) {
	if (!sensorReady) {
		return;
	}

	(void)samples;
	resetFrontBackEstimate();
}

bool readHeadTilt(float &pitchDeg, float &rollDeg) {
	if (!sensorReady) {
		return false;
	}

	float axG = 0.0f;
	float ayG = 0.0f;
	float azG = 0.0f;
	readAccelerationG(axG, ayG, azG);

	pitchDeg = atan2f(axG, sqrtf((ayG * ayG) + (azG * azG))) * 180.0f / PI;
	rollDeg = atan2f(ayG, azG) * 180.0f / PI;
	return true;
}

bool isHeadTilted(float pitchDeg, float rollDeg) {
	return fabsf(pitchDeg) >= kTiltThresholdDeg || fabsf(rollDeg) >= kTiltThresholdDeg;
}

bool readFrontBackDisplacementCm(float &frontBackDisplacementCm) {
	if (!sensorReady) {
		return false;
	}

	float axG = 0.0f;
	float ayG = 0.0f;
	float azG = 0.0f;
	readAccelerationG(axG, ayG, azG);

	const float projectedAccelG = ayG;

	const unsigned long now = millis();
	float dt = 0.0f;
	if (lastFrontBackUpdateMs != 0) {
		dt = static_cast<float>(now - lastFrontBackUpdateMs) / 1000.0f;
	}
	lastFrontBackUpdateMs = now;

	float linearAccelerationMps2 = projectedAccelG * kGravityMps2;
	const bool significantAcceleration = fabsf(linearAccelerationMps2) >= kAccelerationDeadbandMps2;
	if (!significantAcceleration) {
		linearAccelerationMps2 = 0.0f;
		if (now - lastSignificantAccelMs > kShortTermHoldMs) {
			resetFrontBackEstimate();
			frontBackDisplacementCm = 0.0f;
			return true;
		}

		frontBackVelocityMps *= kVelocityDecayFactor;
		frontBackPositionM *= kPositionDecayFactor;
	} else {
		lastSignificantAccelMs = now;
	}

	frontBackVelocityMps += linearAccelerationMps2 * dt;
	frontBackPositionM += frontBackVelocityMps * dt;
	frontBackDisplacementCm = frontBackPositionM * 100.0f;
	return true;
}

bool readCalibratedAccelerationMps2(float &ax, float &ay, float &az) {
	return readAccelerationMps2(ax, ay, az);
}
