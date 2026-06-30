#include <Arduino.h>
#include <Wire.h>

// ========================
// MPU6050
// ========================
#define MPU_ADDR 0x68

// ========================
// MPU VARIABLES
// ========================
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;

float gyroX_offset = -59.0;

// ========================
// ANGLE VARIABLES
// ========================
float angle = 0;
float angle_acc = 0;
float gyro_rate = 0;

// ========================
// TIMING
// ========================
unsigned long lastTime;
float dt;

void setup() {
    Serial.begin(115200);

    Wire.begin(5, 18);

    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU NOT FOUND");
        while (1);
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();

    lastTime = micros();

    Serial.println("Experiment Ready");
}

void loop() {
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    if (dt < 0.001) dt = 0.001;
    if (dt > 0.05) dt = 0.05;

    // MPU READ
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, 14);

    if (Wire.available() < 14) return;

    AccX = Wire.read() << 8 | Wire.read();
    AccY = Wire.read() << 8 | Wire.read();
    AccZ = Wire.read() << 8 | Wire.read();

    Wire.read(); Wire.read(); // skip temp

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    // GYRO PROCESSING
    float gyroX = GyroX - gyroX_offset;
    gyro_rate = gyroX / 131.0;

    // ACCEL ANGLE
    float ax = AccX / 16384.0;
    float az = AccZ / 16384.0;

    angle_acc = atan2(ax, az) * 180 / PI;

    // COMPLEMENTARY FILTER
    angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * angle_acc;

    float actual_angle = angle;

    Serial.print("Angle: ");
    Serial.print(actual_angle);
    Serial.print(" | Accel: ");
    Serial.print(angle_acc);
    Serial.print(" | Gyro Rate: ");
    Serial.println(gyro_rate);

    delay(5);
}
