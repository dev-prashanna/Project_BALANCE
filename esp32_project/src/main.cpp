#include <Wire.h>

// ========================
// PIN DEFINITIONS
// ========================
#define MPU_ADDR 0x68

#define ENA 4
#define IN1 2
#define IN2 19
#define IN3 21
#define IN4 22
#define ENB 23

// ========================
// PID GAINS
// ========================
float Kp = 5.0;
float Ki = 0.0;
float Kd = 0.0;

// ========================
// PRECOMPUTED RECIPROCALS
// ========================
#define INV_131   0.00763358779f
#define INV_16384 0.00006103515f

// ========================
// THRESHOLDS
// ========================
#define DEAD_ZONE     2.0f
#define MIN_PWM       60
#define MAX_PWM       180
#define MAX_INTEGRAL  50.0f
#define FALL_ANGLE    45.0f
#define CAL_SAMPLES   500
#define SERIAL_RATE   20

// ========================
// STATE
// ========================
int16_t AccX, AccZ;
int16_t GyroX;

float gyro_offset = 0;
float angle = 0;
float prevError = 0;
float integral = 0;
unsigned long lastTime;
uint8_t loopCount = 0;
bool fallen = false;

// ========================
// SERIAL TUNING BUFFER
// ========================
char cmdBuf[16];
uint8_t cmdLen = 0;

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                float val = atof(cmdBuf + 1);
                switch (cmdBuf[0]) {
                    case 'p': case 'P':
                        Kp = val;
                        Serial.print("Kp = "); Serial.println(Kp, 4);
                        break;
                    case 'i': case 'I':
                        Ki = val;
                        Serial.print("Ki = "); Serial.println(Ki, 4);
                        break;
                    case 'd': case 'D':
                        Kd = val;
                        Serial.print("Kd = "); Serial.println(Kd, 4);
                        break;
                    case 'z': case 'Z':
                        integral = 0;
                        prevError = 0;
                        Serial.println("Integral reset");
                        break;
                    case '?':
                        Serial.print("Kp="); Serial.print(Kp, 4);
                        Serial.print(" Ki="); Serial.print(Ki, 4);
                        Serial.print(" Kd="); Serial.println(Kd, 4);
                        break;
                    default:
                        Serial.println("p<val> i<val> d<val> z ?(query)");
                        break;
                }
                cmdLen = 0;
            }
        } else if (cmdLen < 15) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

// ========================
// MPU I2C
// ========================
void mpuWrite(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void i2cRecover() {
    Wire.end();
    pinMode(5, INPUT_PULLUP);
    pinMode(18, INPUT_PULLUP);
    for (int i = 0; i < 9; i++) {
        digitalWrite(18, LOW);
        delayMicroseconds(5);
        digitalWrite(18, HIGH);
        delayMicroseconds(5);
    }
    Wire.begin(5, 18);
    Wire.setClock(100000);
}

bool mpuRead() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    uint8_t count = Wire.requestFrom(MPU_ADDR, 14, true);
    if (count < 14) {
        i2cRecover();
        return false;
    }

    AccX = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    AccZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    GyroX = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    return true;
}

// ========================
// CALIBRATION
// ========================
void calibrateMPU() {
    Serial.print("Calibrating... keep UPRIGHT & STILL");

    float sum = 0;
    int valid = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        if (mpuRead()) {
            sum += (float)GyroX;
            valid++;
        }
        delay(2);
    }
    gyro_offset = (valid > 0) ? sum / valid : 0;

    Serial.print(" Done! Offset: ");
    Serial.println(gyro_offset);
}

// ========================
// MOTOR CONTROL
// ========================
void setMotorPWM(int speed) {
    if (speed > 0) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else if (speed < 0) {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
        speed = -speed;
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    }

    if (speed > MAX_PWM) speed = MAX_PWM;
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
}

// ========================
// SETUP
// ========================
void setup() {
    Serial.begin(115200);

    Wire.begin(5, 18);
    Wire.setClock(100000);

    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU6050 NOT FOUND!");
        while (1);
    }

    mpuWrite(0x6B, 0x00);
    mpuWrite(0x1A, 0x03);
    mpuWrite(0x1B, 0x00);
    mpuWrite(0x1C, 0x00);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    ledcAttach(ENA, 1000, 8);
    ledcAttach(ENB, 1000, 8);

    calibrateMPU();

    lastTime = micros();
    Serial.println("Ready! p<val> i<val> d<val> z(reset) ?(query)");
}

// ========================
// LOOP
// ========================
void loop() {
    handleSerial();

    unsigned long now = micros();
    float dt = (now - lastTime) * 0.000001f;
    lastTime = now;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;

    if (!mpuRead()) return;

    float gyro_rate = -(GyroX - gyro_offset) * INV_131;
    float ax = AccX * INV_16384;
    float az = AccZ * INV_16384;

    float angle_acc = -atan2(ax, az) * 57.29577951f;

    angle = 0.98f * (angle + gyro_rate * dt) + 0.02f * angle_acc;

    if (fallen) {
        if (abs(angle_acc) < DEAD_ZONE) {
            fallen = false;
            angle = angle_acc;
            integral = 0;
            prevError = 0;
            Serial.println("RECOVERED");
        } else {
            setMotorPWM(0);
            return;
        }
    }

    if (angle > FALL_ANGLE || angle < -FALL_ANGLE) {
        setMotorPWM(0);
        angle = angle_acc;
        integral = 0;
        prevError = 0;
        fallen = true;
        Serial.println("FALLEN");
        return;
    }

    float error = angle;
    integral += error * dt;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

    float derivative = (error - prevError) / dt;
    prevError = error;

    float pid = Kp * error + Ki * integral + Kd * derivative;
    if (pid > MAX_PWM) pid = MAX_PWM;
    else if (pid < -MAX_PWM) pid = -MAX_PWM;

    if (angle > -DEAD_ZONE && angle < DEAD_ZONE) {
        setMotorPWM(0);
    } else {
        int pwm = (int)pid;
        int absPwm = pwm < 0 ? -pwm : pwm;
        if (absPwm > 0 && absPwm < MIN_PWM) {
            pwm = pwm > 0 ? MIN_PWM : -MIN_PWM;
        }
        setMotorPWM(pwm);
    }

    if (++loopCount >= SERIAL_RATE) {
        loopCount = 0;
        Serial.print(angle, 1);
        Serial.print(" e=");
        Serial.print(error, 1);
        Serial.print(" p=");
        Serial.print((int)(Kp * error));
        Serial.print(" i=");
        Serial.print((int)(Ki * integral));
        Serial.print(" d=");
        Serial.print((int)(Kd * derivative));
        Serial.print(" pwm=");
        Serial.println((int)pid);
    }
}
