#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

VL53L0X lidar;
MPU6050 mpu;

float yaw = 0;
unsigned long lastTime = 0;

// motor 1 pins (left motor)
const int encoderPinC1 = 16;
const int encoderPinC2 = 23;
const int enablePin = 14;
const int input1Pin = 26;
const int input2Pin = 27;

// motor 2 pins (right motor)
const int encoder2PinC1 = 17;
const int encoder2PinC2 = 4;
const int enable2Pin = 25;
const int input3Pin = 33;
const int input4Pin = 32;

const int irLeftPin = 34;
const int irRightPin = 35;

const int irThreshold = 300;
const int lidarThreshold = 100;
// different speeds where used for each motor to make the forward movement straight (as perfect as possible)
const int motorSpeed = 255;
const int motor2Speed = 248;
const int TICKS_PER_20CM = 382;  // this value was adjusted based on testing

volatile long Ticks = 0;
bool useLeftHand = true;  // Start with left-hand rule, can be toggled
void IRAM_ATTR EncoderISR() {
    Ticks++;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!lidar.init()) {
        Serial.println("Failed to detect LiDAR sensor!");
        while (1);
    }
    lidar.setTimeout(500);
    lidar.startContinuous();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    calibrateMPU();

    pinMode(encoderPinC1, INPUT);
    pinMode(encoderPinC2, INPUT);
    pinMode(encoder2PinC1, INPUT);
    pinMode(encoder2PinC2, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinC1), EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2PinC1), EncoderISR, RISING);

    pinMode(enablePin, OUTPUT);
    pinMode(input1Pin, OUTPUT);
    pinMode(input2Pin, OUTPUT);
    pinMode(enable2Pin, OUTPUT);
    pinMode(input3Pin, OUTPUT);
    pinMode(input4Pin, OUTPUT);

    pinMode(irLeftPin, INPUT);
    pinMode(irRightPin, INPUT);

    analogWrite(enablePin,motorSpeed);
    analogWrite(enable2Pin, motor2Speed);
    stopMotors();
    Serial.println("Motors Stopped.");
    delay(5000);
}

float gyroZOffset = 0;

// calibrating the mpu for better precision while rotating
void calibrateMPU() {
    yaw = 0;
    lastTime = millis();
    long sum = 0;
    int n = 500;
    for (int i = 0; i < n; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(2);
    }
    gyroZOffset = sum / (float)n;
    Serial.print("Gyro Z Offset: "); Serial.println(gyroZOffset);
}

float getYawChange() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    float adjustedGz = (gz - gyroZOffset) / 131.0;
    return adjustedGz * dt;
}

// rotate either left or right based on the input direction
void rotate90(bool left) {
    Serial.println(left ? "Turning LEFT..." : "Turning RIGHT...");
    calibrateMPU();
    float targetAngle = (left ? -90 : 90) * 0.95; // 0.95 is a ratio for precision in rotation, providing target angle 90 degrees exactly was not useful
    yaw = 0;

    digitalWrite(input1Pin, left ? LOW : HIGH);
    digitalWrite(input2Pin, left ? HIGH : LOW);
    digitalWrite(input3Pin, left ? LOW : HIGH);
    digitalWrite(input4Pin, left ? HIGH : LOW);

    while (abs(yaw) < abs(targetAngle)) {
        yaw += getYawChange();
        delay(1);
    }
    stopMotors();
    Serial.println("Motors Stopped.");
}

void stopMotors() {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, LOW);
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, LOW);
}

// the robot should move 20cm each time
void moveForward20cm() {
    Ticks = 0;
    digitalWrite(input1Pin, HIGH);
    digitalWrite(input2Pin, LOW);
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, HIGH);
    Serial.println("Moving Forward...");

    while (Ticks < TICKS_PER_20CM) {
        if (lidar.readRangeContinuousMillimeters() < lidarThreshold) {
            stopMotors();
            Serial.println("Motors Stopped.");
            return;
        }
    }
    stopMotors();
    Serial.println("Motors Stopped.");
    calibrateMPU();
}

void loop() {
    stopMotors();
    int irLeftValue = analogRead(irLeftPin);
    int irRightValue = analogRead(irRightPin);
    int lidarDistance = lidar.readRangeContinuousMillimeters();

    Serial.print("LiDAR Distance: "); Serial.println(lidarDistance);

    if (lidarDistance < lidarThreshold) {
        stopMotors();
        delay(100);  // Small delay for stability
        irLeftValue = analogRead(irLeftPin);
        irRightValue = analogRead(irRightPin);

        if (irLeftValue > irThreshold) {
            rotate90(true);
            stopMotors();
            moveForward20cm();
        } else if (irRightValue > irThreshold) {
            rotate90(false);
            stopMotors();
            moveForward20cm();
        } else {
            rotate90(true);
            rotate90(true);
            stopMotors();
            moveForward20cm();
        }
    } else {
        moveForward20cm();
        stopMotors();
    }
    delay(100);
}
