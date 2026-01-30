#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

/* ===================== Pin Definitions ===================== */
// Left motor (IN1-IN4)
int L1 = 2;
int L2 = 3;
int L3 = 4;
int L4 = 5;

// Right motor (IN1-IN4)
int R1 = 6;
int R2 = 7;
int R3 = 8;
int R4 = 9;

/* ===================== Step Sequence ===================== */
// Full-step sequence (4-step)
int stepSeq[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

int leftStepIndex  = 0;
int rightStepIndex = 0;

/* ===================== Motion Parameters ===================== */
int baseDelay = 2000;     // Base step delay (microseconds)
float targetYaw = 0.0;    // Desired heading (degrees)

/* ===================== Yaw PID ===================== */
float kp = 3.0;
float ki = 0.0;
float kd = 0.5;

float yawError = 0;
float lastYawError = 0;
float yawIntegral = 0;

/* ===================== Yaw Estimation ===================== */
float yaw = 0;
unsigned long lastTime;

/* ===================== Functions ===================== */

void stepMotorLeft(bool forward) {
  leftStepIndex += forward ? 1 : -1;
  if (leftStepIndex < 0) leftStepIndex = 3;
  if (leftStepIndex > 3) leftStepIndex = 0;

  digitalWrite(L1, stepSeq[leftStepIndex][0]);
  digitalWrite(L2, stepSeq[leftStepIndex][1]);
  digitalWrite(L3, stepSeq[leftStepIndex][2]);
  digitalWrite(L4, stepSeq[leftStepIndex][3]);
}

void stepMotorRight(bool forward) {
  rightStepIndex += forward ? 1 : -1;
  if (rightStepIndex < 0) rightStepIndex = 3;
  if (rightStepIndex > 3) rightStepIndex = 0;

  digitalWrite(R1, stepSeq[rightStepIndex][0]);
  digitalWrite(R2, stepSeq[rightStepIndex][1]);
  digitalWrite(R3, stepSeq[rightStepIndex][2]);
  digitalWrite(R4, stepSeq[rightStepIndex][3]);
}

/* ===================== Setup ===================== */

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();

  pinMode(L1, OUTPUT); pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT); pinMode(L4, OUTPUT);
  pinMode(R1, OUTPUT); pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT); pinMode(R4, OUTPUT);

  lastTime = millis();
}

/* ===================== Loop ===================== */

void loop() {
  /* ---------- Read gyro ---------- */
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float yawRate = gz / 131.0;   // deg/s
  yaw += yawRate * dt;

  /* ---------- Yaw PID ---------- */
  yawError = targetYaw - yaw;
  yawIntegral += yawError * dt;
  float yawDerivative = (yawError - lastYawError) / dt;
  lastYawError = yawError;

  float correction = kp * yawError + kd * yawDerivative;

  /* ---------- Convert to motor delay ---------- */
  int leftDelay  = baseDelay - correction;
  int rightDelay = baseDelay + correction;

  leftDelay  = constrain(leftDelay, 800, 4000);
  rightDelay = constrain(rightDelay, 800, 4000);

  /* ---------- Step motors ---------- */
  stepMotorLeft(true);
  delayMicroseconds(leftDelay);

  stepMotorRight(true);
  delayMicroseconds(rightDelay);
}
