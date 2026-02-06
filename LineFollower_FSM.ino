#include "mbed.h"

/* ================= PIN DEFINITIONS ================= */
// TB6612FNG
#define AIN1 2
#define AIN2 3
#define PWMA 9

#define BIN1 4
#define BIN2 5
#define PWMB 10

#define STBY 6

/* ================= IR SENSORS ================= */
int irPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
float irValue[8];
int irMax[8] = {0,0,0,0,0,0,0,0};
int irMin[8] = {4095,4095,4095,4095,4095,4095,4095,4095};

/* ================= PID VARIABLES ================= */
float Kp = 60;
float Ki = 0;
float Kd = 40;
float Kspeed = 8;

float error = 0, prevError = 0;
float integral = 0, derivative = 0;
float PID = 0;

/* ================= SPEED ================= */
int baseSpeed = 200;
int dynamicSpeed = baseSpeed;

/* ================= TIME ================= */
unsigned long lastTime = 0;

/* ================= FSM ================= */
enum RobotState {
  FOLLOW_LINE,
  JUNCTION,
  TURN_LEFT,
  TURN_RIGHT,
  GO_STRAIGHT,
  LINE_LOST
};

RobotState state = FOLLOW_LINE;

/* ================= PWM ================= */
mbed::PwmOut pwmA(digitalPinToPinName(PWMA));
mbed::PwmOut pwmB(digitalPinToPinName(PWMB));

/* ================= CONSTANTS ================= */
const float BLACK_TH = 0.75;

/* ================= UTILS ================= */
float applyDeadband(float duty) {
  const float MIN_DUTY = 0.12f;
  if (duty > 0 && duty < MIN_DUTY) return MIN_DUTY;
  return duty;
}

/* ================= IR CALIBRATION ================= */
void calibrateIR(unsigned long durationMs = 3000) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    for (int i = 0; i < 8; i++) {
      int v = analogRead(irPins[i]);
      irMin[i] = min(irMin[i], v);
      irMax[i] = max(irMax[i], v);
    }
  }
}

float readIR(int i) {
  float raw = analogRead(irPins[i]);
  int range = irMax[i] - irMin[i];
  if (range < 10) return 0;
  return constrain((raw - irMin[i]) / range, 0.0f, 1.0f);
}

/* ================= SENSOR LOGIC ================= */
bool seesCenter() { return irValue[3] > BLACK_TH || irValue[4] > BLACK_TH; }
bool seesLeft()   { return irValue[0] > BLACK_TH || irValue[1] > BLACK_TH; }
bool seesRight()  { return irValue[6] > BLACK_TH || irValue[7] > BLACK_TH; }

int blackCount() {
  int c = 0;
  for (int i = 0; i < 8; i++)
    if (irValue[i] > BLACK_TH) c++;
  return c;
}

/* ================= MOTORS ================= */
void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(AIN1, speed >= 0);
  digitalWrite(AIN2, speed < 0);
  pwmA.write(applyDeadband(abs(speed) / 255.0f));
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(BIN1, speed >= 0);
  digitalWrite(BIN2, speed < 0);
  pwmB.write(applyDeadband(abs(speed) / 255.0f));
}

void stopMotors() {
  pwmA.write(0);
  pwmB.write(0);
}

void pivotLeft() {
  motorLeft(-140);
  motorRight(140);
}

void pivotRight() {
  motorLeft(140);
  motorRight(-140);
}

/* ================= PID ================= */
void runPID() {
  int weights[8] = {-16,-9,-4,-1,1,4,9,16};
  float weightedSum = 0, valueSum = 0;

  for (int i = 0; i < 8; i++) {
    weightedSum += weights[i] * irValue[i];
    valueSum += irValue[i];
  }

  error = (valueSum > 0.001) ? weightedSum / valueSum : prevError;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  integral += error * dt;
  integral = constrain(integral, -50, 50);

  derivative = (dt > 0.0001) ? (error - prevError) / dt : 0;
  PID = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  dynamicSpeed = constrain(baseSpeed - Kspeed * abs(error), 100, baseSpeed);

  motorLeft(dynamicSpeed + PID);
  motorRight(dynamicSpeed - PID);
}

/* ================= SETUP ================= */
void setup() {
  analogReadResolution(12);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pwmA.period_us(100);
  pwmB.period_us(100);

  delay(1000);
  calibrateIR();
}

/* ================= LOOP ================= */
void loop() {
  for (int i = 0; i < 8; i++)
    irValue[i] = readIR(i);

  int blacks = blackCount();

  switch (state) {

    case FOLLOW_LINE:
      if (blacks >= 5) {
        stopMotors();
        state = JUNCTION;
      }
      else if (!seesCenter()) {
        stopMotors();
        state = LINE_LOST;
      }
      else {
        runPID();
      }
      break;

    case JUNCTION:
      motorLeft(120);
      motorRight(120);
      delay(20);

      if (seesLeft())       state = TURN_LEFT;
      else if (seesCenter()) state = GO_STRAIGHT;
      else if (seesRight())  state = TURN_RIGHT;
      else                   state = LINE_LOST;

      integral = 0;
      break;

    case TURN_LEFT:
      pivotLeft();
      if (seesCenter()) {
        prevError = 0;
        state = FOLLOW_LINE;
      }
      break;

    case TURN_RIGHT:
      pivotRight();
      if (seesCenter()) {
        prevError = 0;
        state = FOLLOW_LINE;
      }
      break;

    case GO_STRAIGHT:
      motorLeft(150);
      motorRight(150);
      delay(60);
      state = FOLLOW_LINE;
      break;

    case LINE_LOST:
      integral = 0;
      (prevError > 0) ? pivotRight() : pivotLeft();
      if (seesCenter()) {
        prevError = 0;
        state = FOLLOW_LINE;
      }
      break;
  }
}
