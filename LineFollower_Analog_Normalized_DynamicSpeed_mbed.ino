#include "mbed.h"

// PIN DEFINITIONS
// TB6612FNG
#define AIN1 2
#define AIN2 3
#define PWMA 9

#define BIN1 4
#define BIN2 5
#define PWMB 10

#define STBY 6

// IR Sensors
int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
float irValue[8];
int irMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int irMin[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};

// PID VARIABLES
float Kp = 60;
float Ki = 0;
float Kd = 40;
float Kspeed = 8;

float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float PID = 0;

int w;

// SPEED
int baseSpeed = 200;
int dynamicSpeed = baseSpeed;

// TIME
unsigned long lastTime = 0;

float applyDeadband(float duty) {
  const float MIN_DUTY = 0.12f;  // tune this
  if (duty < MIN_DUTY && duty > 0.0f)
    return MIN_DUTY;
  return duty;
}

// Normalization
void calibrateIR(unsigned long durationMs = 3000) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    for (int i = 0; i < 8; i++) {
      int v = analogRead(irPins[i]);
      if (v < irMin[i]) irMin[i] = v;
      if (v > irMax[i]) irMax[i] = v;
    }
  }
}

float readIR(int i) {
  float raw = analogRead(irPins[i]);
  int range = irMax[i] - irMin[i];
  if (range < 10) return 0.0f;
  float norm = (raw - irMin[i]) / range;
  return constrain(norm, 0.0f, 1.0f);
}

mbed::PwmOut pwmA(digitalPinToPinName(PWMA));
mbed::PwmOut pwmB(digitalPinToPinName(PWMB));

// MOTOR FUNCTIONS
void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  
  float duty = abs(speed) / 255.0f;
  duty = applyDeadband(duty);
  pwmA.write(duty);
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  
  float duty = abs(speed) / 255.0f;
  duty = applyDeadband(duty);
  pwmB.write(duty);
}

void stopMotors() {
  pwmA.write(0.0f);
  pwmB.write(0.0f);
}

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);

  analogReadResolution(12);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pwmA.period_us(100);
  pwmB.period_us(100);

  // Start stopped
  pwmA.write(0.0f);
  pwmB.write(0.0f);
  
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

  delay(1000);
  calibrateIR();

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 8; i++) {
    irValue[i] = readIR(i);
  }
  // Weighted error for 8 sensors
  // int weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
  int weights[8] = {-16, -9, -4, -1, 1, 4, 9, 16};
  // int weights[8] = {-25, -16, -9, -4, 4, 9, 16, 25};
  // int weights[8] = {64, 27, 8, 1, -1, -8, -27, -64};
  // float sum = 0;
  // int count = 0;
  float weightedSum = 0;
  float valueSum = 0;

  for (int i = 0; i < 8; i++) {
    w = weights[i];
    // Serial.print(irValue[i]);
    // Serial.print(", ");
    // if (irValue[i] > 0.85) {  // black line
    // // if (irValue[i] == LOW) {  // black line
      weightedSum  += w * irValue[i];
      valueSum += irValue[i];
    //   count++;
    // }
  }
  // Serial.println("");
  
  // error = sum / 8.0f;
  
  // if (count != 0) error = (float)sum / count;
  // else error = prevError;

  if (valueSum > 0.001) error = weightedSum / valueSum;
  else error = prevError;  // line lost handling

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  integral += error * dt;
  if (dt > 0.0001) {
    derivative = (error - prevError) / dt;
    // integral = 0;
  }
  else {
    derivative = 0;
  }

  PID = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prevError = error;

  dynamicSpeed = baseSpeed - Kspeed * abs(error);
  dynamicSpeed = constrain(dynamicSpeed, 100, baseSpeed);
  int leftSpeed  = dynamicSpeed + PID;
  int rightSpeed = dynamicSpeed - PID;
  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  // Serial.print("Error: ");
  // Serial.println(error);
  // Serial.print("l: ");
  // Serial.print(leftSpeed);
  // Serial.print(" r: ");
  // Serial.println(rightSpeed);
  // delay(2000);
}
