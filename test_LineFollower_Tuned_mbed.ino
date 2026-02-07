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
int irValue[8];

// PID VARIABLES
float Kp = 40;
float Ki = 0;
float Kd = 25;

float error_1 = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float PID = 0;

// SPEED
int baseSpeed = 200;

// TIME
unsigned long lastTime = 0;

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
  pwmB.write(duty);
}

void stopMotors() {
  pwmA.write(0.0f);
  pwmB.write(0.0f);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

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

  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 8; i++) {
    irValue[i] = digitalRead(irPins[i]);
  }
  // Weighted error for 8 sensors
  // int weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
  // int weights[8] = {16, 12, 4, 1, -1, -4, -12, -16};
  int weights[8] = {64, 27, 8, 1, -1, -8, -27, -64};
  int sum = 0;
  int count = 0;

  for (int i = 0; i < 8; i++) {
    if (irValue[i] == LOW) {  // black line
      sum += weights[i];
      count++;
    }
  }
  
  if (count != 0) error_1 = (float)sum / count;
  else error_1 = prevError;
  // Serial.print("Error: ");
  // Serial.println(error);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  integral += error_1 * dt;
  derivative = (error_1 - prevError) / dt;

  PID = (Kp * error_1) + (Ki * integral) + (Kd * derivative);
  prevError = error_1;
  
  int leftSpeed  = baseSpeed + PID;
  int rightSpeed = baseSpeed - PID;
  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  // Serial.print("l: ");
  // Serial.print(leftSpeed);
  // Serial.print(" r: ");
  // Serial.println(rightSpeed);
  delay(10);
}
