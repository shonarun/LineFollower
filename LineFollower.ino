// The Original

#include <ArduinoBLE.h>

// BLE SERVICE 
BLEService pidService("12345678-1234-1234-1234-1234567890ab");

BLEFloatCharacteristic kpChar("12345678-1234-1234-1234-1234567890ac", BLERead | BLEWrite);
BLEFloatCharacteristic kdChar("12345678-1234-1234-1234-1234567890ad", BLERead | BLEWrite);
BLEFloatCharacteristic kiChar("12345678-1234-1234-1234-1234567890ae", BLERead | BLEWrite);

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
float Kp = 60;
float Ki = 0.0;
float Kd = 8;

float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float PID = 0;

// SPEED
int baseSpeed = 40;

// TIME
unsigned long lastTime = 0;

// MOTOR FUNCTIONS
void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  }
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

//  SETUP 
  void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  // BLE INIT
  BLE.begin();
  BLE.setLocalName("Nano33_PID_LineFollower");
  BLE.setAdvertisedService(pidService);

  pidService.addCharacteristic(kpChar);
  pidService.addCharacteristic(kiChar);
  pidService.addCharacteristic(kdChar);

  BLE.addService(pidService);

  kpChar.writeValue(Kp);
  kiChar.writeValue(Ki);
  kdChar.writeValue(Kd);

  BLE.advertise();
}

// LOOP
void loop() {
  BLE.poll();

  if (kpChar.written()) {
    Kp = kpChar.value();
    Serial.println("");
    Serial.print("Kp changed to ");
    Serial.println(Kp);
  }
  if (kiChar.written()) {
    Ki = kiChar.value();
    Serial.println("");
    Serial.print("Ki changed to ");
    Serial.println(Ki);
  }
  if (kdChar.written()) {
    Kd = kdChar.value();
    Serial.println("");
    Serial.print("Kd changed to ");
    Serial.println(Kd);
  }

  readSensors();
  calculateError();
  computePID();
  driveMotors();
  delay(5);
}

//  FUNCTIONS 

void readSensors() {
  for (int i = 0; i < 8; i++) {
    irValue[i] = digitalRead(irPins[i]);
  }
}

void calculateError() {
  // Weighted error for 8 sensors
  // int weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
  int weights[8] = {16, 9, 4, 1, -1, -4, -9, -16};
  // int weights[8] = {64, 27, 8, 1, -1, -8, -27, -64};
  int sum = 0;
  int count = 0;

  for (int i = 0; i < 8; i++) {
    if (irValue[i] == LOW) {  // black line
      sum += weights[i];
      count++;
    }
  }
  
  if (count != 0)
  {
    error = (float)sum / count;
  }
  else
  {
    error = prevError;
    // error = prevError * 2;
  }

  Serial.print("Error: ");
  Serial.println(error);
}

void computePID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  integral += error * dt;
  derivative = (error - prevError) / dt;

  PID = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prevError = error;
}

void driveMotors() {
  int leftSpeed  = baseSpeed + PID;
  int rightSpeed = baseSpeed - PID;
  Serial.print("l: ");
  Serial.print(leftSpeed);
  Serial.print(" r: ");
  Serial.println(rightSpeed);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);
}
