// To check motor speeds...
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

mbed::PwmOut pwmA(digitalPinToPinName(PWMA));
mbed::PwmOut pwmB(digitalPinToPinName(PWMB));

void setup() {
  // put your setup code here, to run once:

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  pwmA.period_us(100);
  pwmB.period_us(100);

  // pwmA.write(0.1f);
  // pwmB.write(0.1f);

  pwmA.write(0.8f);
  pwmB.write(0.8f);

}

void loop() {
  // put your main code here, to run repeatedly:

}
