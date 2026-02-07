// To check motor speeds with MDD3A motor driver...
#include "mbed.h"

// PIN DEFINITIONS
// MDD3A Pins
#define AIN1 2
#define AIN2 3

#define BIN1 5
#define BIN2 6

mbed::PwmOut pwmA1(digitalPinToPinName(AIN1));
// mbed::PwmOut pwmA2(digitalPinToPinName(AIN2));
mbed::PwmOut pwmB1(digitalPinToPinName(BIN1));
// mbed::PwmOut pwmB2(digitalPinToPinName(BIN2));

void setup() {
  // put your setup code here, to run once:

  // pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);


  pwmA1.period_us(50);
  pwmB1.period_us(50);
  // pwmA2.period_us(100);
  // pwmB2.period_us(100);

  // pwmA.write(0.1f);
  // pwmB.write(0.1f);

  pwmA1.write(0.6f);
  // pwmA2.write(0.0f);
  pwmB1.write(0.6f);
  // pwmB2.write(0.0f);

}

void loop() {
  // put your main code here, to run repeatedly:

}
