// To check motor speeds with MDD3A motor driver...
#include "mbed.h"

// PIN DEFINITIONS
// MDD3A Pins
#define PWM_A 9
#define DIR_A 3

#define PWM_B 10
#define DIR_B 6

mbed::PwmOut pwmA(digitalPinToPinName(PWM_A));
mbed::PwmOut pwmB(digitalPinToPinName(PWM_B));

void kickStart(mbed::PwmOut& pwm, float targetDuty) {

  pwm.write(KICK_DUTY);
  delay(KICK_MS);

  pwm.write(targetDuty);
}

void softStart(mbed::PwmOut& pwm, float targetDuty) {
  const float KICK_DUTY = 0.75f;
  const int   PUSH_MS   = 20;

  for (float d = KICK_DUTY; d >= targetDuty; d -= 0.02f) {
    pwm.write(d);
    delay(PUSH_MS);
  }
}

void setup() {
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);

  pwmA.period_us(200);
  pwmB.period_us(200);

  pwmA.write(0.0f);
  pwmB.write(0.0f);

  delay(500); 

  pwmA.write(0.6f);
  pwmB.write(0.6f);

  // kickStart(pwmA, 0.35f);
  // kickStart(pwmB, 0.35f);

  // softStart(pwmA, 0.35f);
  // softStart(pwmB, 0.35f);

}

void loop() {

}
