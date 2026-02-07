// To see normalized values

// IR Sensors
int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
float irValue[8];
int irMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int irMin[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};

// Normalization
void calibrateIR(unsigned long durationMs = 40000) {
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
  return norm;
  return constrain(norm, 0.0f, 1.0f);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  analogReadResolution(12);

  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

  delay(1000);
  calibrateIR();

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 8; i++) {
    Serial.print(readIR(i));
    Serial.print(", ");
  }
  Serial.println("");
  
}
