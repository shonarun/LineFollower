// To check the reflectance...

int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  analogReadResolution(12);
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(irPins[i]);
    Serial.print(raw);
    Serial.print(", ");
  }
  Serial.println("");
  delay(2000);

}
