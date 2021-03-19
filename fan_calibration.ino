void setup() {
  // initialise pin 16
  pinMode(16, OUTPUT);
}

void loop() {
  // swiitch fan on and off repeatedly
  digitalWrite(16, HIGH);
  delay(5000);
  digitalWrite(16, LOW);
  delay(5000);
}
