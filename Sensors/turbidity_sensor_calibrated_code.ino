const int turbidityPin = A0;

// Change these after testing your sensor
float clearVoltage = 3.905;   // voltage in clear water
float dirtyVoltage = 3.847;   // voltage in very turbid water

void setup() {
  Serial.begin(9600);
}

void loop() {
  int rawValue = analogRead(turbidityPin);
  float voltage = rawValue * (5.0 / 1023.0);

  float ntu = (clearVoltage - voltage) * (3000.0 / (clearVoltage - dirtyVoltage));

  if (ntu < 0) ntu = 0;
  if (ntu > 3000) ntu = 3000;

  Serial.print("Raw: ");
  Serial.print(rawValue);
  Serial.print("  Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V  NTU: ");
  Serial.println(ntu, 1);

  delay(1000);
}