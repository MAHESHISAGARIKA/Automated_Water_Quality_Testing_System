const int phPin = A0;

// Linear calibration constants
const float m = 1.0551;
const float c = 0.2647;

void setup() {
  Serial.begin(9600);
}

void loop() {
  long sum = 0;

  // Take 10 readings in 1 second
  for (int i = 0; i < 10; i++) {
    sum += analogRead(phPin);
    delay(100);
  }

  float avgRaw = sum / 10.0;
  float voltage = avgRaw * (5.0 / 1023.0);

  // Original pH calculation
  float pHValue = 7 + ((2.5 - voltage) / 0.18);

  // Corrected pH using linear calibration
  float correctedPH = (m * pHValue) + c;

  Serial.print("Raw pH: ");
  Serial.print(pHValue, 2);
  Serial.print("  |  Corrected pH: ");
  Serial.println(correctedPH, 2);
}