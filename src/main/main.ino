#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ==========================
// Pin definitions
// ==========================
const int turbidityPin = A0;
const int phPin        = A1;
const int tdsPin       = A2;

#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ==========================
// Turbidity calibration
// ==========================
float clearVoltage = 3.905;
float dirtyVoltage = 3.847;

// ==========================
// pH calibration
// ==========================
float ph_m = 1.0551;
float ph_c = 0.2647;

// ==========================
// TDS settings
// ==========================
#define VREF 5.0
#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];

// ==========================
// WQI parameters
// ==========================
const float STD_TDS_PPM   = 500.0;
const float STD_TURB_NTU  = 5.0;
const float PH_LOW_LIMIT  = 6.5;
const float PH_HIGH_LIMIT = 8.5;
const float TEMP_IDEAL_C  = 25.0;
const float TEMP_MAX_C    = 35.0;

// Weights
float wPH   = 4.0;
float wTurb = 4.0;
float wTDS  = 3.0;
float wTemp = 1.0;

// ==========================
// Utility
// ==========================
float clamp100(float x) {
  if (x < 0.0) return 0.0;
  if (x > 100.0) return 100.0;
  return x;
}

// ==========================
// Median filter
// ==========================
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }

  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }

  return bTemp;
}

// ==========================
// Read temperature
// ==========================
float readTemperatureC() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C || tempC < -20 || tempC > 80) {
    tempC = 25.0;
  }

  return tempC;
}

// ==========================
// Read pH
// ==========================
float readPH() {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(phPin);
    delay(20);
  }

  float avgRaw = sum / 10.0;
  float voltage = avgRaw * (5.0 / 1023.0);

  float measuredPH = 7.0 + ((2.5 - voltage) / 0.18);
  float correctedPH = (ph_m * measuredPH) + ph_c;

  return correctedPH;
}

// ==========================
// Read turbidity
// ==========================
float readTurbidityNTU() {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(turbidityPin);
    delay(20);
  }

  float avgRaw = sum / 10.0;
  float voltage = avgRaw * (5.0 / 1023.0);

  float ntu = (clearVoltage - voltage) * (3000.0 / (clearVoltage - dirtyVoltage));

  if (ntu < 0) ntu = 0;
  if (ntu > 3000) ntu = 3000;

  return ntu;
}

// ==========================
// Read TDS
// ==========================
float readTDS(float temperatureC) {
  for (int i = 0; i < SCOUNT; i++) {
    analogBuffer[i] = analogRead(tdsPin);
    delay(20);
  }

  for (int i = 0; i < SCOUNT; i++) {
    analogBufferTemp[i] = analogBuffer[i];
  }

  float averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

  float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
  if (compensationCoefficient <= 0) compensationCoefficient = 1.0;

  float compensationVoltage = averageVoltage / compensationCoefficient;

  float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
                  - 255.86 * compensationVoltage * compensationVoltage
                  + 857.39 * compensationVoltage) * 0.5;

  if (tdsValue < 0) tdsValue = 0;

  return tdsValue;
}

// ==========================
// WQI sub-index functions
// ==========================
float q_tds(float tdsPpm) {
  float q = (tdsPpm / STD_TDS_PPM) * 100.0;
  return clamp100(q);
}

float q_turb(float ntu) {
  float q = (ntu / STD_TURB_NTU) * 100.0;
  return clamp100(q);
}

float q_temp(float tC) {
  float denom = (TEMP_MAX_C - TEMP_IDEAL_C);
  if (denom <= 0.0) return 0.0;

  float q = (fabs(tC - TEMP_IDEAL_C) / denom) * 100.0;
  return clamp100(q);
}

float q_ph(float ph) {
  float denom;
  if (ph >= 7.0) denom = (PH_HIGH_LIMIT - 7.0);
  else           denom = (7.0 - PH_LOW_LIMIT);

  if (denom <= 0.0) return 0.0;

  float q = (fabs(ph - 7.0) / denom) * 100.0;
  return clamp100(q);
}

// ==========================
// Compute WQI
// ==========================
float computeWQI(float ph, float tdsPpm, float ntu, float tempC) {
  float sumW = wPH + wTurb + wTDS + wTemp;
  if (sumW <= 0.0) return 0.0;

  float Wph   = wPH   / sumW;
  float Wturb = wTurb / sumW;
  float Wtds  = wTDS  / sumW;
  float Wtemp = wTemp / sumW;

  float wqi = (Wph * q_ph(ph)) +
              (Wturb * q_turb(ntu)) +
              (Wtds * q_tds(tdsPpm)) +
              (Wtemp * q_temp(tempC));

  return clamp100(wqi);
}

const char* wqiCategory(float wqi) {
  if (wqi <= 25.0) return "PURE";
  if (wqi <= 50.0) return "GOOD";
  if (wqi <= 75.0) return "NORMAL";
  return "POOR";
}

void setup() {
  Serial.begin(9600);

  pinMode(turbidityPin, INPUT);
  pinMode(phPin, INPUT);
  pinMode(tdsPin, INPUT);

  sensors.begin();

  Serial.println("Water Quality Monitoring System");
  Serial.println("======================================");
}

void loop() {
  float temperatureC  = readTemperatureC();
  float pHValue       = readPH();
  float turbidityNTU  = readTurbidityNTU();
  float tdsValue      = readTDS(temperatureC);

  float wqi = computeWQI(pHValue, tdsValue, turbidityNTU, temperatureC);
  const char* category = wqiCategory(wqi);

  Serial.print("Temperature: ");
  Serial.print(temperatureC, 2);
  Serial.println(" C");

  Serial.print("pH: ");
  Serial.println(pHValue, 2);

  Serial.print("Turbidity: ");
  Serial.print(turbidityNTU, 1);
  Serial.println(" NTU");

  Serial.print("TDS: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");

  Serial.print("WQI: ");
  Serial.println(wqi, 2);

  Serial.print("Category: ");
  Serial.println(category);

  Serial.println("--------------------------------------");

  delay(1000);
}