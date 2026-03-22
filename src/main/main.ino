#include <Wire.h>
#include <U8x8lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

/* ================= OLED ================= */
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);   // 0x3C default

/* ================= PINS ================= */
const int turbidityPin = A0;
const int phPin        = A1;
const int tdsPin       = A2;
#define ONE_WIRE_BUS   4

/* ================= DS18B20 ================= */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* ================= CONSTANTS ================= */
#define VREF        5.0
#define SCOUNT      30

/* ================= TURBIDITY CALIBRATION ================= */
float clearVoltage = 3.905;
float dirtyVoltage = 3.847;

/* ================= pH CALIBRATION ================= */
float ph_m = 1.0551;
float ph_c = 0.2647;

/* ================= TDS GLOBALS ================= */
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];

/* ================= WQI PARAMETERS ================= */
const float STD_TDS_PPM   = 500.0;
const float STD_TURB_NTU  = 5.0;
const float PH_LOW_LIMIT  = 6.5;
const float PH_HIGH_LIMIT = 8.5;
const float TEMP_IDEAL_C  = 25.0;
const float TEMP_MAX_C    = 35.0;

float wPH   = 4.0;
float wTurb = 4.0;
float wTDS  = 3.0;
float wTemp = 1.0;

/* ================= UTILITY ================= */
float clamp100(float x) {
  if (x < 0.0) return 0.0;
  if (x > 100.0) return 100.0;
  return x;
}

/* ================= MEDIAN FILTER ================= */
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

/* ================= READ TEMPERATURE ================= */
float readTemperatureC() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC == DEVICE_DISCONNECTED_C || tempC < -20 || tempC > 80) {
    tempC = 25.0;
  }

  return tempC;
}

/* ================= READ pH ================= */
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

  if (correctedPH < 0) correctedPH = 0;
  if (correctedPH > 14) correctedPH = 14;

  return correctedPH;
}

/* ================= READ TURBIDITY ================= */
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

/* ================= READ TDS ================= */
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

/* ================= WQI SUB-INDICES ================= */
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

/* ================= COMPUTE WQI ================= */
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

/* ================= SETUP ================= */
void setup() {
  Serial.begin(9600);

  pinMode(turbidityPin, INPUT);
  pinMode(phPin, INPUT);
  pinMode(tdsPin, INPUT);

  sensors.begin();

  oled.begin();
  oled.setPowerSave(0);
  oled.setFont(u8x8_font_victoriabold8_r);

  oled.clear();
  oled.drawString(0, 0, "Water Monitor");
  oled.drawString(0, 1, "Init OK");

  delay(1000);
}

/* ================= LOOP ================= */
void loop() {
  float temperatureC = readTemperatureC();
  float pHValue      = readPH();
  float turbidityNTU = readTurbidityNTU();
  float tdsValue     = readTDS(temperatureC);

  float wqi = computeWQI(pHValue, tdsValue, turbidityNTU, temperatureC);
  const char* category = wqiCategory(wqi);

  /* ---- SERIAL OUTPUT ---- */
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

  /* ---- OLED OUTPUT ---- */
  char line1[17], line2[17], line3[17], line4[17], line5[17], line6[17];

  char tbuf[10];
  char phbuf[10];

  dtostrf(temperatureC, 4, 1, tbuf);
  dtostrf(pHValue,      4, 2, phbuf);

  snprintf(line1, sizeof(line1), "Temp:%s C", tbuf);
  snprintf(line2, sizeof(line2), "TDS :%4ldppm", (long)(tdsValue + 0.5));
  snprintf(line3, sizeof(line3), "NTU :%6ld",   (long)(turbidityNTU + 0.5));
  snprintf(line4, sizeof(line4), "pH  :%s", phbuf);
  snprintf(line5, sizeof(line5), "WQI :%6ld",   (long)(wqi + 0.5));
  snprintf(line6, sizeof(line6), "CAT :%s", category);

  oled.clearLine(0); oled.drawString(0, 0, "Water Monitor");
  oled.clearLine(2); oled.drawString(0, 2, line1);
  oled.clearLine(3); oled.drawString(0, 3, line2);
  oled.clearLine(4); oled.drawString(0, 4, line3);
  oled.clearLine(5); oled.drawString(0, 5, line4);
  oled.clearLine(6); oled.drawString(0, 6, line5);
  oled.clearLine(7); oled.drawString(0, 7, line6);

  delay(1000);
}