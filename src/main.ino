/*
  Water Monitor (U8x8 OLED) + WQI + Category
  FIX: UNO snprintf does not support %f floats -> use dtostrf for temp & pH
  Board: Arduino UNO
  OLED: SSD1306 128x64 I2C (0x3C)
  DS18B20: D4
*/

#include <Wire.h>
#include <U8x8lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

/* ================= OLED ================= */
U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE); // 0x3C default

/* ================= PINS ================= */
#define TDS_PIN        A0
#define TURBIDITY_PIN  A1
#define PH_PIN         A2
#define ONE_WIRE_BUS   4

/* ================= CONSTANTS ================= */
#define VREF        5.0
#define SCOUNT      30
#define PH_SAMPLES  50

/* ================= GLOBALS ================= */
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;

float temperatureC = 25.0;
float tdsValue = 0;

/* ================= DS18B20 ================= */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/* ================= MEDIAN FILTER ================= */
int median30(const int *arr) {
  int bTab[SCOUNT];
  for (int i = 0; i < SCOUNT; i++) bTab[i] = arr[i];

  for (int j = 0; j < SCOUNT - 1; j++) {
    for (int i = 0; i < SCOUNT - 1 - j; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int t = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = t;
      }
    }
  }
  return bTab[SCOUNT / 2];
}

/* ================= pH HELPERS ================= */
// NOT real calibration – only for trimmer adjustment
float phFromVoltageFake(float v) {
  return 7.0 + (2.50 - v) / 0.18;
}

float readPHVoltageAvg() {
  long sum = 0;
  for (int i = 0; i < PH_SAMPLES; i++) {
    sum += analogRead(PH_PIN);
    delay(5);
  }
  float adc = (float)sum / PH_SAMPLES;
  return (adc * VREF) / 1023.0;
}

/* ================= WQI SETTINGS ================= */
const float STD_TDS_PPM   = 500.0;  // ppm
const float STD_TURB_NTU  = 5.0;    // NTU
const float PH_LOW_LIMIT  = 6.5;
const float PH_HIGH_LIMIT = 8.5;

const float TEMP_IDEAL_C = 25.0;
const float TEMP_MAX_C   = 35.0;

/* Weights (you justify in report). Will be normalized. */
float wPH   = 4.0;
float wTurb = 4.0;
float wTDS  = 3.0;
float wTemp = 1.0;

float q_tds(float tdsPpm)  { return (tdsPpm / STD_TDS_PPM) * 100.0; }
float q_turb(float ntu)    { return (ntu / STD_TURB_NTU) * 100.0; }

float q_temp(float tC) {
  float denom = (TEMP_MAX_C - TEMP_IDEAL_C);
  if (denom <= 0.0) return 0.0;
  return (fabs(tC - TEMP_IDEAL_C) / denom) * 100.0;
}

float q_ph(float ph) {
  float denom;
  if (ph >= 7.0) denom = (PH_HIGH_LIMIT - 7.0);
  else          denom = (7.0 - PH_LOW_LIMIT);
  if (denom <= 0.0) return 0.0;
  return (fabs(ph - 7.0) / denom) * 100.0;
}

float computeWQI(float ph, float tdsPpm, float ntu, float tempC) {
  float sumW = wPH + wTurb + wTDS + wTemp;
  if (sumW <= 0.0) return 0.0;

  float Wph   = wPH   / sumW;
  float Wturb = wTurb / sumW;
  float Wtds  = wTDS  / sumW;
  float Wtemp = wTemp / sumW;

  return (Wph * q_ph(ph)) + (Wturb * q_turb(ntu)) + (Wtds * q_tds(tdsPpm)) + (Wtemp * q_temp(tempC));
}

const char* wqiCategory(float wqi) {
  if (wqi <= 25)   return "PURE";
  if (wqi <= 50)   return "GOOD";
  if (wqi <= 100)  return "NORMAL";
  if (wqi <= 200)  return "POOR";
  return "CONTAM.";
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  sensors.begin();

  pinMode(TDS_PIN, INPUT);
  pinMode(TURBIDITY_PIN, INPUT);
  pinMode(PH_PIN, INPUT);

  for (int i = 0; i < SCOUNT; i++) analogBuffer[i] = 0;

  oled.begin();
  oled.setPowerSave(0);

  // Your new font (works well)
  oled.setFont(u8x8_font_victoriabold8_r);

  oled.clear();
  oled.drawString(0, 0, "Water Monitor");
  oled.drawString(0, 1, "Init OK");
  delay(1000);
}

/* ================= LOOP ================= */
void loop() {
  /* ---- TDS sampling ---- */
  static unsigned long sampleTime = 0;
  if (millis() - sampleTime > 40) {
    sampleTime = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex++;
    if (analogBufferIndex >= SCOUNT) analogBufferIndex = 0;
  }

  /* ---- 1 second update ---- */
  static unsigned long updateTime = 0;
  if (millis() - updateTime > 1000) {
    updateTime = millis();

    /* ---- Temperature ---- */
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t > -55 && t < 125) temperatureC = t;

    /* ---- TDS ---- */
    int med = median30(analogBuffer);
    float avgVoltage = med * (VREF / 1023.0);
    float compCoeff = 1.0 + 0.02 * (temperatureC - 25.0);
    float compVoltage = avgVoltage / compCoeff;

    tdsValue =
      (133.42 * compVoltage * compVoltage * compVoltage
      - 255.86 * compVoltage * compVoltage
      + 857.39 * compVoltage) * 0.5;
    if (tdsValue < 0) tdsValue = 0;

    /* ---- Turbidity ---- */
    int rawTur = analogRead(TURBIDITY_PIN);
    float turVoltage = rawTur * (VREF / 1023.0);
    float ntu = (-781.25 * turVoltage) + 3000.0;
    if (ntu < 0) ntu = 0;

    /* ---- pH ---- */
    float phVoltage = readPHVoltageAvg();
    float pH = phFromVoltageFake(phVoltage);

    /* ---- WQI + Category ---- */
    float wqi = computeWQI(pH, tdsValue, ntu, temperatureC);
    const char* cat = wqiCategory(wqi);

    /* ---- SERIAL OUTPUT ---- */
    Serial.print(F("Temp=")); Serial.print(temperatureC, 1);
    Serial.print(F("C  TDS=")); Serial.print(tdsValue, 0);
    Serial.print(F("ppm  NTU=")); Serial.print(ntu, 0);
    Serial.print(F("  pH~=")); Serial.print(pH, 2);
    Serial.print(F("  WQI=")); Serial.print(wqi, 1);
    Serial.print(F("  CAT=")); Serial.println(cat);

    /* ---- OLED OUTPUT ---- */
    char line1[17], line2[17], line3[17], line4[17], line5[17], line6[17];

    // dtostrf(width, precision) -> safe float to string on UNO
    char tbuf[10];
    char phbuf[10];
    dtostrf(temperatureC, 4, 1, tbuf); // ex: "25.3"
    dtostrf(pH,          4, 2, phbuf); // ex: "7.12"

    // keep each line <= 16 chars
    snprintf(line1, sizeof(line1), "Temp:%s C", tbuf);
    snprintf(line2, sizeof(line2), "TDS :%4ldppm", (long)(tdsValue + 0.5));
    snprintf(line3, sizeof(line3), "NTU :%6ld", (long)(ntu + 0.5));
    snprintf(line4, sizeof(line4), "pH  :%s", phbuf);
    snprintf(line5, sizeof(line5), "WQI :%6ld", (long)(wqi + 0.5));
    snprintf(line6, sizeof(line6), "CAT :%s", cat);

    oled.clearLine(0); oled.drawString(0, 0, "Water Monitor");
    oled.clearLine(2); oled.drawString(0, 2, line1);
    oled.clearLine(3); oled.drawString(0, 3, line2);
    oled.clearLine(4); oled.drawString(0, 4, line3);
    oled.clearLine(5); oled.drawString(0, 5, line4);
    oled.clearLine(6); oled.drawString(0, 6, line5);
    oled.clearLine(7); oled.drawString(0, 7, line6);
  }
}
