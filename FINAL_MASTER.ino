/* Ligações I2C do BME/RTC 
GND - GND 
VCC - 3.3/5V 
SDA - SDA/A4 
SCL - SCL/A5 

Ligações I2C UNO - MEGA 
SDA/A4 UNO - SDA/D20 MEGA 
SCL/A5 UNO - SCL/D21 MEGA 
GND-GND 
*/

/* --------------------------- CONFIG --------------------------- */
#define TINY_GSM_MODEM_SIM800

#include <Wire.h>
#include <math.h>
#include <TinyGsmClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Endereços dos sensores
#define BME1_ADDR 0x76
#define BME2_ADDR 0x77

#define SerialAT Serial
#define MODEM_BAUD 9600

TinyGsm modem(SerialAT);
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;

/* --------------------------- BME --------------------------- */
float t1 = 0;
float h1 = 0;
float p1 = 0;

float t2 = 0;
float h2 = 0;
float p2 = 0;

float temperature = 0;
float humidity = 0;
float pressure = 0;

float Psat = 0;
float Habs = 0;

/* --------------------------- CÁLCULOS HUMIDADE DO SOLO --------------------------- */
const float Hveg = 0.5;
const float intensity = 147.13;
const float ref_intensity = 156.17;

float counts = 0;
uint32_t lastRawCounts = 0;

float hCorr = 0;
float pCorr = 0;
float intCorr = 0;
float bioCorr = 0;
float tau = 0;
float atmDepth = 0;
float fullCorr = 0;
float lastFullCorr = 0;

const float N0 = 261.9523;
const float N0_a0 = 0.0808;
const float N0_a1 = 0.372;
const float N0_a2 = 0.115;

const float newN0 = 281.496;
const float newN0_a0 = -0.115;
const float newN0_a1 = 0.3462;

float newCAS_N0 = 0;

float CAS_N0 = 0;

const float UTS_p0 = 1.0940;
const float UTS_p1 = 0.0280;
const float UTS_p2 = 0.254;
const float UTS_p3 = 3.537;
const float UTS_p4 = 0.139;
const float UTS_p5 = -0.00140;
const float UTS_p6 = -0.0088;
const float UTS_p7 = 0.0001150;

const float UTS_Nd = 371.37;

float CAS_UTS = 0;
float CAS = 0;

uint32_t totalCountsDet1 = 0;
uint32_t totalCountsDet2 = 0;
float theta0 = 0.30;
/* --------------------------- COMUNICAÇÃO MASTER-SLAVE --------------------------- */
#define BUFFER_SIZE 90

uint16_t neutronBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

struct SlaveData {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  uint32_t contagensDet1;
  uint32_t contagensDet2;
  float vDet1;
  float vDet2;
};

SlaveData data;

/* --------------------------MISC------------------------- */
float Vbat = 0;
const int VbatPin = A15;

//const char simPIN[] = "1234";
const char phoneNumber[] = "+351967452328"; // Destinatário

const int PowerPin = 22;

const uint16_t MAX_VALID_COUNTS = 50;
uint32_t rejectedSpikes = 0;
static uint16_t lastValidCounts = 0;

/* --------------------------TIMERS------------------------- */
unsigned long startTime;
unsigned long lastI2C = 0;
unsigned long lastSMS = 0;
unsigned long lastCAS = 0;

const unsigned long I2C_interval = 60000;
const unsigned long SMS_interval = 3600000;
const unsigned long CAS_interval = 3600000;

const unsigned long first_SMS_delay = 300000;
const unsigned long first_CAS_delay = 7200000;

float Ftheta(float theta, float H, float I) {
  return UTS_Nd * ((UTS_p1 + UTS_p2 * theta) / (UTS_p1 + theta) * (UTS_p0 + UTS_p6 * H + UTS_p7 * H * H) + exp(-UTS_p3 * theta) * (UTS_p4 + UTS_p5 * H)) - I;
}

float dFtheta(float theta, float H) {
  float A = UTS_p2 * (UTS_p1 + theta) - (UTS_p1 + UTS_p2 * theta);
  float B = (UTS_p1 + theta) * (UTS_p1 + theta);
  float C = UTS_p0 + UTS_p6 * H + UTS_p7 * H * H;
  float D = UTS_p3 * exp(-UTS_p3 * theta) * (UTS_p4 + UTS_p5 * H);

  return UTS_Nd * (A / B * C - D);
}

float NewtonTheta(float theta0, float H, float I) {
  float theta = theta0;

  for (int i = 0; i < 8; i++) {
    float F = Ftheta(theta, H, I);
    float dF = dFtheta(theta, H);

    if (fabs(dF) < 1e-6) break;

    float newTheta = theta - F / dF;

    if (fabs(newTheta - theta) < 1e-6) return newTheta;

    theta = newTheta;
  }

  return theta;
}



float getRollingMean() {
  int n = bufferFilled ? BUFFER_SIZE : bufferIndex;
  if (n == 0) {
    return lastValidCounts;
  }

  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += neutronBuffer[i];
  }
  return sum / n;
}

/* -------------------------- SETUP ------------------------- */
void setup() {
  Serial.begin(9600);
  SerialAT.begin(MODEM_BAUD);

  delay(5000);

  modem.restart();
  //modem.simUnlock(simPIN);

  pinMode(PowerPin, OUTPUT);
  digitalWrite(PowerPin, LOW);
  delay(2000);
  digitalWrite(PowerPin, HIGH);

  Wire.begin();

  bme1.begin(BME1_ADDR);
  bme2.begin(BME2_ADDR);

  pinMode(VbatPin, INPUT);

  startTime = millis();
}

/* -------------------------- LOOP ------------------------- */
void loop() {

  if (millis() - lastI2C >= I2C_interval) {
    lastI2C = millis();

    Wire.requestFrom(8, sizeof(data));

    if (Wire.available() == sizeof(data)) {
      Wire.readBytes((uint8_t*)&data, sizeof(data));

      totalCountsDet1 += data.contagensDet1;
      totalCountsDet2 += data.contagensDet2;

      uint16_t one_min_counts = (data.contagensDet1 + data.contagensDet2) / 2;

      float mean = getRollingMean();
      if (mean < 5) mean = 10;
      float limit = mean * 3.0;

      if (lastValidCounts == 0) {
        lastValidCounts = one_min_counts;
      }
      if (one_min_counts <= limit) {
        lastValidCounts = one_min_counts;
        
      } 
      else {
          one_min_counts = lastValidCounts;
          rejectedSpikes++;
        }
      neutronBuffer[bufferIndex++] = one_min_counts;

      if (bufferIndex >= BUFFER_SIZE) {
        bufferIndex = 0;
        bufferFilled = true;
      }
    }
  }

  if (millis() - startTime >= first_CAS_delay &&
      millis() - lastCAS >= CAS_interval &&
      bufferFilled) {

    lastCAS = millis();

    uint32_t hourCounts = 0;

    for (int i = 30; i < 90; i++) {
      int idx = (bufferIndex + i) % BUFFER_SIZE;
      hourCounts += neutronBuffer[idx];
    }

    t1 = bme1.readTemperature();
    h1 = bme1.readHumidity();
    p1 = bme1.readPressure() / 100.0;

    t2 = bme2.readTemperature();
    h2 = bme2.readHumidity();
    p2 = bme2.readPressure() / 100.0;

    temperature = (t1 + t2) / 2.0;
    humidity = (h1 + h2) / 2.0;
    pressure = (p1 + p2) / 2.0;

    Psat = 6.122 * exp((17.67 * temperature) / (temperature + 243.5));
    Habs = (Psat * humidity * 2.1674) / (273.15 + temperature);

    float atmDepth = pressure * (10.0 / 9.81);
    float hCorr = 1 + 0.0054 * (Habs - 12);
    float pCorr = exp((atmDepth - 1000) / 132);

    float tau = 1.36 * (-0.0009 * atmDepth + 1.7699) *
                (1 - exp(-(0.0064 * atmDepth + 1.8855) *
                pow(6.9, (0.000013 * atmDepth - 1.2237))));

    float intCorr = pow((intensity / ref_intensity) * tau + 1 - tau, -1);
    float bioCorr = 1 / (1 - 0.009 * Hveg * 4.4);

    float fullCorr = hCorr * pCorr * intCorr * bioCorr;

    counts = hourCounts * fullCorr;

    CAS_N0 = (N0_a0 / ((counts / N0) - N0_a1)) - N0_a2;
    habs_N0 = (newN0_a0 * counts/newN0) * pow(newN0_a1 - (counts/newN0), -1);
    CAS = NewtonTheta(theta0, Habs, counts);

    lastRawCounts = hourCounts;
    lastFullCorr = fullCorr;
  }

if ( (millis() - startTime >= first_SMS_delay && lastSMS == 0) || (millis() - lastSMS >= SMS_interval && lastSMS != 0)) {

    lastSMS = millis();

    t1 = bme1.readTemperature();
    h1 = bme1.readHumidity();
    p1 = bme1.readPressure() / 100.0;

    t2 = bme2.readTemperature();
    h2 = bme2.readHumidity();
    p2 = bme2.readPressure() / 100.0;

    temperature = (t1 + t2) / 2.0;
    humidity = (h1 + h2) / 2.0;
    pressure = (p1 + p2) / 2.0;

    Vbat = analogRead(VbatPin);
    Vbat = Vbat * (5.0 / 1023.0) * 3.136;

    char bufV1[8], bufV2[8], bufT[8], bufH[8], bufP[8];
    char bufCAS_UTS[8], bufCAS_N0[8], bufhabs_N0[8], bufCount[8], bufVbat[8];
    char bufCorr[10];

    dtostrf(data.vDet1, 5, 1, bufV1);
    dtostrf(data.vDet2, 5, 1, bufV2);
    dtostrf(temperature, 5, 1, bufT);
    dtostrf(humidity, 5, 1, bufH);
    dtostrf(pressure, 6, 1, bufP);
    dtostrf(CAS * 100.0, 6, 2, bufCAS_UTS);
    dtostrf(CAS_N0 * 100.0, 6, 2, bufCAS_N0);
    dtostrf(newCAS_N0 * 100.0, 6, 2, bufnewCAS_N0);
    dtostrf(counts, 6, 1, bufCount);
    dtostrf(lastFullCorr, 6, 3, bufCorr);
    dtostrf(Vbat, 6, 2, bufVbat);

    char sms[320];

    snprintf(sms, sizeof(sms),
      "%02u/%02u/%04u\n"
      "%02u:%02u:%02u\n"
      "Cnt1: %lu\n"
      "Cnt2: %lu\n"
      "TotCnt1: %lu\n"
      "TotCnt2: %lu\n"
      "V1: %s V\n"
      "V2: %s V\n"
      "T: %s C\n"
      "H: %s %%\n"
      "P: %s hPa\n"
      "Counts: %s\n"
      "RawCnt: %lu\n"
      "Spikes: %lu\n"
      "CorrF: %s\n"
      "CAS_N0: %s %%\n"
      "newCAS_N0: %s %%\n"
      "CAS_UTS: %s %%\n"
      "VBat: %s V\n",

      data.day, data.month, data.year,
      data.hour, data.minute, data.second,
      data.contagensDet1, data.contagensDet2,
      totalCountsDet1, totalCountsDet2,
      bufV1, bufV2, bufT, bufH, bufP,
      bufCount, lastRawCounts, rejectedSpikes,
      bufCorr, bufCAS_N0, bufnewCAS_N0, bufCAS_UTS, bufVbat
    );

    modem.sendSMS(phoneNumber, sms);
  }
}