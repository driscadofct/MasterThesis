/* 
Ligações I2C do BME/RTC
GND - GND
VCC - 3.3/5V
SDA - SDA/A4
SCL - SCL/A5

Ligações I2C UNO - MEGA

SDA/A4 UNO - SDA/D20 MEGA
SCL/A5 UNO - SCL/D21 MEGA
GND-GND


LIGAÇÕES PINOS

------Analógicos------
A0 - peak detector
A1 - Tensão entrada detetor 1
A2 - Tensão entrada detetor 2


-------Digitais--------
D2 + D3 - Saída TTL 1 + 2
D4 - Saída TTL detetor 1
D5 - Saída TTL detetor 2

*/

#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

/* --------------------------- CONFIG --------------------------- */

const int peakDetectorPin = A0;
const int vDetPin1 = A1;
const int vDetPin2 = A2;

const int det1idPin = 4;
const int det2idPin = 5;

const float DivScale1 = 1220.0 / 2.428;
const float DivScale2 = 1219.0 / 2.440;

const uint32_t deadtime_us = 50;

const unsigned long start_delay = 1800000; // 30 min
const unsigned long minute_interval = 60000;
/* --------------------------- VARIÁVEIS --------------------------- */

volatile uint16_t risingTime = 0;
volatile uint16_t fallingTime = 0;
volatile uint32_t pulseWidthTicks = 0;

volatile uint32_t overflowCount = 0;

volatile bool risingFlag = false;
volatile bool fallingFlag = false;

volatile bool pulseActive = false;

volatile bool countsBlocked = false;
volatile uint32_t blockStartMicros = 0;

volatile uint8_t currentDetector = 0;

int peakValue = 0;
float voltage = 0;

uint32_t minuteDet1 = 0;
uint32_t minuteDet2 = 0;

float vDet1 = 0;
float vDet2 = 0;

unsigned long startTime;
bool startCounting = false;

unsigned long lastMinute = 0;
unsigned long lastHVMeasure = 0;


/* --------------------------- COMUNICAÇÃO MASTER-SLAVE --------------------------- */


struct SlaveData {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  uint32_t contagensDet1;
  uint32_t contagensDet2;
  float vDet1;
  float vDet2;
};

volatile SlaveData snapshot;


/* --------------------------- SETUP --------------------------- */

void setup() {
  Serial.begin(9600);

  Wire.begin(8);
  Wire.onRequest(requestEvent);


  pinMode(2, INPUT);
  pinMode(3, INPUT);

  pinMode(det1idPin, INPUT);
  pinMode(det2idPin, INPUT);

  TCCR1A = 0;
  TCCR1B = (1 << CS10);
  TIMSK1 |= (1 << TOIE1);

  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);

  EICRA |= (1 << ISC11) | (1 << ISC10);

  EIMSK |= (1 << INT0) | (1 << INT1);

  rtc.begin();

  //rtc.adjust(DateTime(2026, 3, 17, 0, 38, 10));

  startTime = millis();
  lastMinute = millis();
}
/* --------------------------- ISR --------------------------- */

ISR(INT0_vect) {
  if (pulseActive || countsBlocked) return;

  pulseActive = true;
  risingTime = TCNT1;
  overflowCount = 0;
  risingFlag = true;
}

ISR(INT1_vect) {
  if (!pulseActive) return;

  fallingTime = TCNT1;

  pulseWidthTicks =
    overflowCount * 65536UL +
    (uint16_t)(fallingTime - risingTime);

  fallingFlag = true;
  pulseActive = false;

  countsBlocked = true;
  blockStartMicros = micros();
}

ISR(TIMER1_OVF_vect) {
  if (pulseActive) overflowCount++;
}

/* --------------------------- LOOP --------------------------- */

void loop() {
    if (!startCounting && millis() - startTime >= start_delay) {
    startCounting = true;
  }

  if (countsBlocked) {
    if (micros() - blockStartMicros >= deadtime_us) {
      countsBlocked = false;
    }
  }

  if (risingFlag) {
    risingFlag = false;

    delayMicroseconds(2);

    bool d1 = digitalRead(det1idPin);
    bool d2 = digitalRead(det2idPin);

    if (d1 && !d2) currentDetector = 1;
    else if (d2 && !d1) currentDetector = 2;
    else currentDetector = 3;

    peakValue = analogRead(peakDetectorPin);
    voltage = peakValue * (5.0 / 1023.0);

  }

  if (fallingFlag) {
    fallingFlag = false;

    float pulse_us = pulseWidthTicks * 0.0625;

    if (
    voltage >= 1.0 && voltage <= 4.5 &&
    (
        (currentDetector == 1 && pulse_us >= 10.9 && pulse_us <= 160.0) ||
        (currentDetector == 2 && pulse_us >= 15.0 && pulse_us <= 70.0)
    )
      )
        {
      if (startCounting) {  
        if (currentDetector == 1) minuteDet1++;
        else if (currentDetector == 2) minuteDet2++;
}
}


  }

  if (millis() - lastHVMeasure > 10000) {
    lastHVMeasure = millis();

    vDet1 = detectorVoltage(vDetPin1, DivScale1);
    vDet2 = detectorVoltage(vDetPin2, DivScale2);
  }

    
  if ((millis() - lastMinute) >= minute_interval) {
    lastMinute = millis();

    DateTime now = rtc.now();

    noInterrupts();
    snapshot.year = now.year();
    snapshot.month = now.month();
    snapshot.day = now.day();
    snapshot.hour = now.hour();
    snapshot.minute = now.minute();
    snapshot.second = now.second();
    snapshot.contagensDet1 = minuteDet1;
    snapshot.contagensDet2 = minuteDet2;
    snapshot.vDet1 = vDet1;
    snapshot.vDet2 = vDet2;
    interrupts();

    minuteDet1 = 0;
    minuteDet2 = 0;
  }
}

/* --------------------------- FUNÇÕES --------------------------- */

float detectorVoltage(int pin, float scale) {
  int adcValue = analogRead(pin);
  float vMeasure = adcValue * (5.0 / 1023.0);
  return vMeasure * scale;
}


void requestEvent() {
  Wire.write((uint8_t*)&snapshot, sizeof(snapshot));
}


