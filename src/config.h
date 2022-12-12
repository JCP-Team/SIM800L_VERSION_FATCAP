#include <Arduino.h>
#include <SoftwareSerial.h>

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
// SoftwareSerial SerialAT = SoftwareSerial(5,4);
#define SerialAT Serial1 //??
#define TINY_GSM_DEBUG SerialMon


#define SUBSCRIBE_TOPIC "GDRT-2022"
#define APN "Vodacom"
#define BROKER "test.mosquitto.org" 
#define PORT 1883

#define RELAY_PIN 2 //PIN 4: GPIO2
#define BATTERY_PIN 26 //PIN 31
#define READING_INTERVAL 30000
#define WARMUP_INTERVAL 2*1000*60


float_t err = 0;
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

uint16_t s_error;
char errorMessage[256];
