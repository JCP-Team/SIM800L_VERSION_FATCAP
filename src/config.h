#include <Arduino.h>


#define TINY_GSM_MODEM_SIM868
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon

#define PUBLISH_TOPIC "0001/data"
#define SUBSCRIBE_TOPIC "0001/data"
#define SUBSCRIBE_RESPOND_TOPIC "0001/data"
#define APN "afrihost" // "Vodacom"
#define BROKER "test.mosquitto.org"
#define PORT 1883
#define TOKEN "secret"
#define APP_VERSION "ccfccnadblapdp7slkg0-v1"
#define RELAY_PIN 10 //gp10: pin 14
#define BATTERY_PIN 28 //gp28: pin 34
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
