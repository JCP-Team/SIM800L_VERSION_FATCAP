#include <Arduino.h>
#include <SoftwareSerial.h>

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
// SoftwareSerial SerialAT = SoftwareSerial(5,4);
#define SerialAT Serial2
#define TINY_GSM_DEBUG SerialMon


#define TOKEN "hYVuk5muwL"
#define APP_VERSION "cdj4beidblav5plmgukg-01"
#define PUBLISH_TOPIC "kp1/" APP_VERSION "/dcx/" TOKEN "/json"
#define SUBSCRIBE_TOPIC "kp1/" APP_VERSION "/cex/" TOKEN "/command/relay/status"
#define SUBSCRIBE_RESPOND_TOPIC "kp1/" APP_VERSION "/cex/" TOKEN "/result/relay"
#define APN "Vodacom"
#define BROKER "mqtt.cloud.kaaiot.com"
#define PORT 1883

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
