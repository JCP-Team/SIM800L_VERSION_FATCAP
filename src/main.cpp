#include "config.h"

#include <hardware/pio.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "SCD30.h"
#include "ArduinoJson.h"
#include <Seeed_HM330X.h>
#include <SensirionI2CSen5x.h>


TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

SensirionI2CSen5x sen55;
void external_state(bool in){ //switches relay ON/OFF
    if(in) digitalWrite(RELAY_PIN, HIGH);
    else digitalWrite(RELAY_PIN,LOW);
}

void sensor_setup(){
    Wire1.setSCL(I2C_SCL);
    Wire1.setSDA(I2C_SDA);
    Wire1.begin();
   
    sen55.begin(Wire1);
    s_error = sen55.deviceReset();
    if (s_error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(s_error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    s_error = sen55.startMeasurement();
    if (s_error) {
        Serial.print("Error executing startMeasurement(): ");
        errorToString(s_error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    Serial.println("init success");
}

String senor_json_data(){
    sensor_setup();
    scd30.initialize();
    scd30.setAutoSelfCalibration(1);

    StaticJsonDocument<1024> doc;
    //scd30 data
    float result[3] = {0};
    if (scd30.isAvailable()) {
        scd30.getCarbonDioxideConcentration(result);   
        doc["a"] = result[0];
        doc["b"] = result[1];
        doc["c"] = result[2];   
    }
    //scd30 data end
    //SEN55 data start
    s_error = sen55.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (s_error) {
        Serial.print("s_error trying to execute readMeasuredValues(): ");
        errorToString(s_error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        doc["d"]= round(massConcentrationPm1p0*100)/100;
        doc["e"]=round(massConcentrationPm2p5*100)/100;
        doc["f"]=round(massConcentrationPm4p0*100)/100;
        doc["g"]=round(massConcentrationPm10p0*100)/100;
        if (isnan(ambientHumidity)) {
            doc["h"]=0;
        } else {
            doc["h"]=round(ambientHumidity*100)/100;
        }
        if (isnan(ambientTemperature)) {
            doc["i"]=0;
        } else {
            doc["i"]=round(ambientTemperature*100)/100;
        }
        if (isnan(vocIndex)) {
            doc["j"]=0;
        } else {
            doc["j"]=vocIndex;
        }
        if (isnan(noxIndex)) {
            doc["k"]=0;
        } else {
            doc["k"]=noxIndex;
        }
    }
    //SEN55 data end
    String data;
    serializeJson(doc,data);
    Wire1.end();
    return data;
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for(int i = 0; i < len; i++) {
    char c = (char)payload[i];
    msg += c;
  }

  Serial.print("Recived ");
  Serial.println(msg);
}

boolean network_connect(){
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Connecting to network");
    if (!modem.waitForNetwork(180000L, true)) {
      return false;
    }
  }
  if (!modem.isGprsConnected()) {
    SerialMon.println("Connecting to "+ String(APN));
    if (!modem.gprsConnect(APN,"","")) {
      return false;
    }
  }
  if(!mqtt.connected()){
    SerialMon.println("Connecting to "+ String(BROKER));
    if (!mqtt.connect(TOKEN)) {
      return false;
    }
    // mqtt.subscribe(SUBSCRIBE_TOPIC);
    Serial.println("CONNECTED");
  }
  return true;
}

void setup() {
  SerialMon.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  delay(4000);
  Serial2.setTX(4);
  Serial2.setRX(5);
  TinyGsmAutoBaud(SerialAT, 9600, 115200);
  delay(6000);
  SerialMon.println("Initializing modem...");
  modem.init();
  mqtt.setServer(BROKER, PORT);
  mqtt.setCallback(mqttCallback);
}

enum MAINSTATE{WARMUP,SEND};
MAINSTATE main_state = MAINSTATE::WARMUP;
unsigned long int timerr = 0;
void loop() {
  if(!network_connect()) return;
  mqtt.loop();
  // if(timerr < millis()){
  //   timerr = millis() + 5000;
  //   mqtt.publish(PUBLISH_TOPIC,"{\"temperature\":10}");
  //   Serial.println("published");
  // }
  switch (main_state)
  {
    case MAINSTATE::WARMUP:{
      if(timerr < millis()){
        main_state = MAINSTATE::SEND;
       external_state(1);
        Serial.println("Warming up,relay is on");
        timerr = millis() + WARMUP_INTERVAL;
      }
      break;
    }
    case MAINSTATE::SEND:{
      if(timerr < millis()){
        String data = "["+senor_json_data()+"]";
        mqtt.publish(PUBLISH_TOPIC,data.c_str());
        Serial.println(data);
        main_state= MAINSTATE::WARMUP;
        Serial.println("Published,relay is off");
        external_state(0);
        timerr = millis() + READING_INTERVAL;
      }
      break;
    }
  }
} 
