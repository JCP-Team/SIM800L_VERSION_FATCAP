#include "config.h"

#include <hardware/pio.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "SCD30.h"
#include "ArduinoJson.h"
#include <Seeed_HM330X.h>
#include <SensirionI2CSen5x.h>


TinyGsm       modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

int batt_mv(){ //Measures battery voltage through voltage divider
 return analogRead(BATTERY_PIN)*(3300/4095) *((1+2.7)/2.7); 
}

SensirionI2CSen5x sen55;
void external_state(bool in){ //switches relay ON/OFF
    if(in) digitalWrite(RELAY_PIN, HIGH);
    else digitalWrite(RELAY_PIN,LOW);
}

String senor_json_data(){ //Sensor reading and JSON parsing
    scd30.initialize();
    scd30.setAutoSelfCalibration(1);

    StaticJsonDocument<442> doc; //300+8+128 +8
    //batt
    doc["Battery_voltage"] = batt_mv();
    //scd30 data
    float result[3] = {0};
    if (scd30.isAvailable()) {
        scd30.getCarbonDioxideConcentration(result);   
        doc["CO2"] = ((float)((int)(result[0]*100)))/100;
        doc["Temp"] = ((float)((int)(result[1]*100)))/100;
        doc["Hum"] = ((float)((int)(result[2]*100)))/100;   
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
        doc["Pm1p0"]=((float)((int)(massConcentrationPm1p0*100)))/100;
        doc["Pm2p5"]=((float)((int)(massConcentrationPm2p5*100)))/100;
        doc["Pm4p0"]=((float)((int)(massConcentrationPm4p0*100)))/100;
        doc["Pm10p0"]=((float)((int)(massConcentrationPm10p0*100)))/100;
        if (isnan(ambientHumidity)) {
            doc["AH"]="n/a";
        } else {
            doc["AH"]=((float)((int)(ambientHumidity*100)))/100;;
        }
        if (isnan(ambientTemperature)) {
            doc["ATemp"]="n/a";
        } else {
            doc["ATemp"]=((float)((int)(ambientTemperature*100)))/100;;
        }
        if (isnan(vocIndex)) {
            doc["VocIndex"]="n/a";
        } else {
            doc["VocIndex"]=((float)((int)(vocIndex*100)))/100;;
        }
        if (isnan(noxIndex)) {
            doc["NoxIndex"]="n/a";
        } else {
            doc["NoxIndex"]=((float)((int)(noxIndex*100)))/100;;
        }
    }
    
    //SEN55 data end
    
    String data;
    serializeJson(doc,data);
    Wire.end();
    return data;
}

void sensor_setup(){ // Hardware initiation
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();
    sen55.begin(Wire);
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


void mqttCallback(char* topic, byte* payload, unsigned int len) { // Payload parsing of MQTT callback
  String msg;
  for(int i = 0; i < len; i++) {
    char c = (char)payload[i];
    msg += c;
  }

  Serial.print("Received ");
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
    if (!mqtt.connect("SADASKHKHDADASD5514615616")) {
      return false;
    }
    mqtt.subscribe(SUBSCRIBE_TOPIC);
    Serial.println("CONNECTED");
  }
  return true;
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(RST,OUTPUT);
  digitalWrite(RST,0);
  delay(1000);
  digitalWrite(RST,1);
  delay(1000);
  SerialMon.begin(115200);
  delay(4000);
  Serial2.setTX(TX); // Serial initialisation for GSM Modem
  Serial2.setRX(RX);
  // sensor_setup();
  TinyGsmAutoBaud(SerialAT, 9600, 115200);
  delay(6000);
  SerialMon.println("Initializing modem...");
  modem.init();
  mqtt.setServer(BROKER, PORT);
  mqtt.setCallback(mqttCallback);
}

enum MAINSTATE{WARMUP,SEND}; //Finite State Machine States
MAINSTATE main_state = MAINSTATE::WARMUP;
unsigned long int timerr = 0;

void loop() {
  if(!network_connect()) return;
  mqtt.loop();
  // Serial.println("State:"+String(main_state));
  // Serial.println("Millis:"+String(millis()));
  switch (main_state)
  {
    case MAINSTATE::WARMUP:{
      if(timerr < millis()){
        main_state = MAINSTATE::SEND;
        external_state(1); // Relay ON 
        delay(1000);
        sensor_setup();
        timerr = millis() + WARMUP_INTERVAL; // Sets timer for sensor "warm up" period
        Serial.println("warming up");
      }
      break;
    }
    case MAINSTATE::SEND:{
      if(timerr < millis()){
        Serial.println("sending data");
        String data = senor_json_data(); // Sensor readings and data serialization
        Serial.println(data);
        // Serial.println(senor_json_data());
        mqtt.publish(PUBLISH_TOPIC,data.c_str());
        mqtt.publish(PUBLISH_TOPIC,"Hello world");
        main_state= MAINSTATE::WARMUP;
        external_state(0);
        timerr = millis() + READING_INTERVAL; // Reading interval determines how long sensors are OFF for powersaving
        Serial.println("turning off sensors");
      }
      break;
    }
  }
} 
