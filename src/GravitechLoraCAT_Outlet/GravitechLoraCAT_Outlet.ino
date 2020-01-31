/*
  DS3231: Real-Time Clock. Simple example
  Read more: www.jarzebski.pl/arduino/komponenty/zegar-czasu-rzeczywistego-rtc-ds3231.html
  GIT: https://github.com/jarzebski/Arduino-DS3231
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/
//https://github.com/adafruit/Adafruit_Sensor

#include <MemoryFree.h> //https://github.com/mpflaga/Arduino-MemoryFree
#include "Arduino.h"
#include "wiring_private.h"
#include <SPI.h>
#include <Wire.h>
#include <HTS221.h> //https://github.com/ameltech/sme-hts221-library
#include <DS3231.h> //https://github.com/jarzebski/Arduino-DS3231/blob/master/DS3231.h

#include <Adafruit_BMP085.h>

//---------OLED Start---------
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//---------OLED End-----------

#include "TimeTicker.h"
#include "CATLoRa.h"
#include "Utility.h"
#include "DisplaySSD1306.h"
#include "DynamicSensorPayload.h"
#include "MQ2Sensor.h"
#include "PIRSensor.h"
#include "RelayDevice.h"
#include "LedDevice.h"
#include "GSensorForFloat.h"
#include "CommandController.h"
#include <Adafruit_TSL2561_U.h>

#define LORA_TX           10
#define LORA_RX           11
#define PIN_RELAY_1       5
#define PIN_RELAY_2       6
#define PIN_RELAY_3       9
#define PIN_PIR           12
//#define PIN_LED           13
#define PIN_GAS           A0

#define CHANNEL_TEMPERATURE 1
#define CHANNEL_HUMIDITY    2
#define CHANNEL_PIR         3
#define CHANNEL_GAS         4
#define CHANNEL_RELAY_1    10
#define CHANNEL_RELAY_2    11
#define CHANNEL_RELAY_3    12

#define SETUP_TIME_INTERVAL    20000
#define READY_TIME_INTERVAL    300000

//---------OLED Define Start---------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DisplaySSD1306 oled(&display);
//---------OLED Define End-----------

Uart Serial2(&sercom1, LORA_RX, LORA_TX, SERCOM_RX_PAD_0, UART_TX_PAD_2);

DS3231 clock;
char datetimeBuffer[32];

Adafruit_BMP085 bmp;


RTCDateTime dt;

TimeTicker doSensorTicker;

TimeTicker doLoRaSenderTicker;
TimeTicker doLoRaReceiverTicker;
TimeTicker doLoRaRefreshDeviceConfigTicker;

TimeTicker showFreeMemoryTicker;

CATLoRa catlora(&Serial2);

MQ2Sensor gas("GAS");
PIRSensor pir("PIR");
RelayDevice relay1("Relay 1");
RelayDevice relay2("Relay 2");
RelayDevice relay3("Relay 3");
LedDevice led("LED");
GSensorForFloat tHTS221("Temperature");
GSensorForFloat  hHTS221("Humidity");

CommandController commandCtrl(&clock, &tHTS221, &hHTS221, &gas, &pir, &relay1, &relay2, &relay3);

//state
bool setupState = false;
uint8_t portResponse = 0;
String payloadResponse = "";

//Sensor Data
float humValue = 0.0;
float tempValue = 0.0;
uint16_t gasValue = 0;
bool pirState;
bool relay1State;
bool relay2State;
bool relay3State;

uint8_t secs;
uint32_t transCount = 0;

uint32_t rxCount = 0;
uint32_t txCount = 0;

uint32_t refreshDeviceConfigurationInterval = SETUP_TIME_INTERVAL;
uint32_t readSensorInterval = 5000;
uint32_t txInterval = 22000;
uint32_t rxInterval = 500;

void setup() {
  Serial.begin(115200);
  //    while (!Serial) {};
  delay(5000);
  
  Serial.println(F("------- Setup [START] -------"));
  Serial.println();
  //-----------Initialize DS3231 Start-----------
  Serial.println(F("Initialize DS3231"));
  clock.begin();
  //SET TIME WHEN CONNECT ONLY ARDUINO IDE SERIAL MONITOR
  if (Serial) {
    clock.setDateTime(__DATE__, __TIME__);
    Serial.println(__DATE__);
    Serial.println(__TIME__);
  }
  //-----------Initialize DS3231 End-----------

  oled.begin();

  led.begin(PIN_LED);
  relay1.begin(PIN_RELAY_1, GDEVICE_ACTIVE_HIGH);
  relay2.begin(PIN_RELAY_2, GDEVICE_ACTIVE_HIGH);
  relay3.begin(PIN_RELAY_3, GDEVICE_ACTIVE_HIGH);
  gas.begin(PIN_GAS);
  pir.begin(PIN_PIR);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  //  CAT LORA Connecttion
  catlora.begin(115200, &oled);
  delay(3000);
  catlora.transmitter(11, "09");
  delay(1000);

  //Temperature & Humidity Sensor
  smeHumidity.begin();
}

void loop() {
  //Time
  dt = clock.getDateTime();
  if (secs != dt.second) {
    if (!setupState) {
      sprintf(datetimeBuffer, "%04d-%02d-%02d %02d:%02d:%02d", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
      oled.showMainUI(datetimeBuffer, tempValue, humValue, gasValue, pirState, txCount, rxCount);
    }
    secs = dt.second;

    tHTS221.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);
    hHTS221.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);
    
    pir.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);
    gas.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);

    relay1.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);
    relay2.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);
    relay3.doTasks(dt.dayOfWeek, dt.hour, dt.minute, dt.second);

    if (rxCount > 0 && refreshDeviceConfigurationInterval == SETUP_TIME_INTERVAL) {
      refreshDeviceConfigurationInterval = READY_TIME_INTERVAL;
    }

  }

  if (refreshDeviceConfigurationInterval == SETUP_TIME_INTERVAL) {
    led.blink(300);
  } else {
    led.on();
  }

  // Read Sensor
  doSensorTicker.attach(readSensorInterval, &doSensor);

  // LoRa Sender
  doLoRaSenderTicker.attach(txInterval, &doLoRaSender);

  // LoRa Receiver
  doLoRaReceiverTicker.attach(rxInterval, &doLoRaReceiver);

  // LoRa get new Device Config
  doLoRaRefreshDeviceConfigTicker.attach(refreshDeviceConfigurationInterval, &doLoRaRefreshDeviceConfiguration);

  // Show Free Memory
  showFreeMemoryTicker.attach(30000, &showFreeMemory);
}

void SERCOM1_Handler(void) {
  Serial2.IrqHandler();
}

void showFreeMemory() {
  Serial.print(F("############ FREE RAM => ")); //F function does the same and is now a built in library, in IDE > 1.0.0
  Serial.print(freeMemory(), DEC);  // print how much RAM is available..
  Serial.println(F(" ############"));
}

void doLoRaRefreshDeviceConfiguration() {
  portResponse = 11;
  payloadResponse = "09";
}

void doSensor() {
  hHTS221.setSensorValue(smeHumidity.readHumidity());
  tHTS221.setSensorValue(bmp.readTemperature());
  humValue = hHTS221.getSensorValue();
  tempValue = tHTS221.getSensorValue();
  
  gasValue = gas.read();
  pirState = pir.read();
  
  relay1State = relay1.isTurnOn();
  relay2State = relay2.isTurnOn();
  relay3State = relay3.isTurnOn();
}

void doLoRaSender() {
  Serial.println(F("############ doLoRaSender #############"));

  if (++txCount > 9999) {
    txCount = 1;
  }

  if (payloadResponse != "") {
    Serial.print(F("Send data in payloadResponse : "));
    Serial.print(portResponse);
    Serial.print(F(" "));
    Serial.println(payloadResponse);
    catlora.transmitter( portResponse, payloadResponse);
    portResponse = 0;
    payloadResponse = "";
    return;
  }

  // SEND UPLINK DATA
  DynamicSensorPayload *dsp = new DynamicSensorPayload();
  dsp->setTemperature(tempValue);
  dsp->setHumidity(humValue);
  dsp->setGas(gasValue);
  dsp->setPirState(pirState);
  dsp->setRelay1State(relay1State);
  dsp->setRelay2State(relay2State);
  dsp->setRelay3State(relay3State);

  int port = 1;
  String payload = dsp->dynamicSensorPayload();
  catlora.transmitter(port, payload);

  Serial.print(F("Send Dynamic Sensor Payload : "));
  Serial.print(port);
  Serial.print(F(" "));
  Serial.println(payload);

  free(dsp);
}

void doLoRaReceiver() {
  String strTemp = catlora.receiver();
  if (strTemp != "" && catlora.portReceived != 0) {
    commandCtrl.doAction(catlora.portReceived, catlora.payloadReceived);

    if (++rxCount > 9999) {
      rxCount = 1;
    }
  }
}
