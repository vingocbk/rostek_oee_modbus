/*
  ModbusRTU ESP8266/ESP32
  Simple slave example

  (c)2019 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266

  modified 13 May 2020
  by brainelectronics

  This code is licensed under the BSD New License. See LICENSE.txt for more info.
*/
#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>
#include <Wire.h>
#include "DS3231.h"

#define REGN 10
#define PIN_RX_MODBUS 16
#define PIN_TX_MODBUS 17
#define EEPROM_ADD_SLAVE_ID 1
uint8_t slave_id = 1;
ModbusRTU modBus;

DS3231 ds3231_clock;
RTCDateTime dt;

struct SyncData
{
    bool isSyncDateTime = false;
    bool isSyncWorkingShift = false;
    bool isSyncData = false;
} syncData;



void loadDataBegin();
void initModbus();

void loadDataBegin(){
  slave_id = EEPROM.read(EEPROM_ADD_SLAVE_ID);
  if(slave_id == 0 || slave_id == 255){
    slave_id = 1;
  }
}
void initModbus(){
  modBus.slave(slave_id);
  modBus.addHreg(REGN);
  modBus.Hreg(REGN, 100);
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  Serial1.begin(9600, SERIAL_8N1, PIN_RX_MODBUS, PIN_TX_MODBUS);
  EEPROM.begin(512);
#if defined(ESP32) || defined(ESP8266)
  modBus.begin(&Serial1);
#else
  modBus.begin(&Serial1);
  //modBus.begin(&Serial1, RXTX_PIN);  //or use RX/TX direction control pin (if required)
  modBus.setBaudrate(9600);
#endif

  loadDataBegin();
  initModbus();
  ds3231_clock.begin();
  
}

void loop() {
  modBus.task();
  yield();
}