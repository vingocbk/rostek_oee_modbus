
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
#include "html.h"
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"
#include "soc/soc.h"  //Brownout detector was triggered
#include "soc/rtc_cntl_reg.h"

#define PIN_OUTPUT_RED        		18      //OUTPUT 2    //IDLE
#define PIN_OUTPUT_YELLOW     		19      //OUTPUT 3    //WAITING
#define PIN_OUTPUT_GREEN      		23      //OUTPUT 4    //EXECUTE
#define PIN_OUTPUT_4          		5       //OUTPUT 1

#define PIN_INPUT_COUNT      		36      //INPUT 1 - SENSOR_VP
#define PIN_INPUT_RUN       		39      //INPUT 2 - SENSOR_VN
#define PIN_INPUT_IDLE       		34      //INPUT 3
#define PIN_INPUT_ERROR       		35      //INPUT 4
#define PIN_INPUT_NG       			32      //INPUT 5
#define PIN_INPUT_RUNNING_NUMBER    33      //INPUT 6
#define PIN_INPUT_CONFIG      		25      //INPUT 7
// #define PIN_INPUT_START       		26      //INPUT 8

#define PIN_MODBUS_RX         		16
#define PIN_MODBUS_TX         		17

#define PIN_MODBUS_RX_2        		9
#define PIN_MODBUS_TX_2        		10

#define ON_FET      HIGH
#define OFF_FET     LOW

#define EEPROM_COUNT				0
#define EEPROM_NG					10
#define EEPROM_RUNNING_NUMBER		20
#define EEPROM_SLAVE_ID				30
#define EEPROM_MODBUS_BAUDRATE		40

enum{
	REG_CALL_ORDER = 1,
	REG_START,
	REG_CANCEL,
	REG_DONE,

	// REG_MODBUS_COUNT_H 			= 4496,
	// REG_MODBUS_COUNT_L 			= 4497,
	// REG_MODBUS_STATUS 			= 4500,
	// REG_MODBUS_NG_H 			= 4498,
	// REG_MODBUS_NG_L 			= 4499,
	// REG_MODBUS_RUNNING_NUMBER_H = 4501,
	// REG_MODBUS_RUNNING_NUMBER_L = 4502
	REG_MODBUS_COUNT_H 			= 0,
	REG_MODBUS_COUNT_L 			,
	REG_MODBUS_STATUS 			,
	REG_MODBUS_NG_H 			,
	REG_MODBUS_NG_L 			,
	REG_MODBUS_RUNNING_NUMBER_H ,
	REG_MODBUS_RUNNING_NUMBER_L ,
	REG_MODBUS_ADDING_START		= 10
};

typedef enum{
	MACHINE_STATUS_RUN = 0,
    MACHINE_STATUS_IDLE,
    // MACHINE_STATUS_WAITING,
    // MACHINE_STATUS_EXECUTE,
	MACHINE_STATUS_ERROR,
	MACHINE_STATUS_UNKNOW
} MACHINE_STATUS;

struct SetupConfig
{
    /* data */
	uint8_t slaveId;
	uint32_t modbusBaudrate;
	bool turnOnApMode = false;
    // uint32_t timeRelay1;
    // uint16_t timeRelay2;
    // uint16_t timeRelay3;
    // uint16_t timeRelay4;
    // MachineStatus machineStatus = IDLE;
    // MachineStatus relay1Status = IDLE;
    // MachineStatus relay2Status = IDLE;
    // MachineStatus relay3Status = IDLE;
    // MachineStatus relay4Status = IDLE;
    uint32_t timeCheckTurnOnConfig = 0;
    uint32_t timeCheckTurnOffConfig = 0;
	uint32_t timePrintData = 0;
}setupConfig;

struct DataRegister
{
    /* data */
    uint32_t count;
	uint32_t ng;
	uint32_t runningNumber;
	MACHINE_STATUS  eCurrentMachineStatus;
}dataRegister;


ModbusRTU modbus1, modbus2;
WebServer server(HTTP_PORT);

void checkInputCount(void *pvParameters);
void checkInputMCStatus(void *pvParameters);
void checkInputNG(void *pvParameters);
void checkInputRunningNumber(void *pvParameters);

void checkButtonConfigClick();
void printData();

void showLedStatus(MACHINE_STATUS status);
void loadDataBegin();
void initModbus();
String macID();
void setupConfigMode();
void handleRoot();
void configModbus();
void resetData();
void startWebServer();
void resetDataOee();


void checkInputCount(void *pvParameters){
	for(;;){
		if(digitalRead(PIN_INPUT_COUNT) == BTN_PRESSED){
			vTaskDelay(100/portTICK_PERIOD_MS); 
			while(digitalRead(PIN_INPUT_COUNT) == BTN_PRESSED){
				vTaskDelay(100/portTICK_PERIOD_MS); 
			}
			dataRegister.count++;
			modbus1.Hreg(REG_MODBUS_COUNT_H, dataRegister.count >> 16);
			modbus1.Hreg(REG_MODBUS_COUNT_L, dataRegister.count);
			modbus2.Hreg(REG_MODBUS_COUNT_H, dataRegister.count >> 16);
			modbus2.Hreg(REG_MODBUS_COUNT_L, dataRegister.count);
			EEPROM.write(EEPROM_COUNT, dataRegister.count >> 24);
			EEPROM.write(EEPROM_COUNT+1, dataRegister.count >> 16);
			EEPROM.write(EEPROM_COUNT+2, dataRegister.count >> 8);
			EEPROM.write(EEPROM_COUNT+3, dataRegister.count);
			EEPROM.commit();
		}
		vTaskDelay(100/portTICK_PERIOD_MS); 
	}
}

void checkInputMCStatus(void *pvParameters){
	for(;;){
		if(digitalRead(PIN_INPUT_RUN) == BTN_PRESSED && dataRegister.eCurrentMachineStatus != MACHINE_STATUS_RUN){
			dataRegister.eCurrentMachineStatus = MACHINE_STATUS_RUN;
			modbus1.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
			modbus2.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
			showLedStatus(dataRegister.eCurrentMachineStatus);
		}
		else if(digitalRead(PIN_INPUT_IDLE) == BTN_PRESSED && dataRegister.eCurrentMachineStatus != MACHINE_STATUS_IDLE){
			dataRegister.eCurrentMachineStatus = MACHINE_STATUS_IDLE;
			modbus1.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
			modbus2.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
			showLedStatus(dataRegister.eCurrentMachineStatus);
		}
		else if(digitalRead(PIN_INPUT_ERROR) == BTN_PRESSED && dataRegister.eCurrentMachineStatus != MACHINE_STATUS_ERROR){
			dataRegister.eCurrentMachineStatus = MACHINE_STATUS_ERROR;
			modbus1.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
			modbus2.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
			showLedStatus(dataRegister.eCurrentMachineStatus);
		}
		vTaskDelay(100/portTICK_PERIOD_MS); 
	}
}

void checkInputNG(void *pvParameters){
	for(;;){
		if(digitalRead(PIN_INPUT_NG) == BTN_PRESSED){
			vTaskDelay(100/portTICK_PERIOD_MS); 
			while(digitalRead(PIN_INPUT_NG) == BTN_PRESSED){
				vTaskDelay(100/portTICK_PERIOD_MS); 
			}
			dataRegister.ng++;
			modbus1.Hreg(REG_MODBUS_NG_H, dataRegister.ng >> 16);
			modbus1.Hreg(REG_MODBUS_NG_L, dataRegister.ng);
			modbus2.Hreg(REG_MODBUS_NG_H, dataRegister.ng >> 16);
			modbus2.Hreg(REG_MODBUS_NG_L, dataRegister.ng);
			EEPROM.write(EEPROM_NG, dataRegister.ng >> 24);
			EEPROM.write(EEPROM_NG+1, dataRegister.ng >> 16);
			EEPROM.write(EEPROM_NG+2, dataRegister.ng >> 8);
			EEPROM.write(EEPROM_NG+3, dataRegister.ng);
			EEPROM.commit();
		}
		vTaskDelay(100/portTICK_PERIOD_MS); 
	}
}

void checkInputRunningNumber(void *pvParameters){
	for(;;){
		if(digitalRead(PIN_INPUT_RUNNING_NUMBER) == BTN_PRESSED){
			vTaskDelay(100/portTICK_PERIOD_MS); 
			while(digitalRead(PIN_INPUT_RUNNING_NUMBER) == BTN_PRESSED){
				vTaskDelay(100/portTICK_PERIOD_MS); 
			}
			dataRegister.runningNumber++;
			modbus1.Hreg(REG_MODBUS_RUNNING_NUMBER_H, dataRegister.runningNumber >> 16);
			modbus1.Hreg(REG_MODBUS_RUNNING_NUMBER_L, dataRegister.runningNumber);
			modbus2.Hreg(REG_MODBUS_RUNNING_NUMBER_H, dataRegister.runningNumber >> 16);
			modbus2.Hreg(REG_MODBUS_RUNNING_NUMBER_L, dataRegister.runningNumber);
			EEPROM.write(EEPROM_RUNNING_NUMBER, dataRegister.runningNumber >> 24);
			EEPROM.write(EEPROM_RUNNING_NUMBER+1, dataRegister.runningNumber >> 16);
			EEPROM.write(EEPROM_RUNNING_NUMBER+2, dataRegister.runningNumber >> 8);
			EEPROM.write(EEPROM_RUNNING_NUMBER+3, dataRegister.runningNumber);
			EEPROM.commit();
		}
		vTaskDelay(100/portTICK_PERIOD_MS); 
	}
}

void showLedStatus(MACHINE_STATUS status){
	if(status == MACHINE_STATUS_IDLE){
		digitalWrite(PIN_OUTPUT_RED, ON_FET);
		digitalWrite(PIN_OUTPUT_YELLOW, OFF_FET);
		digitalWrite(PIN_OUTPUT_GREEN, OFF_FET);
	}
	else if(status == MACHINE_STATUS_ERROR){
		digitalWrite(PIN_OUTPUT_RED, OFF_FET);
		digitalWrite(PIN_OUTPUT_YELLOW, ON_FET);
		digitalWrite(PIN_OUTPUT_GREEN, OFF_FET);
	}
	else if(status == MACHINE_STATUS_RUN){
		digitalWrite(PIN_OUTPUT_RED, OFF_FET);
		digitalWrite(PIN_OUTPUT_YELLOW, OFF_FET);
		digitalWrite(PIN_OUTPUT_GREEN, ON_FET);
	}
}

void loadDataBegin(){
	dataRegister.eCurrentMachineStatus = MACHINE_STATUS_UNKNOW;
	showLedStatus(dataRegister.eCurrentMachineStatus);
	dataRegister.count = EEPROM.read(EEPROM_COUNT) << 24
						| EEPROM.read(EEPROM_COUNT+1) << 16
						| EEPROM.read(EEPROM_COUNT+2) << 8
						| EEPROM.read(EEPROM_COUNT+3);
	Serial.print("Count: ");
	Serial.println(dataRegister.count);
	dataRegister.ng = EEPROM.read(EEPROM_NG) << 24
						| EEPROM.read(EEPROM_NG+1) << 16
						| EEPROM.read(EEPROM_NG+2) << 8
						| EEPROM.read(EEPROM_NG+3);
	Serial.print("NG: ");
	Serial.println(dataRegister.ng);
	dataRegister.runningNumber = EEPROM.read(EEPROM_RUNNING_NUMBER) << 24
						| EEPROM.read(EEPROM_RUNNING_NUMBER+1) << 16
						| EEPROM.read(EEPROM_RUNNING_NUMBER+2) << 8
						| EEPROM.read(EEPROM_RUNNING_NUMBER+3);
	Serial.print("Running_Number: ");
	Serial.println(dataRegister.runningNumber);

	setupConfig.slaveId = EEPROM.read(EEPROM_SLAVE_ID);
	Serial.print("Slave ID: ");
	Serial.println(setupConfig.slaveId);

	setupConfig.modbusBaudrate = EEPROM.read(EEPROM_MODBUS_BAUDRATE) << 24
						| EEPROM.read(EEPROM_MODBUS_BAUDRATE+1) << 16
						| EEPROM.read(EEPROM_MODBUS_BAUDRATE+2) << 8
						| EEPROM.read(EEPROM_MODBUS_BAUDRATE+3);
	if(setupConfig.modbusBaudrate > 115200){
		setupConfig.modbusBaudrate = 115200;
	}
	Serial.print("ModBus_Baudrate: ");
	Serial.println(setupConfig.modbusBaudrate);
}

void initModbus(){
	Serial1.end();
	Serial2.end();
	Serial1.begin(setupConfig.modbusBaudrate, SERIAL_8N1, PIN_MODBUS_RX, PIN_MODBUS_TX);
	Serial2.begin(setupConfig.modbusBaudrate, SERIAL_8N1, PIN_MODBUS_RX_2, PIN_MODBUS_TX_2);
	modbus1.begin(&Serial1);
	modbus2.begin(&Serial2);
	modbus1.slave(setupConfig.slaveId);
	modbus1.setBaudrate(setupConfig.modbusBaudrate);
	modbus2.slave(setupConfig.slaveId);
	modbus2.setBaudrate(setupConfig.modbusBaudrate);

	for(int i = 0; i <= REG_MODBUS_ADDING_START+100; i++){
		modbus1.addHreg(i);
		modbus1.Hreg(i,0);
		modbus2.addHreg(i);
		modbus2.Hreg(i,0);
	}

	modbus1.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);
	modbus2.Hreg(REG_MODBUS_STATUS, dataRegister.eCurrentMachineStatus);

	modbus1.Hreg(REG_MODBUS_COUNT_H, dataRegister.count >> 16);
	modbus1.Hreg(REG_MODBUS_COUNT_L, dataRegister.count);
	modbus2.Hreg(REG_MODBUS_COUNT_H, dataRegister.count >> 16);
	modbus2.Hreg(REG_MODBUS_COUNT_L, dataRegister.count);

	modbus1.Hreg(REG_MODBUS_NG_H, dataRegister.ng >> 16);
	modbus1.Hreg(REG_MODBUS_NG_L, dataRegister.ng);
	modbus2.Hreg(REG_MODBUS_NG_H, dataRegister.ng >> 16);
	modbus2.Hreg(REG_MODBUS_NG_L, dataRegister.ng);

	modbus1.Hreg(REG_MODBUS_RUNNING_NUMBER_H, dataRegister.runningNumber >> 16);
	modbus1.Hreg(REG_MODBUS_RUNNING_NUMBER_L, dataRegister.runningNumber);
	modbus2.Hreg(REG_MODBUS_RUNNING_NUMBER_H, dataRegister.runningNumber >> 16);
	modbus2.Hreg(REG_MODBUS_RUNNING_NUMBER_L, dataRegister.runningNumber);
}

String macID(){
    uint8_t mac[WL_MAC_ADDR_LENGTH];
    String macID;
    WiFi.mode(WIFI_AP);
    WiFi.softAPmacAddress(mac);
    macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) + String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
    macID.toUpperCase();
    return macID;
}
void setupConfigMode(){
    Serial.println("[WifiService][setupAP] Open AP....");
    WiFi.softAPdisconnect();
    WiFi.disconnect();
    // server.close();
    delay(500/portTICK_PERIOD_MS);
    WiFi.mode(WIFI_AP);
    // String SSID_AP_MODE = SSID_PRE_AP_MODE + macID();
	String SSID_AP_MODE = SSID_PRE_AP_MODE + WiFi.softAPmacAddress();
    WiFi.softAP(SSID_AP_MODE.c_str());
    IPAddress APIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(APIP, gateway, subnet);
    Serial.println(SSID_AP_MODE);

    Serial.println("[WifiService][setupAP] Softap is running!");
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("[WifiService][setupAP] IP address: ");
    Serial.println(myIP);
}

void handleRoot(){
  char index_html[2048];
  snprintf_P(index_html, sizeof(index_html), index_html_handle_root, 
				setupConfig.slaveId, setupConfig.modbusBaudrate);
  server.send(200, "text/html", index_html);
}

void configModbus(){
	for (uint8_t i = 0; i < server.args(); i++) {
        if(server.argName(i) == "slave_id" && server.arg(i) != ""){
			String slaveIdStr = server.arg(i);
			setupConfig.slaveId = slaveIdStr.toInt();
			modbus1.slave(setupConfig.slaveId);
			modbus2.slave(setupConfig.slaveId);
			Serial.print("Slave ID: ");
			Serial.println(setupConfig.slaveId);
			EEPROM.write(EEPROM_SLAVE_ID, setupConfig.slaveId);
			EEPROM.commit();
        }
		if(server.argName(i) == "baudrate" && server.arg(i) != ""){
			String modbusBaudrateStr = server.arg(i);
			setupConfig.modbusBaudrate = modbusBaudrateStr.toInt();
			initModbus();
			Serial.print("ModBus_Baudrate: ");
			Serial.println(setupConfig.modbusBaudrate);
			EEPROM.write(EEPROM_MODBUS_BAUDRATE, setupConfig.modbusBaudrate >> 24);
			EEPROM.write(EEPROM_MODBUS_BAUDRATE+1, setupConfig.modbusBaudrate >> 16);
			EEPROM.write(EEPROM_MODBUS_BAUDRATE+2, setupConfig.modbusBaudrate >> 8);
			EEPROM.write(EEPROM_MODBUS_BAUDRATE+3, setupConfig.modbusBaudrate);
			EEPROM.commit();
        }
    }
    char index_html[2048];
	snprintf_P(index_html, sizeof(index_html), index_html_handle_root, 
				setupConfig.slaveId, setupConfig.modbusBaudrate);
  	server.send(200, "text/html", index_html);
}

void resetData(){
	resetDataOee();
    char index_html[2048];
	snprintf_P(index_html, sizeof(index_html), index_html_handle_root, 
				setupConfig.slaveId, setupConfig.modbusBaudrate);
  	server.send(200, "text/html", index_html);
}

void startWebServer(){
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config_modbus", HTTP_GET, configModbus);
  server.on("/reset_data", HTTP_GET, resetData);
//   server.onNotFound(notFound);
  server.begin();
  Serial.println( "HTTP server started" );
}

void resetDataOee(){
	Serial.println("Reset Data!!!");
	dataRegister.count = 0;;
	modbus1.Hreg(REG_MODBUS_COUNT_H, dataRegister.count >> 16);
	modbus1.Hreg(REG_MODBUS_COUNT_L, dataRegister.count);
	modbus2.Hreg(REG_MODBUS_COUNT_H, dataRegister.count >> 16);
	modbus2.Hreg(REG_MODBUS_COUNT_L, dataRegister.count);
	EEPROM.write(EEPROM_COUNT, dataRegister.count >> 24);
	EEPROM.write(EEPROM_COUNT+1, dataRegister.count >> 16);
	EEPROM.write(EEPROM_COUNT+2, dataRegister.count >> 8);
	EEPROM.write(EEPROM_COUNT+3, dataRegister.count);

	dataRegister.ng = 0;
	modbus1.Hreg(REG_MODBUS_NG_H, dataRegister.ng >> 16);
	modbus1.Hreg(REG_MODBUS_NG_L, dataRegister.ng);
	modbus2.Hreg(REG_MODBUS_NG_H, dataRegister.ng >> 16);
	modbus2.Hreg(REG_MODBUS_NG_L, dataRegister.ng);
	EEPROM.write(EEPROM_NG, dataRegister.ng >> 24);
	EEPROM.write(EEPROM_NG+1, dataRegister.ng >> 16);
	EEPROM.write(EEPROM_NG+2, dataRegister.ng >> 8);
	EEPROM.write(EEPROM_NG+3, dataRegister.ng);

	dataRegister.runningNumber = 0;
	modbus1.Hreg(REG_MODBUS_RUNNING_NUMBER_H, dataRegister.runningNumber >> 16);
	modbus1.Hreg(REG_MODBUS_RUNNING_NUMBER_L, dataRegister.runningNumber);
	modbus2.Hreg(REG_MODBUS_RUNNING_NUMBER_H, dataRegister.runningNumber >> 16);
	modbus2.Hreg(REG_MODBUS_RUNNING_NUMBER_L, dataRegister.runningNumber);
	EEPROM.write(EEPROM_RUNNING_NUMBER, dataRegister.runningNumber >> 24);
	EEPROM.write(EEPROM_RUNNING_NUMBER+1, dataRegister.runningNumber >> 16);
	EEPROM.write(EEPROM_RUNNING_NUMBER+2, dataRegister.runningNumber >> 8);
	EEPROM.write(EEPROM_RUNNING_NUMBER+3, dataRegister.runningNumber);
	EEPROM.commit();
}

void checkButtonConfigClick(){
    if(digitalRead(PIN_INPUT_CONFIG) == BTN_PRESSED){
        if(millis() - setupConfig.timeCheckTurnOnConfig >= CONFIG_HOLD_TIME){
			setupConfig.turnOnApMode = true;
			setupConfig.timeCheckTurnOffConfig  = millis();
            setupConfig.timeCheckTurnOnConfig = millis();
            setupConfigMode();
            startWebServer();
        }
    }
    else{
        setupConfig.timeCheckTurnOnConfig = millis();
    }

	if(setupConfig.turnOnApMode){
		if(WiFi.softAPgetStationNum() == 0){
			if(millis() - setupConfig.timeCheckTurnOffConfig >= TIME_TURN_OFF_WIFI){
				WiFi.softAPdisconnect();
				setupConfig.turnOnApMode = false;
			}
		}
		else{
			setupConfig.timeCheckTurnOffConfig = millis();
		}
	}
}

void printData(){
	Serial.print("Count: ");
	Serial.print(dataRegister.count);
	Serial.print(" - Status: ");
	switch (dataRegister.eCurrentMachineStatus)
	{
	case MACHINE_STATUS_RUN:
		Serial.print("Run");
		break;
	case MACHINE_STATUS_IDLE:
		Serial.print("Idle");
		break;
	case MACHINE_STATUS_ERROR:
		Serial.print("Error");
		break;
	case MACHINE_STATUS_UNKNOW:
		Serial.print("Unknow");
		break;
	default:
		break;
	}
	Serial.print(" - NG: ");
	Serial.print(dataRegister.ng);
	Serial.print(" - Running_Number: ");
	Serial.println(dataRegister.runningNumber);
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector
	Serial.begin(115200, SERIAL_8N1);
	EEPROM.begin(256);
	pinMode(PIN_OUTPUT_RED, OUTPUT);
	pinMode(PIN_OUTPUT_YELLOW, OUTPUT);
	pinMode(PIN_OUTPUT_GREEN, OUTPUT);
	pinMode(PIN_OUTPUT_4, OUTPUT);
	pinMode(PIN_INPUT_COUNT, 			INPUT);
	pinMode(PIN_INPUT_RUN, 				INPUT);
	pinMode(PIN_INPUT_IDLE, 			INPUT);
	pinMode(PIN_INPUT_ERROR, 			INPUT);
	pinMode(PIN_INPUT_NG, 				INPUT);
	pinMode(PIN_INPUT_RUNNING_NUMBER, 	INPUT);
	pinMode(PIN_INPUT_CONFIG, 			INPUT);
	delay(100/portTICK_PERIOD_MS);
	digitalWrite(PIN_OUTPUT_RED, OFF_FET);
	digitalWrite(PIN_OUTPUT_YELLOW, OFF_FET);
	digitalWrite(PIN_OUTPUT_GREEN, OFF_FET); 
	digitalWrite(PIN_OUTPUT_4, OFF_FET);
	delay(100/portTICK_PERIOD_MS);
	loadDataBegin();
	initModbus();

	xTaskCreatePinnedToCore(
		checkInputCount,    /* Function to implement the task */
		"checkInputCount",  /* Name of the task */
		4096,             /* Stack size in words */
		NULL,             /* Task input parameter */
		0,                /* Priority of the task */
		NULL,             /* Task handle. */
		0);               /* Core where the task should run */
	xTaskCreatePinnedToCore(
		checkInputMCStatus,    /* Function to implement the task */
		"checkInputMCStatus",  /* Name of the task */
		4096,             /* Stack size in words */
		NULL,             /* Task input parameter */
		0,                /* Priority of the task */
		NULL,             /* Task handle. */
		0);               /* Core where the task should run */
	xTaskCreatePinnedToCore(
		checkInputNG,    /* Function to implement the task */
		"checkInputNG",  /* Name of the task */
		4096,             /* Stack size in words */
		NULL,             /* Task input parameter */
		0,                /* Priority of the task */
		NULL,             /* Task handle. */
		0);               /* Core where the task should run */
	xTaskCreatePinnedToCore(
		checkInputRunningNumber,    /* Function to implement the task */
		"checkInputRunningNumber",  /* Name of the task */
		4096,             /* Stack size in words */
		NULL,             /* Task input parameter */
		0,                /* Priority of the task */
		NULL,             /* Task handle. */
		0);               /* Core where the task should run */
}

void loop() {
	checkButtonConfigClick();
	modbus1.task();
	modbus2.task();
	if(millis() - setupConfig.timePrintData >= TIME_PRINT_DATA){
		setupConfig.timePrintData = millis();
		printData();
	}

	yield();
	server.handleClient();
	vTaskDelay(10/portTICK_PERIOD_MS);
}