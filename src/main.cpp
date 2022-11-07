#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>
#include <Wire.h>
#include "DS3231.h"

#define PIN_RX_MODBUS                     	16
#define PIN_TX_MODBUS                     	17
#define PIN_INPUT_COUNTER                 	26    	//INPUT 8
#define PIN_INPUT_RUN_STATUS              	25   	//INPUT 7
#define PIN_INPUT_IDLE_STATUS             	33    	//INPUT 6
#define PIN_INPUT_ERROR_STATUS            	32    	//INPUT 5
#define PIN_INPUT_RESET_MC                	35    	//INPUT 4
#define PIN_INPUT_START_CHANGE_MOLD       	34    	//INPUT 3
#define PIN_INPUT_STOP_CHANGE_MOLD        	39    	//INPUT 2
#define PIN_INPUT_RESET_SLAVE_ID          	36    	//INPUT 1
#define PIN_OUTPUT_CHANGEMOLD_STATUS      	5    	//INPUT 1

#define MODBUS_DATA_BUFF_SIZE               250
#define TIME_COUNTER_MINIMUM                0
#define SLAVE_ID_DEFAUT                     200
#define DEBOUNCE_BUTTON_DEFAUT              500
#define DEBOUNCE_INPUT_DEFAUT               500
#define DEBOUNCE_COUNTER_DEFAUT             500
#define IS_INSTALLED                        1
#define NOT_INSTALLED                       0

enum{
    EEPROM_ADD_IS_SETUP_SLAVE_ID = 1,
	EEPROM_ADD_SLAVE_ID,
    EEPROM_ADD_IS_SETUP_BUTTON_DEBOUNCE,
	EEPROM_ADD_BUTTON_DEBOUNCE_HIGH,
    EEPROM_ADD_BUTTON_DEBOUNCE_LOW,
    EEPROM_ADD_IS_SETUP_INPUT_DEBOUNCE,
	EEPROM_ADD_INPUT_DEBOUNCE_HIGH,
    EEPROM_ADD_INPUT_DEBOUNCE_LOW,
    EEPROM_ADD_IS_SETUP_COUNTER_DEBOUNCE,
	EEPROM_ADD_COUNTER_DEBOUNCE_HIGH,
    EEPROM_ADD_COUNTER_DEBOUNCE_LOW,
	// EEPROM_ADD_COUNTER_DEBOUNCE_LOW,
	// EEPROM_ADD_COUNTER_DEBOUNCE_HIGH
};

enum{
	REG_U16_OEE_ACTUAL                  	= 1,
	REG_U32_OEE_RUN_TIME_HIGH,           
	REG_U32_OEE_RUN_TIME_LOW,            
	REG_U16_OEE_MC_STATUS,               
	REG_U32_OEE_TOTAL_TIME_HIGH,         
	REG_U32_OEE_TOTAL_TIME_LOW,          
	REG_U16_OEE_DONE_CHANGE_MOLD,      
	REG_GET_SLAVE_ID,   
	REG_GET_BUTTON_DEBOUNCE, 
	REG_GET_INPUT_DEBOUNCE, 
	REG_GET_COUNTER_DEBOUNCE,
	REG_GET_COUNTER_DEBOUNCE_LOW,
	REG_GET_COUNTER_DEBOUNCE_HIGH,

	REG_SET_SLAVE_ID                        =100,    					
	REG_SET_TIME,    										//0-none, 1 - time Epoch Unix, - 2 time year,hour
	REG_SET_CURRENT_TIME_EPOCH,       			
	REG_SET_CURRENT_YEAR,       					
	REG_SET_CURRENT_MONTH,        				
	REG_SET_CURRENT_DATE,        				
	REG_SET_CURRENT_HOUR,        				
	REG_SET_CURRENT_MINUTE,        				
	REG_SET_CURRENT_SECOND,
	REG_SET_WORKING_SHIFT,        				
	REG_SET_WORKING_SHIFT_1_START_HOUR,
	REG_SET_WORKING_SHIFT_1_START_MINUTE,
	REG_SET_WORKING_SHIFT_1_START_SECOND,
	REG_SET_WORKING_SHIFT_1_END_HOUR,
	REG_SET_WORKING_SHIFT_1_END_MINUTE,
	REG_SET_WORKING_SHIFT_1_END_SECOND,
	REG_SET_WORKING_SHIFT_2_START_HOUR,
	REG_SET_WORKING_SHIFT_2_START_MINUTE,
	REG_SET_WORKING_SHIFT_2_START_SECOND,
	REG_SET_WORKING_SHIFT_2_END_HOUR,
	REG_SET_WORKING_SHIFT_2_END_MINUTE,
	REG_SET_WORKING_SHIFT_2_END_SECOND,
	REG_SYNC_ACTUAL,
	REG_SYNC_RUNNING_TIME,
	REG_SYNC_DONE_CHANGE_MOLD,
	REG_SYNC_TOTAL_TIME,
	REG_SET_BUTTON_DEBOUNCE,
	REG_SET_INPUT_DEBOUNCE,        		
	REG_SET_COUNTER_DEBOUNCE,
    REG_RESET_ALL
} ;

ModbusRTU modBus;
struct SetupSlave
{
    bool isSetupSlaveId = false;
    uint8_t slaveId = SLAVE_ID_DEFAUT;
} setupSlave;

// DS3231 ds3231_clock;
// RTCDateTime dt;

typedef enum{
    MACHINE_STATUS_NA = 0,
    MACHINE_STATUS_EXECUTE = 1,
    MACHINE_STATUS_IDLE = 2,
    MACHINE_STATUS_ERROR = 3
} MACHINE_STATUS;

struct SyncData
{
    bool isSyncDateTime = false;
    bool isSyncWorkingShift = false;
    bool isSyncData = false;
} syncData;

struct OEEVariables {
    //save data
    int32_t i32RunningTimeCounter = TIME_COUNTER_MINIMUM;             /*!< Total Runningtime second counter */
    int32_t i32SyncOffsetRunningTimeCounter = TIME_COUNTER_MINIMUM;             /*!< Total Offset Runningtime second counter */
    int32_t i32RunningTimePrevious = TIME_COUNTER_MINIMUM;             /*!< Total Runningtime second counter */
    int32_t i32PartialRunningTimeCounter = TIME_COUNTER_MINIMUM;             /*!< Total Runningtime second counter */
    //save data
    int32_t i32DownTimeCounter = TIME_COUNTER_MINIMUM;                /*!< Total Downtime second counter  */
    int32_t i32DownTimePrevious = TIME_COUNTER_MINIMUM;  
    int32_t i32PartialDownTimeCounter = TIME_COUNTER_MINIMUM;          /*!< Downtime second counter  */
    int32_t i32ChangMoldDownTimeCounter = TIME_COUNTER_MINIMUM;        /*!< Change Mold Downtime second counter  */
    int32_t i32QualityDownTimeCounter = TIME_COUNTER_MINIMUM;          /*!< Quality Downtime second counter  */
    int32_t i32MachineErrorDownTimeCounter = TIME_COUNTER_MINIMUM;     /*!< Machine Error Downtime second counter  */
    int32_t i32OthersDownTimeCounter = TIME_COUNTER_MINIMUM;           /*!< Other Reason Downtime second counter  */
    int32_t i32NewDownTimeCounter  = TIME_COUNTER_MINIMUM;             /*!< New Update, get data onserver before calculate  */
    int32_t i32NewDownTimeCounterPrevious  = TIME_COUNTER_MINIMUM;

    int32_t i32ChangMoldDownTimePrevious = TIME_COUNTER_MINIMUM;        /*!< Change Mold Downtime second previous  */
    int32_t i32QualityDownTimePrevious = TIME_COUNTER_MINIMUM;          /*!< Quality Downtime second previous  */
    int32_t i32MachineErrorDownTimePrevious = TIME_COUNTER_MINIMUM;     /*!< Machine Error Downtime second previous  */
    int32_t i32OthersDownTimePrevious = TIME_COUNTER_MINIMUM;           /*!< Other Reason Downtime second previous  */

    int32_t i32SyncOffsetUpTime = TIME_COUNTER_MINIMUM;

    uint16_t ui16ProductCounter = 0;
    uint16_t ui16SyncOffsetProductCounter = 0;
    uint16_t ui16ProductCounterDaily = 0;
    int32_t i32ActualProduct = 0;   //save data
    int32_t i32TargetProduct = 0;   //save data
    int16_t i16RunningNumber = 0;   //number of done change mold

    int32_t i32MPTProductInput = 0;
    int32_t i32NGProductInput = 0;

    float     fAvailability = 0.0;
    float     fProductivity = 0.0;
    float     fQuality = 0.0;
    float     fOEEOverall = 0.0;

    uint32_t ui32LastMachineExecuteStatusTime     = 0;
    uint32_t ui32LastMachineDowntimeStatusTime     = 0;
    uint32_t ui32OrderExecuteStartTime             = 0;
    uint32_t ui32OEEDataPublishTime             = 0;

    uint8_t ui8WorkingShiftStartHour1 = 0;
    uint8_t ui8WorkingShiftStartMinute1 = 0;
    uint8_t ui8WorkingShiftEndHour1 = 0;
    uint8_t ui8WorkingShiftEndMinute1 = 0;
    uint8_t ui8WorkingShiftStartHour2 = 0;
    uint8_t ui8WorkingShiftStartMinute2 = 0;
    uint8_t ui8WorkingShiftEndHour2 = 0;
    uint8_t ui8WorkingShiftEndMinute2 = 0;
    uint8_t ui8CountCurrentWorking = 0;
    uint8_t ui8CountCurrentChangeMold1 = 0;
    uint8_t ui8CountCurrentChangeMold2 = 0;
    uint8_t ui8CountCurrentChangeMold3 = 0;

    int32_t i32PartialDownTimeTimeStamp = 0;                           /*!< Downtime time stamp  */
    int32_t i32OrderExecutedTimeCounter = TIME_COUNTER_MINIMUM;        /*!< Order execute second counter from order state change to ready  */

    bool bIsDownTimeHappenned = false;                  /*!< Downtime status  */
    bool bIsChangeMold = false;                    /*!< Change mold status */
    bool bIsChangeMoldFinished = false;            /*!< Change mold finished status */

    MACHINE_STATUS  eCurrentMachineStatus = MACHINE_STATUS_NA;
    MACHINE_STATUS  ePreviousMachineStatus = MACHINE_STATUS_NA;


    bool bIsRunSwitchGPIOStatus = false;
    bool bIsIdleSwitchGPIOStatus = false;
    bool bIsErrorSwitchGPIOStatus = false;
    bool bIsCounterGPIOStatus = false;
    bool bIsBuzzerGPIOStatus = false;
    // bool bIsSyncDataWorkingShift = false;
    bool bIsSyncActualWhenOffline = false;
    bool bIsSyncWorkingShiftTime = false;
    bool bIsSyncDataDisconnect = false;
    // bool bIsSyncDataActual = false;

    uint16_t modbusData[MODBUS_DATA_BUFF_SIZE];
    uint16_t ui16buttonDebounce  = DEBOUNCE_BUTTON_DEFAUT;
    uint16_t ui16inputDebounce = DEBOUNCE_INPUT_DEFAUT;
    uint16_t ui16CounterDebounce = DEBOUNCE_COUNTER_DEFAUT;

} OEEVars;

void loadDataBegin();
void initModbus();
void checkCommandModbus();
void TaskReadInput(void *pvParameters);
void TaskReadCounter(void *pvParameters);

void loadDataBegin(){
    if(EEPROM.read(EEPROM_ADD_IS_SETUP_SLAVE_ID) == IS_INSTALLED){
        setupSlave.isSetupSlaveId = true;
        setupSlave.slaveId = EEPROM.read(EEPROM_ADD_SLAVE_ID);
    }
    else{
        setupSlave.slaveId = SLAVE_ID_DEFAUT;
    }
    if(EEPROM.read(EEPROM_ADD_IS_SETUP_BUTTON_DEBOUNCE) == IS_INSTALLED){
        OEEVars.ui16buttonDebounce = EEPROM.read(EEPROM_ADD_BUTTON_DEBOUNCE_HIGH);
        OEEVars.ui16buttonDebounce = (OEEVars.ui16buttonDebounce << 8) | EEPROM.read(EEPROM_ADD_BUTTON_DEBOUNCE_LOW);
    }
    if(EEPROM.read(EEPROM_ADD_IS_SETUP_INPUT_DEBOUNCE) == IS_INSTALLED){
        OEEVars.ui16inputDebounce = EEPROM.read(EEPROM_ADD_INPUT_DEBOUNCE_HIGH);
        OEEVars.ui16inputDebounce = (OEEVars.ui16inputDebounce << 8) | EEPROM.read(EEPROM_ADD_INPUT_DEBOUNCE_LOW);
    }
    if(EEPROM.read(EEPROM_ADD_IS_SETUP_COUNTER_DEBOUNCE) == IS_INSTALLED){
        OEEVars.ui16CounterDebounce = EEPROM.read(EEPROM_ADD_IS_SETUP_COUNTER_DEBOUNCE);
        OEEVars.ui16CounterDebounce = (OEEVars.ui16CounterDebounce << 8) | EEPROM.read(EEPROM_ADD_IS_SETUP_COUNTER_DEBOUNCE);
    }
}
void initModbus(){
	modBus.slave(setupSlave.slaveId);
    for(int i = 0; i < MODBUS_DATA_BUFF_SIZE; i++){
        modBus.addHreg(i);
	    modBus.Hreg(i, 0);
    }
	modBus.Hreg(REG_SET_SLAVE_ID, setupSlave.slaveId);
    modBus.Hreg(REG_SET_BUTTON_DEBOUNCE, OEEVars.ui16buttonDebounce);
    modBus.Hreg(REG_SET_INPUT_DEBOUNCE, OEEVars.ui16inputDebounce);
    modBus.Hreg(REG_SET_COUNTER_DEBOUNCE, OEEVars.ui16CounterDebounce);
}

void checkCommandModbus(){
    if(setupSlave.slaveId != modBus.Hreg(REG_SET_SLAVE_ID)){
        setupSlave.slaveId = modBus.Hreg(REG_SET_SLAVE_ID);
        modBus.slave(setupSlave.slaveId);
        EEPROM.write(EEPROM_ADD_IS_SETUP_SLAVE_ID, IS_INSTALLED);
        EEPROM.write(EEPROM_ADD_SLAVE_ID, setupSlave.slaveId);
        EEPROM.commit();
    }
    if(OEEVars.ui16buttonDebounce != modBus.Hreg(REG_SET_BUTTON_DEBOUNCE)){
        OEEVars.ui16buttonDebounce = modBus.Hreg(REG_SET_BUTTON_DEBOUNCE);
        EEPROM.write(EEPROM_ADD_IS_SETUP_BUTTON_DEBOUNCE, IS_INSTALLED);
        EEPROM.write(EEPROM_ADD_BUTTON_DEBOUNCE_HIGH, uint8_t(OEEVars.ui16buttonDebounce >> 8));
        EEPROM.write(EEPROM_ADD_BUTTON_DEBOUNCE_LOW, uint8_t(OEEVars.ui16buttonDebounce));
        EEPROM.commit();
    }
    if(OEEVars.ui16inputDebounce != modBus.Hreg(REG_SET_INPUT_DEBOUNCE)){
        OEEVars.ui16inputDebounce = modBus.Hreg(REG_SET_INPUT_DEBOUNCE);
        EEPROM.write(EEPROM_ADD_IS_SETUP_INPUT_DEBOUNCE, IS_INSTALLED);
        EEPROM.write(EEPROM_ADD_INPUT_DEBOUNCE_HIGH, uint8_t(OEEVars.ui16inputDebounce >> 8));
        EEPROM.write(EEPROM_ADD_INPUT_DEBOUNCE_LOW, uint8_t(OEEVars.ui16inputDebounce));
        EEPROM.commit();
    }
    if(OEEVars.ui16CounterDebounce != modBus.Hreg(REG_SET_COUNTER_DEBOUNCE)){
        OEEVars.ui16CounterDebounce = modBus.Hreg(REG_SET_COUNTER_DEBOUNCE);
        EEPROM.write(EEPROM_ADD_IS_SETUP_COUNTER_DEBOUNCE, IS_INSTALLED);
        EEPROM.write(EEPROM_ADD_COUNTER_DEBOUNCE_HIGH, uint8_t(OEEVars.ui16CounterDebounce >> 8));
        EEPROM.write(EEPROM_ADD_COUNTER_DEBOUNCE_LOW, uint8_t(OEEVars.ui16CounterDebounce));
        EEPROM.commit();
    }
    if(modBus.Hreg(REG_RESET_ALL) == IS_INSTALLED){
        modBus.Hreg(REG_RESET_ALL, NOT_INSTALLED);
        EEPROM.write(EEPROM_ADD_IS_SETUP_SLAVE_ID, NOT_INSTALLED);
        EEPROM.write(EEPROM_ADD_SLAVE_ID, SLAVE_ID_DEFAUT);
        EEPROM.write(EEPROM_ADD_IS_SETUP_BUTTON_DEBOUNCE, NOT_INSTALLED);
        EEPROM.write(EEPROM_ADD_BUTTON_DEBOUNCE_HIGH, uint8_t(DEBOUNCE_BUTTON_DEFAUT >> 8));
        EEPROM.write(EEPROM_ADD_BUTTON_DEBOUNCE_LOW, uint8_t(DEBOUNCE_BUTTON_DEFAUT));

        EEPROM.write(EEPROM_ADD_IS_SETUP_INPUT_DEBOUNCE, IS_INSTALLED);
        EEPROM.write(EEPROM_ADD_INPUT_DEBOUNCE_HIGH, uint8_t(DEBOUNCE_INPUT_DEFAUT >> 8));
        EEPROM.write(EEPROM_ADD_INPUT_DEBOUNCE_LOW, uint8_t(DEBOUNCE_INPUT_DEFAUT));

        EEPROM.write(EEPROM_ADD_IS_SETUP_COUNTER_DEBOUNCE, IS_INSTALLED);
        EEPROM.write(EEPROM_ADD_COUNTER_DEBOUNCE_HIGH, uint8_t(DEBOUNCE_COUNTER_DEFAUT >> 8));
        EEPROM.write(EEPROM_ADD_COUNTER_DEBOUNCE_LOW, uint8_t(DEBOUNCE_COUNTER_DEFAUT));
        EEPROM.commit();
    }
}

void TaskReadInput(void *pvParameters){
    for(;;){

    }
}

void TaskReadCounter(void *pvParameters){
    for(;;){

    }
}


void setup() {
	Serial.begin(115200, SERIAL_8N1);
	Serial1.begin(9600, SERIAL_8N1, PIN_RX_MODBUS, PIN_TX_MODBUS);
	EEPROM.begin(512);
    pinMode(PIN_INPUT_COUNTER, INPUT);
    pinMode(PIN_INPUT_RUN_STATUS, INPUT);
    pinMode(PIN_INPUT_IDLE_STATUS, INPUT);
    pinMode(PIN_INPUT_ERROR_STATUS, INPUT);
    pinMode(PIN_INPUT_RESET_MC, INPUT);
    pinMode(PIN_INPUT_START_CHANGE_MOLD, INPUT);
    pinMode(PIN_INPUT_STOP_CHANGE_MOLD, INPUT);
    pinMode(PIN_INPUT_RESET_SLAVE_ID, INPUT);
    pinMode(PIN_OUTPUT_CHANGEMOLD_STATUS, OUTPUT);
#if defined(ESP32) || defined(ESP8266)
	modBus.begin(&Serial1);
#else
	modBus.begin(&Serial1);
	//modBus.begin(&Serial1, RXTX_PIN);  //or use RX/TX direction control pin (if required)
	modBus.setBaudrate(9600);
#endif

	initModbus();
	loadDataBegin();
	// ds3231_clock.begin();

    xTaskCreatePinnedToCore(
    TaskReadInput,    /* Function to implement the task */
    "TaskReadInput",  /* Name of the task */
    4096,             /* Stack size in words */
    NULL,             /* Task input parameter */
    0,                /* Priority of the task */
    NULL,             /* Task handle. */
    0);               /* Core where the task should run */

    xTaskCreatePinnedToCore(
    TaskReadCounter,    /* Function to implement the task */
    "OEEDefaultTask",  /* Name of the task */
    4096,             /* Stack size in words */
    NULL,             /* Task input parameter */
    0,                /* Priority of the task */
    NULL,             /* Task handle. */
    0);               /* Core where the task should run */
  
}

void loop() {
    checkCommandModbus();
	modBus.task();
    vTaskDelay(100/ portTICK_PERIOD_MS);
	yield();
}