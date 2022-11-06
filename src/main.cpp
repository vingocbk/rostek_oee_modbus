#include <Arduino.h>
#include <ModbusRTU.h>
#include <EEPROM.h>
#include <Wire.h>
#include "DS3231.h"

#define REGN 10
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

enum{
	EEPROM_ADD_SLAVE_ID 	= 1,
	EEPROM_ADD_BUTTON_DEBOUNCE,
	EEPROM_ADD_INPUT_DEBOUNCE,
	EEPROM_ADD_COUNTER_DEBOUNCE,
	EEPROM_ADD_COUNTER_DEBOUNCE_LOW,
	EEPROM_ADD_COUNTER_DEBOUNCE_HIGH
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

	REG_SET_SLAVE_ID        				= 100,
	REG_SET_SLAVE_ID_VALUE,    					
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
	REG_SET_COUNTER_DEBOUNCE_LOW,        
	REG_SET_COUNTER_DEBOUNCE_HIGH,       
} ;


#define MODBUS_DATA_BUFF_SIZE               250
#define TIME_COUNTER_MINIMUM                0

uint8_t slave_id = 1;
ModbusRTU modBus;

DS3231 ds3231_clock;
RTCDateTime dt;

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


    bool bIsMachineStateChanged = false;                  /*!< Machine status has changed  */

    bool bIsRunSwitchGPIOPresent = false;
    bool bIsIdleSwitchGPIOPresent = false;
    bool bIsErrorSwitchGPIOPresent = false;
    bool bIsRestartSwitchGPIOPresent = false;
    bool bIsStartChangeMoldSwitchGPIOPresent = false;
    bool bIsStopChangeMoldSwitchGPIOPresent = false;
    bool bIsBuzzerGPIOPresent = false;
    bool bIsChangeMoldGPIOPresent = false;
    bool bIsRelay3GPIOPresent = false;

    bool bIsRunSwitchGPIOStatus = false;
    bool bIsIdleSwitchGPIOStatus = false;
    bool bIsErrorSwitchGPIOStatus = false;
    bool bIsCounterGPIOStatus = false;
    bool bIsBuzzerGPIOStatus = false;
    // bool bIsSyncDataWorkingShift = false;
    bool bIsSyncActualWhenOffline = false;
    bool bIsSyncWorkingShiftTime = false;
    bool bIsSyncDataDisconnect = false;

    uint16_t modbusData[MODBUS_DATA_BUFF_SIZE];
    // CALLING_AGV  ePreviousCallingAGVOrderType = CALLING_AGV_NA;
    // ORDER_TYPE_SELECT  ePreviousOrderTypeSelection = ORDER_TYPE_SELECT_NA;
    int32_t timestamp_int32 = 0;
} OEEVars;

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