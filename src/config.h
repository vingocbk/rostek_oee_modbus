#ifndef CONFIG_H
#define CONFIG_H
#define EEPROM_TIME_RELAY_1         0
#define EEPROM_TIME_RELAY_2         2
#define EEPROM_TIME_RELAY_3         4
#define EEPROM_TIME_RELAY_4         6

#define SERIAL_BAUDRATE             115200
#define MAX_SIZE_EEPROM_BUFFER      512
#define CONFIG_HOLD_TIME            3000
#define TIME_TURN_OFF_WIFI          10*60*1000  //10 minute
#define TIME_PRINT_DATA             3000

#define HTTP_PORT                   80
#define WL_MAC_ADDR_LENGTH          6
#define SSID_PRE_AP_MODE            "OEE-"

#define BTN_PRESSED                 LOW
#define RELAY_ON                    HIGH
#define RELAY_OFF                   LOW

#define CONVERT_S_TO_MS              1000


#endif      //CONFIG_H