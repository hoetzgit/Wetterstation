/*
*
* config Wetter001
*
*/
// on/off 
#define WITH_W132 1           // 0 to disable Ventus W132 sensor readout.
#define WITH_W174 1           // 0 to disable Ventus W174 sensor readout.
#define WITH_BME280 1         // 0 to disable BME280 sensor readout.
#define WITH_BME680 0         // 0 to disable BME260 sensor readout.
#define WITH_BH1750 1         // 0 to disable BH1750 sensor readout.
#define WITH_BH1750B 1        // 0 to disable BH1750 sensor readout.
#define WITH_BH1750AS 1       // 0 to disable BH1750 sensor readout.
#define WITH_ML8511 0         // 0 to disable ML8511 sensor readout.
#define WITH_VEML6070 0       // 0 to disable VEML6070 sensor readout.
#define WITH_VEML6075 1       // 0 to disable VEML6075 sensor readout.
#define WITH_MHRD 0           // 0 to disable MH-RD sensor readout.
#define WITH_DS18B20 1        // 0 to disable DS18B20 sensor readout.
#define WITH_AS3935MI 1       // 0 to disable AS3935MI sensor readout.
#define WITH_AS3935KA 1       // 0 to disable AS3935KA sensor readout.
#define WITH_TSL2591 1        // 0 to disable TSL2591 sensor readout
#define WITH_VOLTAGE 1        // 0 to disable Voltage readout.
#define WITH_FLASH 1          // 0 to disable Flash.
#define WITH_SD 0             // 0 to disable SD Datalogger.
#define WITH_RTC 1            // 0 to disable RTC.
#define WITH_ZAMBRETTI 0      // 0 to disable Zambretti Forecast
#define WITH_WEEWX_UPLOAD 0   // 0 to disable WeeWX Upload.
#define WITH_DEEPSLEEP 1      // 0 to disable Deepsleep and enable delay function
#define WITH_TIMER 0          // 0 to disable Timer Routines
#define WITH_BROKER 1         // 0 to disable publish to broker

// Station IP Address
IPAddress ip(192, 168, 0, 171);

// device Id
#define STATION_ID "ventus"
#define STATION_NUM_ID 1

// BROKER
#if (WITH_BROKER > 0)
  #define BROKER_CLIENTID "ventus"
#endif
  
// Weatherunderground Protokoll to WeeWX
#define WEEWX_SERVER "192.168.0.187"
#define WEEWX_ID "VENTUS"
#define WEEWX_PASSWORD "geheim"

// Voltage divider R1 = 220k+100k+220k =540k and R2=100k
#define VOLTAGE_CALIB_FACTOR 5.20 // change this value to calibrate the battery voltage

// BME280 offsets
#define BME280_TEMPERATURE_OFFSET 0.0
#define BME280_PRESSURE_OFFSET 0.0
#define BME280_HUMIDITY_OFFSET 0.0

// BME680 offsets
#define BME680_TEMPERATURE_OFFSET 0.0
#define BME680_PRESSURE_OFFSET 0.0
#define BME680_HUMIDITY_OFFSET 0.0

// W132 offsets
#define W132_WINDDIR_OFFSET 0

// Sleep time
#if (WITH_W132 > 0)
  #define SLEEP_DAY_SECONDS 30
  #define SLEEP_NIGHT_SECONDS 30
#elif (WITH_W174 > 0) 
  #define SLEEP_DAY_SECONDS 32
  #define SLEEP_NIGHT_SECONDS 32
#else
  #define SLEEP_DAY_SECONDS 30
  #define SLEEP_NIGHT_SECONDS 30
#endif
