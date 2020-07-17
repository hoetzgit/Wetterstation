/*
*
* config Solar Weather Station
*
*/
// on/off 
#define WITH_W132 0           // 0 to disable Ventus W132 sensor readout.
#define WITH_W174 0           // 0 to disable Ventus W174 sensor readout.
#define WITH_BME280 1         // 0 to disable BME280 sensor readout.
#define WITH_BME680 0         // 0 to disable BME260 sensor readout.
#define WITH_BH1750 0         // 0 to disable BH1750 sensor readout.
#define WITH_BH1750B 0        // 0 to disable BH1750 sensor readout.
#define WITH_BH1750AS 0       // 0 to disable BH1750 sensor readout.
#define WITH_ML8511 0         // 0 to disable ML8511 sensor readout.
#define WITH_VEML6070 0       // 0 to disable VEML6070 sensor readout.
#define WITH_VEML6075 0       // 0 to disable VEML6075 sensor readout.
#define WITH_MHRD 0           // 0 to disable MH-RD sensor readout.
#define WITH_DS18B20 0        // 0 to disable DS18B20 sensor readout.
#define WITH_AS3935 0         // 0 to disable AS3935 sensor readout.
#define WITH_TSL2591 0        // 0 to disable TSL2591 sensor readout
#define WITH_VOLTAGE 1        // 0 to disable Voltage readout.
#define WITH_FLASH 1          // 0 to disable Flash.
#define WITH_SD 0             // 0 to disable SD Datalogger.
#define WITH_RTC 1            // 0 to disable RTC.
#define WITH_ZAMBRETTI 0      // 0 to disable Zambretti Forecast
#define WITH_WEEWX_UPLOAD 1   // 0 to disable WeeWX Upload.
#define WITH_DEEPSLEEP 1      // 0 to disable Deepsleep and enable delay function
#define WITH_TIMER 0          // 0 to disable Timer Routines
#define WITH_BROKER 1         // 0 to disable publish to broker

// device Id
#define STATION_ID  "solar"
#define STATION_NUM_ID 2

// WLAN
#define WLAN_SSID "Radio Eriwan"
#define WLAN_PASSWORD	"mgrsuacccrert136"

// BROKER
#if (WITH_BROKER > 0)
  #define BROKER_SERVER "192.168.0.182"
  #define BROKER_SERVERPORT 1883 
  #define BROKER_USERNAME ""
  #define BROKER_PASSWORD ""
  #define BROKER_CLIENTID "solar"
  #define BROKER_OUTPUTFORMAT "json"
  #define BROKER_COLLECTED_VALUES "loop"
  #define BROKER_MQTT_BUFFER_SIZE 1024
  #define BROKER_KEEP_ALIVE 90
  const String ACTION_TOPIC_PRAEFIX = String(STATION_ID) + "/action/";
#endif

// NTP Server
#define NTP_SERVER "192.168.0.254"

// Station IP Address
IPAddress ip(192, 168, 0, 172);
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);

// Weatherunderground Protokoll to WeeWX
#define WEEWX_SERVER "192.168.0.182"
#define WEEWX_SERVER_PORT 8080
#define WEEWX_SERVER_PATH "/weatherstation/updateweatherstation.php"
#define WEEWX_SERVER_UPLOAD "GET /weatherstation/updateweatherstation.php"
#define WEEWX_ID "SOLAR"
#define WEEWX_PASSWORD "geheim"

// Voltage Sensor Id
#define VOLTAGE_SENSOR_ID "stationVoltage"
// Voltage divider R1 = 220k+100k+220k =540k and R2=100k
#define VOLTAGE_CALIB_FACTOR 5.24 // change this value to calibrate the battery voltage

// invalid Temperature
#define INVALID_FLOAT_VALUE -9999.0

// BME280 offsets
#define BME280_TEMPERATURE_OFFSET -1.0
#define BME280_PRESSURE_OFFSET -2.0
#define BME280_HUMIDITY_OFFSET -2.0

// BME680 offsets
#define BME680_TEMPERATURE_OFFSET 0.0
#define BME680_PRESSURE_OFFSET 0.0
#define BME680_HUMIDITY_OFFSET 0.0

// W132 offsets
#define W132_WINDDIR_OFFSET 0
// W132 max offline
#define W132_MAX_OFFLINE 300 // 5 Minutes

// W174 max offline
#define W174_MAX_OFFLINE 300 // 5 Minutes
// W174 rain bucket size
#define W174_BUCKET_SIZE 0.25 // 0.25mm rain/count

// Sleep Mode hours
#define SLEEP_DAY_HOUR 5
#define SLEEP_NIGHT_HOUR 22

// Sleep time
#define SLEEP_DAY_SECONDS 90
#define SLEEP_NIGHT_SECONDS 90
