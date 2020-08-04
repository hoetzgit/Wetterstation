/*
*
* config Weather Station
*
*/
// Debug options
#define WITH_DEBUG_TO_SERIAL 1 // 0 to disable debug output to serial
#define WITH_DEBUG_TO_BROKER 1 // 0 to disable debug output to broker
#define WITH_ERROR_TO_BROKER 1 // 0 to disable error output to broker
#define WITH_DEBUG_BROKER 1    // 0 to disable broker debug serial output 
#define WITH_DEBUG_SENSORS 1   // 0 to disable sensor debug serial output
#define WITH_CALC_TO_BROKER 1  // 0 to disable calulated values publishing

// Station altitude
#define STATION_ALTITUDE 394

// WLAN
#define WLAN_SSID "Radio Eriwan"
#define WLAN_PASSWORD "xxx"

// Station IP Address
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);

// BROKER
#if (WITH_BROKER > 0)
  #define BROKER_SERVER "192.168.0.182"
  #define BROKER_SERVERPORT 1883 
  #define BROKER_USERNAME ""
  #define BROKER_PASSWORD ""
  #define BROKER_OUTPUTFORMAT "json"
  #define BROKER_COLLECTED_VALUES "loop"
  #define BROKER_MQTT_BUFFER_SIZE 1024
  #define BROKER_KEEP_ALIVE 90
  const String ACTION_TOPIC_PRAEFIX = String(STATION_ID) + "/action/";

  // System topics
  const String ACTION_TOPIC_OTA                 = String(ACTION_TOPIC_PRAEFIX) + "ota";
  const String ACTION_TOPIC_REBOOT              = String(ACTION_TOPIC_PRAEFIX) + "reboot";
  const String ACTION_TOPIC_BUFFERCLEAN         = String(ACTION_TOPIC_PRAEFIX) + "bufferclean";
  const String ACTION_TOPIC_CONFIGCLEAN         = String(ACTION_TOPIC_PRAEFIX) + "configclean";
  const String ACTION_TOPIC_STOP_PUBLISH_SENSOR = String(ACTION_TOPIC_PRAEFIX) + "stop_publish_sensor";
  const String ACTION_TOPIC_STOP_PUBLISH_DEBUG  = String(ACTION_TOPIC_PRAEFIX) + "stop_publish_debug";
  const String ACTION_TOPIC_STOP_PUBLISH_ERROR  = String(ACTION_TOPIC_PRAEFIX) + "stop_publish_error";
  const String ACTION_TOPIC_STOP_PUBLISH_BUFFER = String(ACTION_TOPIC_PRAEFIX) + "stop_publish_buffer";
  const String ACTION_TOPIC_STOP_SEND_WEEWX     = String(ACTION_TOPIC_PRAEFIX) + "stop_send_weewx";

    // Subscribed bits for brokerStatus
  #define BIT_BROKER_RECEIVED_OTA 0
  #define BIT_BROKER_RECEIVED_REBOOT 1
  #define BIT_BROKER_RECEIVED_BUFFERCLEAN 2
  #define BIT_BROKER_RECEIVED_CONFIGCLEAN 3
  #define BIT_BROKER_RECEIVED_STOP_PUBLISH_SENSOR 4
  #define BIT_BROKER_RECEIVED_STOP_PUBLISH_DEBUG 5
  #define BIT_BROKER_RECEIVED_STOP_PUBLISH_ERROR 6
  #define BIT_BROKER_RECEIVED_STOP_PUBLISH_BUFFER 7
  #define BIT_BROKER_RECEIVED_STOP_SEND_WEEWX 8
  unsigned int brokerStatus = 0;
  
  // Action/Status Bits for Weather Station
  #define BIT_ACTION_REQUIRED_OTA 0
  #define BIT_STATUS_ACTIVE_OTA 1
  #define BIT_ACTION_REQUIRED_REBOOT 2
  #define BIT_ACTION_REQUIRED_BUFFERCLEAN 3
  #define BIT_ACTION_REQUIRED_CONFIGCLEAN 4
  #define BIT_STATUS_STOPPED_PUBLISH_SENSOR 5
  #define BIT_STATUS_STOPPED_PUBLISH_DEBUG 6
  #define BIT_STATUS_STOPPED_PUBLISH_ERROR 7
  #define BIT_STATUS_STOPPED_PUBLISH_BUFFER 8
  #define BIT_STATUS_STOPPED_SEND_WEEWX 9
  #define BIT_STATUS_PUBLISHED_BUFFER 10
#endif

// Weatherunderground Protokoll to WeeWX
#define WEEWX_SERVER_PORT 8080
#define WEEWX_SERVER_PATH "/weatherstation/updateweatherstation.php"
#define WEEWX_SERVER_UPLOAD "GET /weatherstation/updateweatherstation.php"

// NTP Server
#define NTP_SERVER "192.168.0.254"

// System "sensor" Id
#define SYSTEM_SENSOR_ID "system"

// Voltage Sensor Id
#define VOLTAGE_SENSOR_ID "stationVoltage"

// invalid Temperature
#define INVALID_FLOAT_VALUE -9999.0

// W132 max offline
#define W132_MAX_OFFLINE 300 // 5 Minutes

// W174 max offline
#define W174_MAX_OFFLINE 300 // 5 Minutes

// W174 rain bucket size
#define W174_BUCKET_SIZE 0.25 // 0.25mm rain/count

// Sleep Mode hours
#define SLEEP_DAY_HOUR 5
#define SLEEP_NIGHT_HOUR 22
