//
//   Wetterstation
//   read values from different sensors and transmits them via mqtt to a broker and http to WeeWX
//
//   Hardware
//   WEMOS D1 Mini pro
//   https://www.reichelt.de/ersatz-windmesser-fuer-funkwetterstation-ventus-w132-p132595.html
//   https://www.reichelt.de/ersatz-regenmesser-fuer-ventus-funkwetterstation-ventus-w174-p132594.html
//
//   credits and inspirations
//   http://forum.arduino.cc/index.php?topic=136836.15 #19 code from user "jurs"
//   https://github.com/yu55/auriol_reader
//   http://www.tfd.hu/tfdhu/files/wsprotocol/auriol_protocol_v20.pdf
//   https://glsk.net/2018/02/battery-powered-weather-station-with-esp8266-and-bme280/
//   https://forum.iobroker.net/topic/23763/windanzeige-mit-ventus-w132-wemos-d1-mini
//   https://github.com/Zwer2k/WeatherStationDataRx
//   https://github.com/3KUdelta/Solar_WiFi_Weather_Station
//
//   Missing values calculating with the August-Roche-Magnus approximation:
//   http://bmcnoldy.rsmas.miami.edu/Humidity.html
//   RH: = 100*(EXP((17.625*TD)/(243.04+TD))/EXP((17.625*T)/(243.04+T)))
//   TD: = 243.04*(LN(RH/100)+((17.625*T)/(243.04+T)))/(17.625-LN(RH/100)-((17.625*T)/(243.04+T)))
//    T: = 243.04*(((17.625*TD)/(243.04+TD))-LN(RH/100))/(17.625+LN(RH/100)-((17.625*TD)/(243.04+TD)))
//   Example:
//   double a = 17.271;
//   double b = 237.7;
//   adjusted_temp = measured_temp + TEMP_CORR
//   adjusted_humi = 100 * (exp((a * DewpointTemperature) / (b + DewpointTemperature)) / exp((a * adjusted_temp) / (b + adjusted_temp)));
//
//   AMS AS3935:
//   https://bitbucket.org/christandlg/as3935mi/src/master/
//   Kaminari: https://github.com/shred/kaminari
//   connect the AS3935 to the Arduino like this:
//   PlayingwithFusion
//   AS3935   --------- Wemos D1 mini                        Kaminari
//   VCC      --------- 5V/3.3V                              5V/3.3V
//   GND      --------- GND                                  GND
//   CS       --------- 15 GPIO15 D8                         15 GPIO15 D8       
//   SI       --------- GND (set to SPI, VCC set to I2C)     GND
//   IRQ      --------- 5 GPIO5 D1 oder 2 GPIO2 D4           5 GPIO5 D1
//   SCK      --------- 14 GPIO14 D5                         14 GPIO14 D5
//   MISO     --------- 12 GPIO12 D6                         12 GPIO12 D6
//   MOSI     --------- 13 GPIO13 D7                         13 GPIO13 D7
//   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//   CJMCU-3935
//   EN_VREG  --------- 5V
//   other pins can be left unconnected.
//
#define WITH_DEBUG_TO_SERIAL 0 // 0 to disable debug output to serial
#define WITH_DEBUG_TO_BROKER 1 // 0 to disable debug output to broker
#define WITH_ERROR_TO_BROKER 1 // 0 to disable error output to broker
#define WITH_DEBUG_BROKER 1    // 0 to disable broker debug serial output 
#define WITH_DEBUG_SENSORS 1   // 0 to disable sensor debug serial output
#define WITH_CALC_TO_BROKER 1  // 0 to disable calulated values publishing

// Station altitude
#define STATION_ALTITUDE 394

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <TimeLib.h>             //https://github.com/PaulStoffregen/Time.git

//#include "ventus.h"   // Ventus Station
//#include "solar.h"   // Solar Station
#include "blitz.h"   // Blitz Station

#if (WITH_FLASH > 0)
  #include <FS.h>
#endif

#if (WITH_SD > 0)
  #include <SPI.h>
  #include <SdFat.h>
  SdFat sd;
#endif

#if (WITH_RTC > 0)
  #include <RTClib.h>
  RTC_DS1307 rtc;
#endif

#if WITH_DEBUG_TO_SERIAL > 0
  //#define SERIALSPEED 115200 // Set speed of serial in Arduino IDE
  #define SERIALSPEED 74880 // Set speed of serial in Arduino IDE
  #define debugStart(...) Serial.begin(__VA_ARGS__)   //debugStart is a macro, begin
  #define debug(...)      Serial.print(__VA_ARGS__)   //debug is a macro, debug print
  #define debugln(...)    Serial.println(__VA_ARGS__) //debugln is a macro, debug print print with LF
#else
  #define debugStart(...) //now defines a blank line
  #define debug(...)      //now defines a blank line
  #define debugln(...)    //now defines a blank line
#endif

//***************************************************************
// ESP8266 WiFiClient class
//***************************************************************
//WiFiClient WiFiClient;
// or... use WiFiFlientSecure for SSL
WiFiClient wifiClient;
WiFiClient weewxClient;

//***************************************************************
// ESP8266 RTC memory
//**************************************************************
// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct rtcDataType
{
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t bssid[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcData;

//***************************************************************
// OTA Update
//***************************************************************
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

//***************************************************************
// NTP Client class
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
//***************************************************************
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, 0, 60000);

//***************************************************************
// Id for System data
//***************************************************************
// System "sensor" Id
#define SYSTEM_SENSOR_ID "system"

//***************************************************************
// Broker configuration
//***************************************************************
#if (WITH_BROKER > 0)
  #include <PubSubClient.h>

  // Broker client class
  PubSubClient mqttclient(wifiClient);

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
  unsigned int stationActions = 0;
  
  int bufferedTopics = 0;
  int bufferedTopicsFailed = 0;
  int restoredTopics = 0;
  int restoredTopicsFailed = 0;
  int restoredTopicsBuffered = 0;
#endif

//***************************************************************
// Zambretti configuration
//***************************************************************
#if (WITH_ZAMBRETTI > 0)
  #define LANGUAGE 'DE'          // either 'DE' for German or 'EN' for English
  #include "Translation.h"
  
  // Zambretti "sensor" Id
  #define ZAMBRETTI_SENSOR_ID "zambretti"
  
  // Zambretti Data that we'll store on disk
  struct zambrettiDataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    unsigned long accuracy;
    int accuracy_in_percent;
    float savedBarometer[12];
    float diffBarometer[12];
    int barometer;
    String ZambrettisWords;             // Final statement about weather forecast
    String trend_in_words;              // Trend in words
    String forecast_in_words;           // Weather forecast in words
    String pressure_in_words;           // Air pressure in words
    String accuracy_in_words;           // Zambretti's prediction accuracy in words
  } zambrettiData;
#endif

//***************************************************************
// Config files
//***************************************************************
#define STATION_CONFIG_FILE "/station.cfg"
#define ZAMBRETTI_FILE "/zambretti.sav"
#define TOPIC_BUFFER_FILE "/topics.sav"
#define TOPIC_ERROR_FILE "/topics.err"

//***************************************************************
// Configuration that we'll store on disk
//***************************************************************
struct configDataType
{
  unsigned long actTimestamp;
} configData;

//***************************************************************
// Voltage Station Data
//***************************************************************
struct voltageDataType
{
  byte received;
  byte published;
  unsigned long measurementTimestamp;
  float stationVoltage;
} voltageData;

//***************************************************************
// General variables
//***************************************************************
float stationVoltage;    // actual battery voltage
unsigned long measurementTimestamp; //act. measuremt time for collected values
float pressureSealevel = 1013.25;
unsigned int loops;      // variable fuer loops
volatile unsigned long loopFirstStartTime;
volatile unsigned int actSleepSeconds;
volatile unsigned long realSleep; // calculated sleep time for deepsleep

//***************************************************************
// Sensor Ventus W132/W174
//***************************************************************
#define BIT_VENTUS_STATUS_W132_IS_STOPPED 0
#define BIT_VENTUS_STATUS_W132_TEMPHUMIDITY_RECEIVED 1
#define BIT_VENTUS_STATUS_W132_WINDSPEED_RECEIVED 2
#define BIT_VENTUS_STATUS_W132_WINDDIRGUST_RECEIVED 3
#define BIT_VENTUS_STATUS_W132_TEMPERATURE_RECEIVED 4
#define BIT_VENTUS_STATUS_W132_WIND_RECEIVED 5
//
#define BIT_VENTUS_STATUS_W174_IS_STOPPED 6
#define BIT_VENTUS_STATUS_W174_IS_REQUIRED 7
unsigned int ventusStatus = 0;

#if (WITH_W132 > 0 || WITH_W174 > 0)
  #define VENTUS_SYNC 9000   // length in µs of starting pulse
  #define VENTUS_ONE 4000    // length in µs of ONE pulse
  #define VENTUS_ZERO 2000   // length in µs of ZERO pulse
  #define VENTUS_TOL 400     // pulse length variation in µs for ONE and ZERO pulses
  #define VENTUS_DATALEN 32  // Ventus message data lenght
  #define VENTUS_CRCLEN 4    // Ventus message CRC lenght
  #define VENTUS_ERROR_CRC 1 // W132/W174 value CRC invalid
  unsigned int ventusMaxReceiveMilliSeconds = 60000; // max 1 minute receive time to get data
#endif

#if (WITH_W132 > 0)
  // forward function declarations
  ICACHE_RAM_ATTR void w132InterruptHandler();
  void w132fifoWrite(long item);

  // sensor connects
  #define W132_DATAPIN 14 // D5/GPIO14 connected data pin of W132 module

  // ventus sensor ids
  #define W132_SENSOR_ID "W132"

  // fifo Buffer size
  #define W132_FIFOSIZE 8 // fifo buffer size

  volatile long w132fifoBuf[W132_FIFOSIZE]; // ring buffer
  volatile byte w132fifoReadIndex, w132fifoWriteIndex;  // ring buffer read and write index W132
  
  volatile struct w132TempHumidityDataType
  {
    byte valid;
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    byte windBatteryStatus;
    byte trend;
    float temperature; //Celsius
    float tempf; //temperature as Fahrenheit
    byte humidity;

    float dewpoint; // Celsius
    float dewptf; // Dewpoint as Farenheit
    float spreadDewpoint; // Diff in Celsius
    float spreadDewptf; // Diff in Farenheit
    float heatindex; //Celsius
    float heatindexf; // Farenheit
    float windchill;  //Celsius
    float windchillf; // Farenheit
  } w132TempHumidityData;

  volatile struct w132WindSpeedDataType
  {
    byte valid;
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    byte windBatteryStatus;
    float windSpeed;
    float windSpeedKmh;
    
    float dewpoint; // Celsius
    float dewptf; // Dewpoint as Farenheit
    float spreadDewpoint; // Diff in Celsius
    float spreadDewptf; // Diff in Farenheit
    float heatindex; //Celsius
    float heatindexf; // Farenheit
    float windchill;  //Celsius
    float windchillf; // Farenheit
  } w132WindSpeedData;

  volatile struct w132WindDirGustDataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    byte windBatteryStatus;
    unsigned int windDir;
    char* windDirCardinalPoints;
    float windGust;
    float windGustKmh;
  } w132WindDirGustData;

  volatile unsigned long w132DataReceived;
  volatile unsigned long w132LastDataReceived;
#endif

#if (WITH_W174 > 0)
  // forward function declarations
  ICACHE_RAM_ATTR void w174InterruptHandler();
  void w174fifoWrite(long item);

  // sensor connects
  #define W174_DATAPIN 12 // D6/GPIO12 connected data pin of W174 module

  // ventus sensor ids
  #define W174_SENSOR_ID "W174"

  // fifo Buffer size
  #define W174_FIFOSIZE 8 // fifo buffer size

  volatile long w174fifoBuf[W174_FIFOSIZE]; // ring buffer
  volatile byte w174fifoReadIndex, w174fifoWriteIndex;  // ring buffer read and write index W174

  volatile struct w174DataType
  {
    byte valid;
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    byte rainBatteryStatus;
    float rain;
    unsigned long rainCounter;
    float rainTotal;
  } w174Data;

  volatile unsigned long w174DataReceived;
  volatile unsigned long w174LastDataReceived;
#endif

//***************************************************************
// Sensor AS3935
//***************************************************************
#if (WITH_AS3935MI > 0)
  #include <SPI.h>
  #include <AS3935SPI.h>

  #define AS3935MI_SENSOR_ID "AS3935MI"
  #define AS3935MI_PIN_IRQ 5
  #define AS3935MI_PIN_CS 15

  //create an AS3935 object using the SPI interface, chip select pin 15 and IRQ pin number 2
  AS3935SPI as3935mi(AS3935MI_PIN_CS, AS3935MI_PIN_IRQ);

  //this value will be set to true by the AS3935 interrupt service routine.
  volatile bool as3935miInterrupt = false;
  
  struct as3935miDataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    uint8_t event;
    int strike;
    int strike_sum;
    int disturber;
    int disturber_sum;
    int noise;
    int noise_sum;
    int unknown;
    int unknown_sum;
    float distance;
    float distance_sum;
    float energy;
    float energy_sum;
  } as3935miData;

  // cache AS3935 interrupt service routine
  ICACHE_RAM_ATTR void as3935miInterruptHandler();
#endif

//***************************************************************
// Timer Function
// Source: https://ullisroboterseite.de/esp8266-timing.html
//***************************************************************
#if (WITH_TIMER > 0)
  // Software Timer for other Sensors
  
  os_timer_t blitzTimer;     // Verwaltungsstruktur des Timers
  int Counter = 0;           // Argument für die Callback-Funktion
  bool timerActive = false;  // Flag, dass in der Callback-Funktion gesetzt wird
                             // Die aufwändigen und zeitintensiven Funktionen werden
                             // anhand dieses Flags im Hauptprogramm durchgeführt.
#endif

//***************************************************************
// Sensor TSL2591
//***************************************************************
#if (WITH_TSL2591 > 0)
  #define TSL2591_SENSOR_ID "TSL2591"
  #include <Adafruit_Sensor.h>
  #include "Adafruit_TSL2591.h"

  Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

  volatile struct tsl2591DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    float luminosity;
  } tsl2591Data;  
#endif

//***************************************************************
// Sensor BH1750
//***************************************************************
#if (WITH_BH1750 > 0)
  #define BH1750_SENSOR_ID "BH1750"
  #include <BH1750.h>  //https://github.com/claws/BH1750
  BH1750 LightSensor;
  String bh1750LevelSetup = "";
#endif

#if (WITH_BH1750B > 0)
  #define BH1750B_SENSOR_ID "BH1750B"
  //http://domoticx.com/arduino-lichtintensiteit-sensor-bh1750-gy-302/
  int BH1750_Device = 0x23; // I2C address for light sensor
  unsigned int luminosity, Scaled_FtCd;
  float FtCd, Wattsm2;
#endif

#if (WITH_BH1750AS > 0)
  #define BH1750AS_SENSOR_ID "BH1750AS"
  #include <AS_BH1750.h> //https://github.com/hexenmeister/AS_BH1750
  AS_BH1750 ASLightSensor;
#endif

#if (WITH_BH1750 > 0) || (WITH_BH1750B > 0) || (WITH_BH1750AS > 0)
  volatile struct bh1750DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    float luminosity;
    float fc;
    float radiation;
  } bh1750Data;
#endif

//***************************************************************
// Sensor BME280
//***************************************************************
#if WITH_BME280 > 0
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>

  #define BME280_SENSOR_ID "BME280"
  Adafruit_BME280 bme280;

  struct bme280DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    //messured:
    float temperature;
    byte humidity;
    float pressure;
    //calculated:
    float tempf; //temperature as Fahrenheit
    float absbaromin; // Pressure inch
    float baromin; // Barometer as inch
    float barometer; // Barometer hPa
    float dewpoint; // Celsius
    float dewptf; // Dewpoint as Farenheit
    float spreadDewpoint; // Diff in Celsius
    float spreadDewptf; // Diff in Farenheit
    float heatindex; //Celsius
    float heatindexf; // Farenheit
    float approxAltitude;
  } bme280Data;
#endif

//***************************************************************
// Sensor BME680
//***************************************************************
#if WITH_BME680 > 0
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME680.h>
  
  #define BME680_SENSOR_ID "BME680"
  Adafruit_BME680 bme680;

  struct bme680DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    //messured:
    float temperature;
    byte humidity;
    float pressure;
    float gas;
    //calculated:
    float tempf; //temperature as Fahrenheit
    float absbaromin; // Pressure inch
    float baromin; // Barometer as inch
    float barometer; // Barometer hPa
    float dewpoint; // Celsius
    float dewptf; // Dewpoint as Farenheit
    float spreadDewpoint; // Diff in Celsius
    float spreadDewptf; // Diff in Farenheit
    float heatindex; //Celsius
    float heatindexf; // Farenheit
    float approxAltitude;
  } bme680Data;
#endif

//***************************************************************
// Sensor MHRD
//***************************************************************
#if WITH_MHRD > 0
  #define MHRD_SENSOR_ID "MH-RD"

  struct mhrdDataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    float rainState;
  } mhrdData;
#endif

//***************************************************************
// Sensor ML8511
//***************************************************************
#if WITH_ML8511 > 0
  #define ML8511_SENSOR_ID "ML8511"
  #define ML8511_DATAPIN 17 //A0

  struct ml8511DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    float uvIntensity;
  } ml8511Data;
#endif

//***************************************************************
// Sensor VEML6075
//***************************************************************
#if WITH_VEML6075 > 0
  #define VEML6075_SENSOR_ID "VEML6075"
  #include <Adafruit_VEML6075.h>
  Adafruit_VEML6075 uv6075 = Adafruit_VEML6075();

  struct veml6075DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    byte UV;
    byte UVA;
    byte UVB;
  } veml6075Data;
#endif

//***************************************************************
// Sensor VEML6070
//***************************************************************
#if WITH_VEML6070 > 0
  #define VEML6070_SENSOR_ID "VEML6070"
  #include <Adafruit_VEML6070.h>
  Adafruit_VEML6070 uv6070 = Adafruit_VEML6070();

  struct veml6070DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    byte UV;
    byte uvLevel;
    char *uvRisk;
  } veml6070;  
#endif

//***************************************************************
// Sensor DS18B20
//***************************************************************
#if (WITH_DS18B20 > 0)
  #include <OneWire.h>
  #include <DallasTemperature.h>
  
  #define ONE_WIRE_BUS D7
  #define DS18B20_SENSOR_ID "DS18B20"
  
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);

  // Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature sensorDS18B20(&oneWire);

  struct ds18b20DataType
  {
    byte received;
    byte published;
    unsigned long measurementTimestamp;
    float temperature;
  } ds18b20Data;
#endif

//***************************************************************
//***************************************************************
// Weather Station Funtions
//***************************************************************
//***************************************************************

//***************************************************************
//Forward function definitions
//***************************************************************
boolean brokerPublish(String topic, String payload, boolean retained);
boolean errorPublish(String sensorId, String errorMessage, boolean retained);

//***************************************************************
//Calculate Time Differences
//***************************************************************
inline static unsigned long timeDifference(unsigned long now, unsigned long past)
{
    // This is actually safe from millis() overflow because all types are unsigned long!
    return past > 0 ? (now - past) : 0;
}

//***************************************************************
//  Calculates the altitude (in meters) from the specified atmospheric
//  pressure (in hPa), and sea-level pressure (in hPa)
//  with Temperature Compensation.
//  (see https://keisan.casio.com/exec/system/1224585971)
//  
//  NOTE:
//  If the altitude is more than 11km high above sea level, 
//  the hypsometric formula cannot be applied because the temperature 
//  lapse rate varies considerably with altitude.
//    @param  seaLevel      Sealevel pressure in hPa
//    @param  atmospheric   Atmospheric pressure in hPa
//    @param  temperature   temperature in Celsius
//***************************************************************
#if (WITH_BME680 > 0)
float bme680ReadAltitude(float seaLevel, float pressure, float temperature)
{
  // Equation taken from https://keisan.casio.com/exec/system/1224585971
  return ( (pow(seaLevel / pressure, 0.1902225) - 1.0) * (temperature + 273.15) ) / 0.0065;
}
#endif

//***************************************************************
// Calculate relative pressure
//***************************************************************
float getBarometer(float pressure)
{
  float SLpressure_hPa = (((pressure * 100.0)/pow((1-((float)(STATION_ALTITUDE))/44330), 5.255))/100.0);
  return ((int)(SLpressure_hPa+.5));
}

//***************************************************************
// Calculate HI (heatindex in °C)
// --> HI starts working above 26,7 °C
//***************************************************************
float getHeatIndex(float temp, float humidity)
{
  if (temp > 26.7)
  {
    double c1 = -8.784, c2 = 1.611, c3 = 2.338, c4 = -0.146, c5= -1.230e-2, c6=-1.642e-2, c7=2.211e-3, c8=7.254e-4, c9=-2.582e-6;
    double T = temp;
    double R = humidity;
  
    double A = (( c5 * T) + c2) * T + c1;
    double B = ((c7 * T) + c4) * T + c3;
    double C = ((c9 * T) + c8) * T + c6;

    return ((C * R + B) * R + A); 
  } 
  else
  {
    return temp;
  }
}

//***************************************************************
// Calculate dewpoint
//***************************************************************
float getDewPoint(float temp, float humidity)
{
  double a = 17.271;
  double b = 237.7;
  double tempcalc = (a * temp) / (b + temp) + log(humidity*0.01);
  return ((b * tempcalc) / (a - tempcalc));
}

//***************************************************************
// Calculate windchill
//***************************************************************
float getWindChill(float temp, float windspeed)
{
  if ((windspeed / 3.6) > 1.4)
  {
    return (13.12 + (0.6215 * temp) - (11.37 * pow(windspeed, 0.16)) + (0.3965 * temp * pow(windspeed, 0.16)));
  }
  else
  {
    return temp;
  }
}

//***************************************************************
// Returns the broker connection state
//***************************************************************
boolean brokerIsConnected()
{
  #if (WITH_BROKER > 0)
    return mqttclient.connected();
  #endif

  return false;
}

//***************************************************************
// Returns the wifi state
//***************************************************************
boolean wifiIsConnected()
{
  return (WiFi.status() == WL_CONNECTED);
}

//***************************************************************
// Returns actually timestamp
//***************************************************************
unsigned long getTimestamp()
{
  if (!wifiIsConnected())
  {
    #if (WITH_RTC > 0)
      debugln("Get time RTC DS1307.");
      DateTime now = rtc.now();
      configData.actTimestamp = now.unixtime();

      debug("actual timestamp (RTC): ");
      debugln(configData.actTimestamp);
        
      return configData.actTimestamp;
    #else
      debugln("RTC DS1307 not available, get time from config file.");
      debug("actual timestamp (fallback): ");
      debugln(configData.actTimestamp);

      return ++configData.actTimestamp; // only fallback
    #endif
  }
  else
  {
    debugln("Get time from NTP Server.");
    timeClient.update();
    configData.actTimestamp = timeClient.getEpochTime();

    debug("actual timestamp (NTP): ");
    debugln(configData.actTimestamp);
    
    return configData.actTimestamp;
  }
}

//***************************************************************
// Buffer Topic values to flash or SD
//***************************************************************
boolean brokerPublishToBuffer(String topic, String payload)
{
  #if (WITH_BROKER > 0)
    if (bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_BUFFER) > 0)
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Buffering topics is disabled!");
      #endif
      return true;
    }
    
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debugln("Broker message save to file...");
    #endif
        
    // Open for appending (writing at end of file)
    #if (WITH_FLASH > 0)
      File file = SPIFFS.open(TOPIC_BUFFER_FILE, "a");
    #elif (WITH_SD > 0)
      File file = sd.open(TOPIC_BUFFER_FILE, FILE_WRITE);
    #else
      debugln("SD/FLASH not configured!");
      bufferedTopicsFailed++;
      return false;
    #endif
    
    if (!file)
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Failed to open topics file!");
      #endif
      bufferedTopicsFailed++;
      return false;
    }
  
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;
  
    jsonBuffer["topic"] = topic;
    jsonBuffer["payload"] = payload;
  
    char saveJson[1024];
    serializeJson(jsonBuffer, saveJson);
  
    jsonBuffer.clear();
  
    if(file.println(saveJson))
    {
      file.close();
      bufferedTopics++;
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Broker message saved to file.");
      #endif
      bitClear(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
      return true;
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Broker message could not be saved!");
      #endif
      file.close();
      bufferedTopicsFailed++;
      return false;
    }
  #endif

  return false;
}

//***************************************************************
// Publish or buffer topic values
//***************************************************************
boolean brokerPublish(String topic, String payload, boolean retained=false)
{
  #if (WITH_BROKER > 0)
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debug("Broker topic: ");
      debugln(topic);
      debug("Broker payload: ");
      debugln(payload);
    #endif
  
    if(brokerIsConnected())
    {
      const char* publishTopic = topic.c_str();
      const char* publishPayload = payload.c_str();
  
      debugln("Topic and Payload converted.");
      
      if(mqttclient.publish(publishTopic, publishPayload, retained))
      {
        mqttclient.loop();
        #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
          debugln("Broker topic published.");
          debugln();
        #endif
        delay(50);
        return true;
      }
      else
      {
        #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
          debugln("Broker topic publishing failed!");
        #endif
        // if not ready to publish or publishing failed
        return brokerPublishToBuffer(topic, payload);
      }
    }
    else
    {
      // if not ready to publish or publishing failed
      return brokerPublishToBuffer(topic, payload);
    }
  #endif

  return false;
}

//***************************************************************
// Error publishing
//***************************************************************
boolean errorPublish(String sensorId, String errorMessage, boolean retained=false)
{
  #if (WITH_BROKER > 0)
    if (WITH_ERROR_TO_BROKER == 0 || bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_ERROR) > 0)
    {
      return true;
    }
    
    // Broker topic
    String errorTopic = String(STATION_ID) + "/error/" + String(BROKER_OUTPUTFORMAT);
  
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;
    
    jsonBuffer["dateTime"] = getTimestamp();
    jsonBuffer["type"] = "ERROR";
    jsonBuffer["sensor"] = sensorId;
    jsonBuffer["message"] = errorMessage;
    
    char errorPayload[1024];
    serializeJson(jsonBuffer, errorPayload);
    
    return brokerPublish(errorTopic, errorPayload, retained);
  #endif

  return false;
}

//***************************************************************
// Reads the configuration file
//***************************************************************
boolean readConfiguration()
{
  debugln("********************* CONFIG READ **************************");

  // W174 Data
  #if (WITH_W174 > 0)
    w174Data.valid = 0;
  #endif

  // W132 Data
  #if (WITH_W132 > 0)
    w132TempHumidityData.valid = 0;
    w132WindSpeedData.valid = 0;
  #endif

  // Config Buffer if required

  #if (WITH_BROKER > 0)
    debug("Clean Config: ");
    debugln(bitRead(stationActions, BIT_ACTION_REQUIRED_CONFIGCLEAN));
    
    if (bitRead(stationActions, BIT_ACTION_REQUIRED_CONFIGCLEAN) == 1)
    {
      debugln("Cleaning buffered config.");
      #if (WITH_FLASH > 0)
        SPIFFS.remove(STATION_CONFIG_FILE);
        //SPIFFS.format();
      #elif (WITH_SD > 0)
        sd.remove(STATION_CONFIG_FILE);
      #endif
      bitClear(stationActions, BIT_ACTION_REQUIRED_CONFIGCLEAN);
      brokerPublish(String(ACTION_TOPIC_CONFIGCLEAN), "0", true);
      debugln("************************************************************");
      debugln();
      return true;
    }
  #endif
        
  // Open file for reading
  #if (WITH_FLASH > 0)
    debugln("Load config file from SPIFFS (Flash) ...");
    File file = SPIFFS.open(STATION_CONFIG_FILE, "r");
  #elif (WITH_SD > 0) // Open file for SD Card reading
    debugln("Load config file from SD Card ...");
    File file = sd.open(STATION_CONFIG_FILE, FILE_READ);
  #else
    debugln("************************************************************");
    debugln();
    return false;
  #endif

  if (!file)
  {
    file.close();
    debug("Failed to open config file: ");
    debugln(STATION_CONFIG_FILE);
    errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open config file!", false);
    debugln("************************************************************");
    debugln();
    return false;
  }

  unsigned long actTime = getTimestamp();
  // Create JsonBuffer
  StaticJsonDocument<1024> jsonBuffer;
/*
  char lineBuffer[1024];
  int l = 0;
  
  if (file.available())
  {
    debug("File not avaiable, file: ");
    debugln(String(STATION_CONFIG_FILE));
    file.close();
    debugln("************************************************************");
    debugln();
    return false;
  }
    
  l = file.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer));
  file.close();
  
  lineBuffer[l] = 0;
  debug("Buffer: ");
  debugln(lineBuffer);
*/

  // Deserialize the JSON document
  // DeserializationError error = deserializeJson(jsonBuffer, lineBuffer);
  DeserializationError error = deserializeJson(jsonBuffer, file);
  file.close();
  
  if (error)
  {
    debug("DeserializationError file: ");
    debugln(String(STATION_CONFIG_FILE));
    debugln("************************************************************");
    debugln();
    return false;
  }

  configData.actTimestamp = (int)jsonBuffer["actTimestamp"];
  debug("Timestamp.............................: ");
  debugln(configData.actTimestamp);

  //stationActions = (int)jsonBuffer["stationActions"];
  //debug("stationActions........................: ");
  //debugln(stationActions);

  #if (WITH_W174 > 0)
    w174Data.measurementTimestamp = (int)jsonBuffer["w174Data.measurementTimestamp"];
    debug("W174 measurementTimestamp.............: ");
    debugln(w174Data.measurementTimestamp);

    w174Data.rainCounter = (int)jsonBuffer["w174Data.rainCounter"];
    w174Data.rain = 0;
    w174Data.rainTotal = w174Data.rainCounter * W174_BUCKET_SIZE;
    debug("W174 rainCounter......................: ");
    debugln(w174Data.rainCounter);

    w174Data.rainBatteryStatus = (int)jsonBuffer["w174Data.rainBatteryStatus"];
    debug("W174 rainBatteryStatus................: ");
    debugln(w174Data.rainBatteryStatus);

    w174Data.valid = 1;
  #endif

  #if (WITH_W132 > 0)
    w132TempHumidityData.measurementTimestamp = (int)jsonBuffer["w132TempHumidityData.measurementTimestamp"];
    debug("W132 TempHumidity measurementTimestamp: ");
    debugln(w132TempHumidityData.measurementTimestamp);

    w132TempHumidityData.temperature = (float)jsonBuffer["w132TempHumidityData.temperature"];
    debug("W132 TempHumidity temperature.........: ");
    debugln(w132TempHumidityData.temperature);

    w132TempHumidityData.humidity = (int)jsonBuffer["w132TempHumidityData.humidity"];
    debug("W132 TempHumidity humidity............: ");
    debugln(w132TempHumidityData.humidity);

    if (actTime - w132TempHumidityData.measurementTimestamp <= W132_MAX_OFFLINE)
    {
      w132TempHumidityData.valid = 1;
    }

    w132WindSpeedData.measurementTimestamp = (int)jsonBuffer["w132WindSpeedData.measurementTimestamp"];
    debug("W132 WindSpeed measurementTimestamp...: ");
    debugln(w132WindSpeedData.measurementTimestamp);

    w132WindSpeedData.windSpeedKmh = (float)jsonBuffer["w132TempHumidityData.windSpeedKmh"];
    debug("W132 WindSpeed windSpeedKmh...........: ");
    debugln(w132WindSpeedData.windSpeedKmh);

    if (actTime - w132WindSpeedData.measurementTimestamp <= W132_MAX_OFFLINE)
    {
      w132WindSpeedData.valid = 1;
    }    

    debug("Last W174 RainData valid..............: ");
    debugln(w174Data.valid);

    debug("Last W132 TempHumidityData valid......: ");
    debugln(w132TempHumidityData.valid);

    debug("Last W132 WindSpeedData valid.........: ");
    debugln(w132WindSpeedData.valid);
  #endif

  debugln("ConfigData file loaded.");
  debugln("************************************************************");
  debugln();

  return true;
  
/*
  // last actual timestamp
  String temp_data = file.readStringUntil('\n');  
  configData.actTimestamp = temp_data.toInt();
  debug("Timestamp.............................: ");
  debugln(configData.actTimestamp);

  #if (WITH_W174 > 0)
    // last W174 measurement timestamp
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w174Data.measurementTimestamp = temp_data.toInt();
    debug("W174 measurementTimestamp.............: ");
    debugln(w174Data.measurementTimestamp);

    // last W174 rain counter
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w174Data.rainCounter = temp_data.toInt();
    w174Data.rain = 0;
    w174Data.rainTotal = w174Data.rainCounter * W174_BUCKET_SIZE;
    debug("W174 rainCounter......................: ");
    debugln(w174Data.rainCounter);

    // last W174 Battery status
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w174Data.rainBatteryStatus = temp_data.toInt();
    debug("W174 rainBatteryStatus................: ");
    debugln(w174Data.rainBatteryStatus);

    w174Data.valid = 1;
  #endif

  #if (WITH_W132 > 0)
    // last W132 temperature measurement timestamp
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w132TempHumidityData.measurementTimestamp = temp_data.toInt();
    debug("W132 TempHumidity measurementTimestamp: ");
    debugln(w132TempHumidityData.measurementTimestamp);

    // last W132 last temperature
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w132TempHumidityData.temperature = temp_data.toFloat();
    debug("W132 TempHumidity temperature.........: ");
    debugln(w132TempHumidityData.temperature);

    // last W132 last humidity
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w132TempHumidityData.humidity = temp_data.toInt();
    debug("W132 TempHumidity humidity............: ");
    debugln(w132TempHumidityData.humidity);

    if (actTime - w132TempHumidityData.measurementTimestamp <= W132_MAX_OFFLINE)
    {
      w132TempHumidityData.valid = 1;
    }

    // last W132 windSpeed measurement timestamp
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w132WindSpeedData.measurementTimestamp = temp_data.toInt();
    debug("W132 WindSpeed measurementTimestamp...: ");
    debugln(w132WindSpeedData.measurementTimestamp);

    // last W132 last windSpeed
    temp_data = "";
    temp_data = file.readStringUntil('\n');  
    w132WindSpeedData.windSpeedKmh = temp_data.toFloat();
    debug("W132 WindSpeed windSpeedKmh...........: ");
    debugln(w132WindSpeedData.windSpeedKmh);

    if (actTime - w132WindSpeedData.measurementTimestamp <= W132_MAX_OFFLINE)
    {
      w132WindSpeedData.valid = 1;
    }    

    debug("Last W132 TempHumidityData valid......: ");
    debugln(w132TempHumidityData.valid);

    debug("Last W132 WindSpeedData valid.........: ");
    debugln(w132WindSpeedData.valid);

  #endif
  
  debugln("************************************************************");
  
  // Close the file (File's destructor doesn't close the file)
  file.close();
  debugln("ConfigData file loaded.");
  debugln();

*/
}

//***************************************************************
// Write the configuration file
//***************************************************************
bool writeConfiguration()
{
  debugln("********************* CONFIG WRITE **************************");


  #if (WITH_FLASH > 0)
    SPIFFS.remove(STATION_CONFIG_FILE);
    File file = SPIFFS.open(STATION_CONFIG_FILE, "w");
    if (!file)
    {
      debugln("Failed to open config file for writing!");
      errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open config file for writing!", false);
      debugln("*************************************************************");
      debugln();
      return false;
    }
  #elif (WITH_SD > 0)
    sd.remove(STATION_CONFIG_FILE);
    File file = sd.open(STATION_CONFIG_FILE, O_WRONLY | O_CREAT | O_TRUNC);
    if (!file)
    {
      debugln("Failed to open config file for writing!");
      errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open config file for writing!", false);
      debugln("*************************************************************");
      debugln();
      return false;
    }
  #else
    debugln("SD/FLASH not configured!");
    errorPublish(String(SYSTEM_SENSOR_ID), "SD/FLASH not configured!", false);
    debugln("*************************************************************");
    debugln();
    return false;
  #endif

  debugln("Write data to config file.");

    // Create JsonBuffer
  StaticJsonDocument<1024> jsonBuffer;
  unsigned long actTime = getTimestamp();

  debug("Timestamp.............................: ");
  debugln(actTime);
  jsonBuffer["actTimestamp"] = actTime;

  //debug("stationActions........................: ");
  //debugln(stationActions);
  //jsonBuffer["stationActions"] = stationActions;

  #if (WITH_W174 > 0)
    debug("W174 measurementTimestamp.............: ");
    debugln(w174Data.measurementTimestamp);
    jsonBuffer["w174Data.measurementTimestamp"] = w174Data.measurementTimestamp;
    
    debug("W174 rainCounter......................: ");
    debugln(w174Data.rainCounter);
    jsonBuffer["w174Data.rainCounter"] = w174Data.rainCounter;
    
    debug("W174 rainBatteryStatus................: ");
    debugln(w174Data.rainBatteryStatus);
    jsonBuffer["w174Data.rainBatteryStatus"] = w174Data.rainBatteryStatus;
  #endif

  #if (WITH_W132 > 0)
    //if (w132TempHumidityData.valid > 0)
    {
      debug("W132 TempHumidity measurementTimestamp: ");
      debugln(w132TempHumidityData.measurementTimestamp);
      jsonBuffer["w132TempHumidityData.measurementTimestamp"] = w132TempHumidityData.measurementTimestamp;
      
      debug("W132 TempHumidity temperature.........: ");
      debugln(w132TempHumidityData.temperature);
      jsonBuffer["w132TempHumidityData.temperature"] = w132TempHumidityData.temperature;
      
      debug("W132 TempHumidity humidity............: ");
      debugln(w132TempHumidityData.humidity);
      jsonBuffer["w132TempHumidityData.humidity"] = w132TempHumidityData.humidity;
    }

    //if (w132WindSpeedData.valid > 0)
    {
      debug("W132 WindSpeed measurementTimestamp...: ");
      debugln(w132WindSpeedData.measurementTimestamp);
      jsonBuffer["w132WindSpeedData.measurementTimestamp"] = w132WindSpeedData.measurementTimestamp;
      
      debug("W132 WindSpeed windSpeedKmh...........: ");
      debugln(w132WindSpeedData.windSpeedKmh);
      jsonBuffer["w132WindSpeedData.windSpeedKmh"] = w132WindSpeedData.windSpeedKmh;
    }

    debug("Last W74 RainData valid...............: ");
    debugln(w174Data.valid);

    debug("Last W132 TempHumidityData valid......: ");
    debugln(w132TempHumidityData.valid);

    debug("Last W132 WindSpeedData valid.........: ");
    debugln(w132WindSpeedData.valid);
  #endif

  //char saveJson[1024];
  //serializeJson(jsonBuffer, saveJson);

  if (serializeJson(jsonBuffer, file) == 0)
  {
    debug("Config could not be saved to file: ");
    debugln(String(STATION_CONFIG_FILE));
    debugln("*************************************************************");
    debugln();
    return false;
  }
  else
  {
    file.close();
    debug("Config saved to file: ");
    debugln(String(STATION_CONFIG_FILE));
    debugln("*************************************************************");
    debugln();
    return true;    
  }

  /*
  if(file.println(saveJson))
  {
    file.println("\n");
    file.close();
    debug("Config saved to file: ");
    debugln(String(STATION_CONFIG_FILE));
    debugln("*************************************************************");
    debugln();
    return true;
  }
  else
  {
    file.close();
    debug("Config could not be saved to file: ");
    debugln(String(STATION_CONFIG_FILE));
    debugln("*************************************************************");
    debugln();
    return false;
  }  
*/

/*  
  debug("Timestamp.............................: ");
  debugln(actTime);
  file.println(actTime);

  #if (WITH_W174 > 0)
    debug("W174 measurementTimestamp.............: ");
    debugln(w174Data.measurementTimestamp);
    debug("W174 rainCounter......................: ");
    debugln(w174Data.rainCounter);
    debug("W174 rainBatteryStatus................: ");
    debugln(w174Data.rainBatteryStatus);

    file.println(w174Data.measurementTimestamp);
    file.println(w174Data.rainCounter);
    file.println(w174Data.rainBatteryStatus);
  #endif

  #if (WITH_W132 > 0)
    //if (w132TempHumidityData.valid > 0)
    {
      debug("W132 TempHumidity measurementTimestamp: ");
      debugln(w132TempHumidityData.measurementTimestamp);
      debug("W132 TempHumidity temperature.........: ");
      debugln(w132TempHumidityData.temperature);
      debug("W132 TempHumidity humidity............: ");
      debugln(w132TempHumidityData.humidity);

      file.println(w132TempHumidityData.measurementTimestamp);
      file.println(w132TempHumidityData.temperature);
      file.println(w132TempHumidityData.humidity);
    }

    //if (w132WindSpeedData.valid > 0)
    {
      debug("W132 WindSpeed measurementTimestamp...: ");
      debugln(w132WindSpeedData.measurementTimestamp);
      debug("W132 WindSpeed windSpeedKmh...........: ");
      debugln(w132WindSpeedData.windSpeedKmh);

      file.println(w132WindSpeedData.measurementTimestamp);
      file.println(w132WindSpeedData.windSpeedKmh);
    }

    debug("Last W132 TempHumidityData valid......: ");
    debugln(w132TempHumidityData.valid);

    debug("Last W132 WindSpeedData valid.........: ");
    debugln(w132WindSpeedData.valid);
    
  #endif

  file.close();

  debugln("*************************************************************");

  //debugln("ConfigData saved to file.");
  debugln();

  return true;
*/
}

//***************************************************************
// Prints the content of a file to serial
//***************************************************************
boolean printFile(String filename)
{
  #if (WITH_DEBUG_TO_SERIAL > 0)
    debugln();
    debugln("************************* PRINT FILE *******************************");
    debug("Print File: ");
    debugln(filename);
    debugln();

    // Open file for reading
    #if (WITH_FLASH > 0)
      File file = SPIFFS.open(filename, "r");
    #elif (WITH_SD > 0)
      File file = sd.open(filename, FILE_READ);
    #else
      debugln("********************************************************************");
      debugln();
      return false;
    #endif
    
    if (!file)
    {
      debugln("Failed to read file!");
      debugln("********************************************************************");
      debugln();
      return false;
    }

    /*
    char lineBuffer[1024];
    int i = 0;
    while (file.available())
    {
      int l = file.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer));
      lineBuffer[l] = 0;

      i++;
      debug("Saved Line ");
      debug(i);
      debug(": ");
      debugln(lineBuffer);
    }
    
    // Close the file (File's destructor doesn't close the file)
    file.close();
    debugln();
    */

    // Extract each characters by one by one
    while (file.available())
    {
      debug((char)file.read());
    }
    debugln();

    // Close the file
    file.close();
    debugln("********************************************************************");
    debugln();

  #endif

  return true;
}

//***************************************************************
// Reads battery voltage
//***************************************************************
float batteryVoltage()
{
  #if (WITH_VOLTAGE > 0)
    // Voltage divider R1 = 220k+100k+220k =540k and R2=100k
    float volt = (analogRead(A0) * (VOLTAGE_CALIB_FACTOR/1024));
    debug("Voltage: ");
    debug(volt, 2); // print with 2 decimal places
    debugln(" V");
    return volt; 
  #else
    return 0.0;
  #endif  
}

//***************************************************************
// calculate CRC rtcData
//***************************************************************
uint32_t calculateCRC32( const uint8_t *data, size_t length )
{
  uint32_t crc = 0xffffffff;
  while( length-- )
  {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 )
    {
      bool bit = crc & 0x80000000;
      if( c & i )
      {
        bit = !bit;
      }

      crc <<= 1;
      if( bit )
      {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}

//***************************************************************
// Connect to WiFi access point.
//***************************************************************
boolean wifiConnect()
{
  // Try to read WiFi settings from RTC memory
  bool rtcValid = false;
  if (ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcData, sizeof(rtcData)))
  {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if( crc == rtcData.crc32 )
    {
      rtcValid = true;
    }
  }

  WiFi.forceSleepWake();
  delay(1);
  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.hostname(STATION_ID);
  if (rtcValid)
  {
    debugln("Saved WiFi connection is true.");
    // The RTC data was good, make a quick connection
    WiFi.begin( WLAN_SSID, WLAN_PASSWORD, rtcData.channel, rtcData.bssid, true );
  }
  else 
  {
    debugln("Saved WiFi connection is false.");
    // The RTC data was not valid, so make a regular connection
    WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  }

  unsigned long startTime = millis();

  debug("Attempting WiFi connection SSID ");
  debug(WLAN_SSID);
  debug(" ");

  while (!wifiIsConnected() && millis() - startTime < 10000)
  {
    delay(500);
    debug(".");
  }

  if(!wifiIsConnected())
  {
    // give up, disconnect WiFi
    debugln();
    debugln("WiFi not connected, give up!");
    debugln();

    // Write current connection info back to RTC
    rtcData.crc32 = 0; //set CRC to false
  }
  else
  {
    debugln();
    debug("WiFi connected. ");
    debug("IP address: ");
    debugln(WiFi.localIP());
    debugln();
    
    // Write current connection info back to RTC
    rtcData.channel = WiFi.channel();
    memcpy(rtcData.bssid, WiFi.BSSID(), 6); // Copy 6 bytes of BSSID (AP's MAC address)
    rtcData.crc32 = calculateCRC32(((uint8_t*)&rtcData) + 4, sizeof(rtcData) - 4);
  }

  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));
 
  return wifiIsConnected();
}
  
//***************************************************************
// Disconnect WiFi access point.
//***************************************************************
void wifiDisconnect()
{
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_OFF);
  debugln("WiFi disconnected.");
}

//***************************************************************
// Debug publishing
//***************************************************************
boolean debugPublish(String sensorId, String debugMessage, boolean retained=false)
{
  #if (WITH_BROKER > 0)
    if (WITH_DEBUG_TO_BROKER == 0 || bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_DEBUG) > 0)
    {
      return true;
    }
  
    // Broker topic
    String debugTopic = String(STATION_ID) + "/debug/" + String(BROKER_OUTPUTFORMAT);
  
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;
    
    jsonBuffer["dateTime"] = getTimestamp();
    jsonBuffer["type"] = "DEBUG";
    jsonBuffer["sensor"] = sensorId;
    jsonBuffer["message"] = debugMessage;
    
    char debugPayload[1024];
    serializeJson(jsonBuffer, debugPayload);
  
    return brokerPublish(debugTopic, debugPayload, retained);
  #endif

  return false;
}

//***************************************************************
// Sensor publishing
//***************************************************************
boolean sensorPublish(String sensorId, String payload, boolean retained)
{
  #if (WITH_BROKER > 0)
    if (bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_SENSOR) > 0)
    {
      return true;
    }
  
    String sensorTopic = String(STATION_ID) + "/" + sensorId + "/" + String(BROKER_OUTPUTFORMAT);
    
    return brokerPublish(sensorTopic, payload, retained);
  #endif

  return false;
}

//***************************************************************
// Subscribe topics
//***************************************************************
void brokerSubscribe()
{
  #if (WITH_BROKER > 0)
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debugln();
      debugln("Broker subscribe Topics...");
    #endif
    
    if (!brokerIsConnected())
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Broker is not connected!");
        debugln();
      #endif
      return;
    }
    
    // OTA Broker topic
    String topic = String(ACTION_TOPIC_OTA);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Reboot topic
    topic = String(ACTION_TOPIC_REBOOT);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Buffer clean topic
    topic = String(ACTION_TOPIC_BUFFERCLEAN);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Config clean topic
    topic = String(ACTION_TOPIC_CONFIGCLEAN);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Stop publish sensor topic
    topic = String(ACTION_TOPIC_STOP_PUBLISH_SENSOR);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Stop publish debug topic
    topic = String(ACTION_TOPIC_STOP_PUBLISH_DEBUG);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Stop publish error topic
    topic = String(ACTION_TOPIC_STOP_PUBLISH_ERROR);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Stop publish buffer topic
    topic = String(ACTION_TOPIC_STOP_PUBLISH_BUFFER);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    // Stop sending to weewx topic
    topic = String(ACTION_TOPIC_STOP_SEND_WEEWX);
    
    if (mqttclient.subscribe(topic.c_str()))
    {
      mqttclient.loop();
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribed ");
        debug(topic);
        debugln(" successfully.");
      #endif
    }
    else
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker subscribe ");
        debug(topic);
        debugln(" failed!");
      #endif
    }
  
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debugln();
    #endif
  #endif
}

//***************************************************************
// Publish buffered topics
//***************************************************************
boolean brokerPublishBuffer()
{
  #if (WITH_BROKER > 0)
    // clean Buffer if required
    if (bitRead(stationActions, BIT_ACTION_REQUIRED_BUFFERCLEAN) == 1)
    {
      debugln("Cleaning buffered messages.");
      #if (WITH_FLASH > 0)
        SPIFFS.remove(TOPIC_BUFFER_FILE);
      #elif (WITH_SD > 0)
        sd.remove(TOPIC_BUFFER_FILE);
      #endif
      bitClear(stationActions, BIT_ACTION_REQUIRED_BUFFERCLEAN);
      String stationTopic = String(ACTION_TOPIC_BUFFERCLEAN);
      brokerPublish(stationTopic, "0", true);
    }
  
    if (bitRead(brokerStatus, BIT_STATUS_STOPPED_PUBLISH_BUFFER) == 1)
    {
      return true;
    }
  
    if (bitRead(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER) == 1)
    {
      return true;
    }

    if(!brokerIsConnected())
    {
      return false;
    }

    bufferedTopics = 0;
    restoredTopics = 0;
    restoredTopicsFailed = 0;
    restoredTopicsBuffered = 0;

    #if (WITH_FLASH > 0)
      if (!SPIFFS.exists(TOPIC_BUFFER_FILE))
      {
        bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
        return true;
      }
    #elif (WITH_SD > 0)
      if (!sd.exists(TOPIC_BUFFER_FILE))
      {
        bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
        return true;
      }
    #else
      debugln("SD/FLASH not configured!");
      errorPublish(String(SYSTEM_SENSOR_ID), "SD/FLASH not configured!", false);
      bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
      return false;
    #endif
    
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debugln("Broker publish buffered messages...");
    #endif
  
    // Open file for reading
    #if (WITH_FLASH > 0)
      File file = SPIFFS.open(TOPIC_BUFFER_FILE, "r");
    #elif (WITH_SD > 0)
      File file = sd.open(TOPIC_BUFFER_FILE, FILE_READ);
    #endif
    
    if (!file)
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Failed to open saved topics file!");
        debugln();
      #endif
      bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
      return false;
    }

    char lineBuffer[1024];
    while (file.available())
    {
      int l = file.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer));
      lineBuffer[l] = 0;
      bufferedTopics++;
    }
      
    // Close the file (File's destructor doesn't close the file)
    file.close();
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debug("Buffered Topics: ");
      debugln(bufferedTopics);
    #endif

    if (bufferedTopics < 1)
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("No topics saved, nothing to publish.");
      #endif
      bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
      return true;
    }
  
    #if (WITH_FLASH > 0)
      File errfile = SPIFFS.open(TOPIC_ERROR_FILE, "w");
    #elif (WITH_SD > 0)
      File errfile = sd.open(TOPIC_ERROR_FILE, FILE_WRITE);
    #endif
    
    if (!errfile)
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Failed to open topics error file!");
        debugln();
      #endif
      bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
      return false;
    }
  
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;
  
    while (file.available() && brokerIsConnected())
    {
      mqttclient.loop();
      
      int l = file.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer));
      lineBuffer[l] = 0;
  
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        bufferedTopics++;
        debug("Buffer Topic ");
        debug(bufferedTopics);
        debug(" Buffer: ");
        debugln(lineBuffer);
      #endif
  
      jsonBuffer.clear();
      // Deserialize the JSON document
      DeserializationError error = deserializeJson(jsonBuffer, lineBuffer);
    
      if (error)
      {
        #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
          debugln("Parsing buffered message failed!");
        #endif
        restoredTopicsFailed++;
        delay(10);
        continue;
      }
  
      const char* topic = jsonBuffer["topic"];
      const char* payload = jsonBuffer["payload"];
  
      jsonBuffer.clear();
      // Deserialize the JSON document
      error = deserializeJson(jsonBuffer, payload);
  
      if (error)
      {
        #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
          debugln("Parsing payload failed!");
        #endif
        restoredTopicsFailed++;
        delay(10);
        continue;
      }
  
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Buffered topic: ");
        debugln(topic);
        debug("Buffered payload: ");
        debugln(payload);
      #endif
  
      if(mqttclient.publish(topic, payload))
      {
        restoredTopics++;
        #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
          debugln("Topic published.");
        #endif
        delay(100);
      }
      else
      {
        errfile.println(lineBuffer);
        restoredTopicsBuffered++;
        #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
          debugln("Topic publishing failed!");
        #endif
      }
    }
   
    // Close the file (File's destructor doesn't close the file)
    file.close();
    errfile.close();
  
    // Save errors for next run
    #if (WITH_FLASH > 0)
      SPIFFS.remove(TOPIC_BUFFER_FILE);
      SPIFFS.rename(TOPIC_ERROR_FILE, TOPIC_BUFFER_FILE);
    #elif (WITH_SD > 0)
      sd.remove(TOPIC_BUFFER_FILE);
      sd.rename(TOPIC_ERROR_FILE, TOPIC_BUFFER_FILE);
    #endif
  
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debug("Topics buffered: ");
      debugln(bufferedTopics);
      debug("Topics restored: ");
      debugln(restoredTopics);
      debug("Topics restoring failed: ");
      debugln(restoredTopicsFailed);
      debug("Topics buffered again: ");
      debugln(restoredTopicsBuffered);
      debugln("finished.");
      debugln();
    #endif
  
    bitSet(brokerStatus, BIT_STATUS_PUBLISHED_BUFFER);
  #endif

  return true;
}

//***************************************************************
// Returns the current state of the client. If a connection attempt fails,
// this can be used to get more information about the failure.
//***************************************************************
String brokerDescribeError(int error)
{
  switch(error)
  {
    case -4:
      return "the server didn't respond within the keepalive time";
      break;
    case -3:
      return "the network connection was broken";
      break;
    case -2:
      return "the network connection failed";
      break;
    case -1:
      return "the client is disconnected cleanly";
      break;
    case -0:
      return "the client is connected";
      break;
    case 1:
      return "the server doesn't support the requested version of BROKER";
      break;
    case 2:
      return "the server rejected the client identifier";
      break;
    case 3:
      return "the server was unable to accept the connection";
      break;
    case 4:
      return "the username/password were rejected";
      break;
    case 5:
      return "the client was not authorized to connect";
      break;
    default:
      return "unknown Error";
      break;
  }
}

//***************************************************************
// Function to disconnect the BROKER server.
//***************************************************************
boolean brokerDisconnect(String state)
{
  #if (WITH_BROKER > 0)
    if(!brokerIsConnected())
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Broker is disconnected.");
      #endif
      return true;
    }
    
    String debugMessage = "Station goes to " + state;
    debugMessage += ", Topics buffered count is " + String(bufferedTopics);
    debugMessage += ", Topics restored count is " + String(restoredTopics);
    debugPublish(String(SYSTEM_SENSOR_ID), debugMessage, true);
    
    mqttclient.disconnect();
    // wait until connection is closed completely
    while( mqttclient.state() != -1)
    {  
      delay(100);
    }
  
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debugln("Broker disconnected.");
    #endif
  #endif

  return false;
}

//***************************************************************
// Callback function to read subscribed topics
//***************************************************************
void brokerCallback(char* topic, byte* payload, unsigned int length)
{
  #if (WITH_BROKER > 0)
    payload[length] = '\0';
    String message = (char*)payload;
  
    /*
    debugln("************************* MQTT Callback ****************************");
    debug("Topic..: ");
    debugln(topic);
    debug("Payload: ");
    debugln(message);
    debugln("********************************************************************");
    debugln();
    */
    
    // OTA update?
    String checkTopic = String(ACTION_TOPIC_OTA);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_OTA);
      if(message.toInt() == 1)
      {
        bitSet(stationActions, BIT_ACTION_REQUIRED_OTA);
      }
      else
      {
        bitClear(stationActions, BIT_ACTION_REQUIRED_OTA);
      }
    }
  
    // Reboot is required?
    checkTopic = String(ACTION_TOPIC_REBOOT);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_REBOOT);
      if(message.toInt() == 1)
      {
        bitSet(stationActions, BIT_ACTION_REQUIRED_REBOOT);
      }
      else
      {
        bitClear(stationActions, BIT_ACTION_REQUIRED_REBOOT);
      }
    }
  
    // Buffer clean is required?
    checkTopic = String(ACTION_TOPIC_BUFFERCLEAN);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_BUFFERCLEAN);
      if(message.toInt() == 1)
      {
        bitSet(stationActions, BIT_ACTION_REQUIRED_BUFFERCLEAN);
      }
      else
      {
        bitClear(stationActions, BIT_ACTION_REQUIRED_BUFFERCLEAN);
      }
    }
  
    // Config clean is required?
    checkTopic = String(ACTION_TOPIC_CONFIGCLEAN);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_CONFIGCLEAN);
      if(message.toInt() == 1)
      {
        bitSet(stationActions, BIT_ACTION_REQUIRED_CONFIGCLEAN);
      }
      else
      {
        bitClear(stationActions, BIT_ACTION_REQUIRED_CONFIGCLEAN);
      }
    }
  
    // Publishing sensor datas?
    checkTopic = String(ACTION_TOPIC_STOP_PUBLISH_SENSOR);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_SENSOR);
      if (message.toInt() == 1)
      {
        bitSet(stationActions, BIT_STATUS_STOPPED_PUBLISH_SENSOR);
      }
      else
      {
        bitClear(stationActions, BIT_STATUS_STOPPED_PUBLISH_SENSOR);
      }
    }
  
    // Publishing debug datas?
    checkTopic = String(ACTION_TOPIC_STOP_PUBLISH_DEBUG);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_DEBUG);
      if (message.toInt() == 1)
      {
        bitSet(stationActions, BIT_STATUS_STOPPED_PUBLISH_DEBUG);
      }
      else
      {
        bitClear(stationActions, BIT_STATUS_STOPPED_PUBLISH_DEBUG);
      }
    }
  
    // Publishing error datas?
    checkTopic = String(ACTION_TOPIC_STOP_PUBLISH_ERROR);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_ERROR);
      if (message.toInt() == 1)
      {
        bitSet(stationActions, BIT_STATUS_STOPPED_PUBLISH_ERROR);
      }
      else
      {
        bitClear(stationActions, BIT_STATUS_STOPPED_PUBLISH_ERROR);
      }
    }
  
    // Publishing buffer datas?
    checkTopic = String(ACTION_TOPIC_STOP_PUBLISH_BUFFER);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_BUFFER);
      if (message.toInt() == 1)
      {
        bitSet(stationActions, BIT_STATUS_STOPPED_PUBLISH_BUFFER);
      }
      else
      {
        bitClear(stationActions, BIT_STATUS_STOPPED_PUBLISH_BUFFER);
      }
    }
  
    // Stop WeeWX sending?
    checkTopic = String(ACTION_TOPIC_STOP_SEND_WEEWX);
  
    if(strcmp(topic, checkTopic.c_str()) == 0)
    {
      bitSet(brokerStatus, BIT_BROKER_RECEIVED_STOP_SEND_WEEWX);
      if(message.toInt() == 1)
      {
        bitSet(stationActions, BIT_STATUS_STOPPED_SEND_WEEWX);
      }
      else
      {
        bitClear(stationActions, BIT_STATUS_STOPPED_SEND_WEEWX);
      }
    }
  #endif
}

//***************************************************************
// Returns if all required subscribed topics received
//***************************************************************
boolean brokerReceiveAllSubscribedTopics()
{
  #if (WITH_BROKER > 0)
    boolean allReceived = false;
    String message = "";
    loops = 0;
  
    while (!allReceived && loops < 60)
    {
      loops++;    
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_OTA) == 1)
      {
        if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_REBOOT) == 1)
        {
          if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_BUFFERCLEAN) == 1)
          {
            if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_CONFIGCLEAN) == 1)
            {
              if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_SENSOR) == 1)
              {
                if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_DEBUG) == 1)
                {
                  if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_ERROR) == 1)
                  {
                    if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_BUFFER) == 1)
                    {
                      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_SEND_WEEWX) == 1)
                      {
                        allReceived = true;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
  
      if (!allReceived)
      {
        mqttclient.loop();
        delay(50);
      }
    }
  
    if(allReceived)
    {
      debugln("All required subscribed topics received.");
    }
    else
    {
      debugln("Not all required subscribed topics received!");
    }
    
    #if ((WITH_DEBUG_BROKER > 0) && ((WITH_DEBUG_TO_SERIAL > 0) || (WITH_DEBUG_TO_BROKER > 0) || (WITH_ERROR_TO_BROKER > 0)))
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_OTA) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_OTA) + ", Action OTA required: " + String(bitRead(stationActions, BIT_ACTION_REQUIRED_OTA));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_OTA) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_REBOOT) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_REBOOT) + ", Action Reboot required: " + String(bitRead(stationActions, BIT_ACTION_REQUIRED_REBOOT));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_REBOOT) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_BUFFERCLEAN) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_BUFFERCLEAN) + ", Action Bufferclean required: " + String(bitRead(stationActions, BIT_ACTION_REQUIRED_BUFFERCLEAN));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_BUFFERCLEAN) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_CONFIGCLEAN) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_CONFIGCLEAN) + ", Action Configclean required: " + String(bitRead(stationActions, BIT_ACTION_REQUIRED_CONFIGCLEAN));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_CONFIGCLEAN) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_SENSOR) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_STOP_PUBLISH_SENSOR) + ", Action stop publish Sensor: " + String(bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_SENSOR));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_STOP_PUBLISH_SENSOR) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_DEBUG) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_STOP_PUBLISH_DEBUG) + ", Action stop publish Debug: " + String(bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_DEBUG));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_STOP_PUBLISH_DEBUG) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_ERROR) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_STOP_PUBLISH_ERROR) + ", Action stop publish Error: " + String(bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_ERROR));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_STOP_PUBLISH_ERROR) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_PUBLISH_BUFFER) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_STOP_PUBLISH_BUFFER) + ", Action stop publish Buffer: " + String(bitRead(stationActions, BIT_STATUS_STOPPED_PUBLISH_BUFFER));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_STOP_PUBLISH_BUFFER) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      if (bitRead(brokerStatus, BIT_BROKER_RECEIVED_STOP_SEND_WEEWX) > 0)
      {
        message = "Action topic " + String(ACTION_TOPIC_STOP_SEND_WEEWX) + ", Action stop send WeeWX: " + String(bitRead(stationActions, BIT_STATUS_STOPPED_SEND_WEEWX));  
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        debugPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
      else
      {
        message = "Subscribed action topic " + String(ACTION_TOPIC_STOP_SEND_WEEWX) + " not received!";
        #if (WITH_DEBUG_BROKER > 0 && WITH_DEBUG_TO_SERIAL > 0)
          debugln(message);
        #endif
        errorPublish(String(SYSTEM_SENSOR_ID), message, false);
      }
  
      debugln();
    #endif
  
    return allReceived;
  #endif

  return true;
}

//***************************************************************
// Function to connect the Broker server.
//***************************************************************
boolean brokerConnect(bool reConnect)
{
  #if (WITH_BROKER > 0)
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debug("Attempting Broker connection to Server ");
      debugln(BROKER_SERVER);
    #endif
  
    if(!wifiIsConnected())
    {
      debugln("WiFi is not connected, start connecting ....");
      if (!wifiConnect())
      {
        debugln("Error during WiFi connect. Abort!");
        return false;
      }
      debugln("WiFi connected.");
    }
  
    if(brokerIsConnected())
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debugln("Broker is connected.");
      #endif
      return true;
    }
    
    // Server settings
    mqttclient.setServer(BROKER_SERVER, BROKER_SERVERPORT);
    mqttclient.setCallback(brokerCallback);
    mqttclient.setBufferSize(BROKER_MQTT_BUFFER_SIZE);
    //mqttclient.setKeepAlive(BROKER_KEEP_ALIVE);
    
    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    
    jsonBuffer["dateTime"] = getTimestamp();
    jsonBuffer["type"] = "DEBUG";
    jsonBuffer["sensor"] = String(SYSTEM_SENSOR_ID);
    jsonBuffer["message"] = "Broker is offline now (last will)."; //last will
  
    //jsonBuffer["Time"] = getTimestamp();
    //jsonBuffer["State"] = "offline"; //last will
    
    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);
  
    // Connect with client ID
    String clientId = BROKER_CLIENTID;
    String stationTopic = String(STATION_ID) + "/debug/" + String(BROKER_OUTPUTFORMAT);
  
    // ... Give ESP 10 seconds to connect to broker.
    unsigned long startTime = millis();
  
    /*
     * boolean connect (clientID, username, password, willTopic, willQoS, willRetain, willMessage, cleanSession)
     * Connects the client with a Will message, username, password and clean-session flag specified.
     * Note : even if the cleanSession is set to false/0 the client will not retry failed qos 1 publishes. This flag is only of use to maintain subscriptions on the broker.
     * Parameters
     * clientID : the client ID to use when connecting to the server.
     * username : the username to use. If NULL, no username or password is used (const char[])
     * password : the password to use. If NULL, no password is used (const char[])
     * willTopic : the topic to be used by the will message (const char[])
     * willQoS : the quality of service to be used by the will message (int : 0,1 or 2)
     * willRetain : whether the will should be published with the retain flag (int : 0 or 1)
     * willMessage : the payload of the will message (const char[])
     * cleanSession : whether to connect clean-session or not (boolean)
     * Returns
     *  false - connection failed.
     *  true - connection succeeded.
    */
    while (!mqttclient.connect(clientId.c_str(), BROKER_USERNAME, BROKER_PASSWORD, (const char *)stationTopic.c_str(), 1, 1, (const char *)publishJson, true) && (millis() - startTime < 10000))
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
       debug(".");
      #endif
      // Wait 1 seconds before retrying
      delay(1000);
    }
  
    if (!brokerIsConnected())
    {
      #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
        debug("Broker connection failed, reason: ");
        debugln(brokerDescribeError(mqttclient.state()));
        debugln("give up!");
      #endif
      return false;
    }
    
    #if (WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_BROKER > 0)
      debugln("Broker is connected.");
    #endif

    if (!reConnect)
    {
      brokerSubscribe();
      delay(100);
    }
    
    return debugPublish(String(SYSTEM_SENSOR_ID), "Broker is online now.", false);
  #endif

  return false;
}

//***************************************************************
// Zambretti Trend
//***************************************************************
int ZambrettiTrend()
{
  int trend = 0;                                    // -1 falling; 0 steady; 1 raising

  #if (WITH_ZAMBRETTI > 0)
    debugln("Calculating Zambretti Trend");
    
    //--> giving the most recent pressure reads more weight
    zambrettiData.diffBarometer[0]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[1])   * 1.5;
    zambrettiData.diffBarometer[1]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[2]);
    zambrettiData.diffBarometer[2]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[3])   / 1.5;
    zambrettiData.diffBarometer[3]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[4])   / 2;
    zambrettiData.diffBarometer[4]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[5])   / 2.5;
    zambrettiData.diffBarometer[5]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[6])   / 3;
    zambrettiData.diffBarometer[6]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[7])   / 3.5;
    zambrettiData.diffBarometer[7]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[8])   / 4;
    zambrettiData.diffBarometer[8]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[9])   / 4.5;
    zambrettiData.diffBarometer[9]  = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[10])  / 5;
    zambrettiData.diffBarometer[10] = (zambrettiData.savedBarometer[0] - zambrettiData.savedBarometer[11])  / 5.5;
    
    //--> calculating the average and storing it into [11]
    zambrettiData.diffBarometer[11] = (zambrettiData.diffBarometer[0]
                               + zambrettiData.diffBarometer[1]
                               + zambrettiData.diffBarometer[2]
                               + zambrettiData.diffBarometer[3]
                               + zambrettiData.diffBarometer[4]
                               + zambrettiData.diffBarometer[5]
                               + zambrettiData.diffBarometer[6]
                               + zambrettiData.diffBarometer[7]
                               + zambrettiData.diffBarometer[8]
                               + zambrettiData.diffBarometer[9]
                               + zambrettiData.diffBarometer[10]) / 11;
    
    debug("Current trend: ");
    debugln(zambrettiData.diffBarometer[11]);
  
    if (zambrettiData.diffBarometer[11] > 3.5)
    {
      zambrettiData.trend_in_words = TEXT_RISING_FAST;
      trend = 1;
    }
    else if (zambrettiData.diffBarometer[11] > 1.5   && zambrettiData.diffBarometer[11] <= 3.5)  
    {
      zambrettiData.trend_in_words = TEXT_RISING;
      trend = 1;
    }
    else if (zambrettiData.diffBarometer[11] > 0.25  && zambrettiData.diffBarometer[11] <= 1.5)
    {
      zambrettiData.trend_in_words = TEXT_RISING_SLOW;
      trend = 1;
    }
    else if (zambrettiData.diffBarometer[11] > -0.25 && zambrettiData.diffBarometer[11] < 0.25)
    {
      zambrettiData.trend_in_words = TEXT_STEADY;
      trend = 0;
    }
    else if (zambrettiData.diffBarometer[11] >= -1.5 && zambrettiData.diffBarometer[11] < -0.25)
    {
      zambrettiData.trend_in_words = TEXT_FALLING_SLOW;
      trend = -1;
    }
    else if (zambrettiData.diffBarometer[11] >= -3.5 && zambrettiData.diffBarometer[11] < -1.5)
    {
      zambrettiData.trend_in_words = TEXT_FALLING;
      trend = -1;
    }
    else if (zambrettiData.diffBarometer[11] <= -3.5)
    {
      zambrettiData.trend_in_words = TEXT_FALLING_FAST;
      trend = -1;
    }
  
    debugln(zambrettiData.trend_in_words);

  #endif
  
  return trend;
}

//***************************************************************
// Zambretti Letter
//***************************************************************
char ZambrettiLetter()
{
  char z_letter = ' ';

  #if (WITH_ZAMBRETTI > 0)
    debugln("Calculating Zambretti letter");
  
    int(z_trend) = ZambrettiTrend();
    unsigned long current_timestamp = getTimestamp();    // Actual timestamp read from NTPtime_t now;
    //int actMonth = timeClient.getMonth();
    int actMonth = month(current_timestamp);
    
    // Case trend is falling
    if (z_trend == -1)
    {
      float zambretti = 0.0009746 * zambrettiData.barometer * zambrettiData.barometer - 2.1068 * zambrettiData.barometer + 1138.7019; 
      //A Winter falling generally results in a Z value lower by 1 unit
      if (month(current_timestamp) < 4 || month(current_timestamp) > 9) zambretti = zambretti - 1;
      if (zambretti < 0) zambretti = 0;
      debug("Calculated and rounded Zambretti in numbers: ");
      debugln(round(zambretti));
      switch (int(round(zambretti)))
      {
        case 0:  z_letter = 'A'; break;       //Settled Fine
        case 1:  z_letter = 'A'; break;       //Settled Fine
        case 2:  z_letter = 'B'; break;       //Fine Weather
        case 3:  z_letter = 'D'; break;       //Fine Becoming Less Settled
        case 4:  z_letter = 'H'; break;       //Fairly Fine Showers Later
        case 5:  z_letter = 'O'; break;       //Showery Becoming unsettled
        case 6:  z_letter = 'R'; break;       //Unsettled, Rain later
        case 7:  z_letter = 'U'; break;       //Rain at times, worse later
        case 8:  z_letter = 'V'; break;       //Rain at times, becoming very unsettled
        case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
      }
    }
    
    // Case trend is steady
    if (z_trend == 0)
    {
      float zambretti = 138.24 - 0.133 * zambrettiData.barometer;
      debug("Calculated and rounded Zambretti in numbers: ");
      debugln(round(zambretti));
      switch (int(round(zambretti)))
      {
        case 0:  z_letter = 'A'; break;       //Settled Fine
        case 1:  z_letter = 'A'; break;       //Settled Fine
        case 2:  z_letter = 'B'; break;       //Fine Weather
        case 3:  z_letter = 'E'; break;       //Fine, Possibly showers
        case 4:  z_letter = 'K'; break;       //Fairly Fine, Showers likely
        case 5:  z_letter = 'N'; break;       //Showery Bright Intervals
        case 6:  z_letter = 'P'; break;       //Changeable some rain
        case 7:  z_letter = 'S'; break;       //Unsettled, rain at times
        case 8:  z_letter = 'W'; break;       //Rain at Frequent Intervals
        case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
        case 10: z_letter = 'Z'; break;       //Stormy, much rain
      }
    }
    
    // Case trend is rising
    if (z_trend == 1)
    {
      float zambretti = 142.57 - 0.1376 * zambrettiData.barometer;
      //A Summer rising, improves the prospects by 1 unit over a Winter rising
      if (month(current_timestamp) >= 4 && month(current_timestamp) <= 9) zambretti = zambretti + 1;
      if (zambretti > 13) zambretti = 13;
      debug("Calculated and rounded Zambretti in numbers: ");
      debugln(round(zambretti));
      switch (int(round(zambretti)))
      {
        case 0:  z_letter = 'A'; break;       //Settled Fine
        case 1:  z_letter = 'A'; break;       //Settled Fine
        case 2:  z_letter = 'B'; break;       //Fine Weather
        case 3:  z_letter = 'C'; break;       //Becoming Fine
        case 4:  z_letter = 'F'; break;       //Fairly Fine, Improving
        case 5:  z_letter = 'G'; break;       //Fairly Fine, Possibly showers, early
        case 6:  z_letter = 'I'; break;       //Showery Early, Improving
        case 7:  z_letter = 'J'; break;       //Changeable, Improving
        case 8:  z_letter = 'L'; break;       //Rather Unsettled Clearing Later
        case 9:  z_letter = 'M'; break;       //Unsettled, Probably Improving
        case 10: z_letter = 'Q'; break;       //Unsettled, short fine Intervals
        case 11: z_letter = 'T'; break;       //Very Unsettled, Finer at times
        case 12: z_letter = 'Y'; break;       //Stormy, possibly improving
        case 13: z_letter = 'Z'; break;;      //Stormy, much rain
      }
    }
    
    debug("This is Zambretti's famous letter: ");
    debugln(z_letter);
  
  #endif
  
  return z_letter;
}

//***************************************************************
// Zambretti says
//***************************************************************
String ZambrettiSays(char code)
{
  String zambrettis_words = "";

  #if (WITH_ZAMBRETTI > 0)
    switch (code)
    {
      case 'A': zambrettis_words = TEXT_ZAMBRETTI_A; break;  //see Tranlation.h
      case 'B': zambrettis_words = TEXT_ZAMBRETTI_B; break;
      case 'C': zambrettis_words = TEXT_ZAMBRETTI_C; break;
      case 'D': zambrettis_words = TEXT_ZAMBRETTI_D; break;
      case 'E': zambrettis_words = TEXT_ZAMBRETTI_E; break;
      case 'F': zambrettis_words = TEXT_ZAMBRETTI_F; break;
      case 'G': zambrettis_words = TEXT_ZAMBRETTI_G; break;
      case 'H': zambrettis_words = TEXT_ZAMBRETTI_H; break;
      case 'I': zambrettis_words = TEXT_ZAMBRETTI_I; break;
      case 'J': zambrettis_words = TEXT_ZAMBRETTI_J; break;
      case 'K': zambrettis_words = TEXT_ZAMBRETTI_K; break;
      case 'L': zambrettis_words = TEXT_ZAMBRETTI_L; break;
      case 'M': zambrettis_words = TEXT_ZAMBRETTI_M; break;
      case 'N': zambrettis_words = TEXT_ZAMBRETTI_N; break;
      case 'O': zambrettis_words = TEXT_ZAMBRETTI_O; break;
      case 'P': zambrettis_words = TEXT_ZAMBRETTI_P; break; 
      case 'Q': zambrettis_words = TEXT_ZAMBRETTI_Q; break;
      case 'R': zambrettis_words = TEXT_ZAMBRETTI_R; break;
      case 'S': zambrettis_words = TEXT_ZAMBRETTI_S; break;
      case 'T': zambrettis_words = TEXT_ZAMBRETTI_T; break;
      case 'U': zambrettis_words = TEXT_ZAMBRETTI_U; break;
      case 'V': zambrettis_words = TEXT_ZAMBRETTI_V; break;
      case 'W': zambrettis_words = TEXT_ZAMBRETTI_W; break;
      case 'X': zambrettis_words = TEXT_ZAMBRETTI_X; break;
      case 'Y': zambrettis_words = TEXT_ZAMBRETTI_Y; break;
      case 'Z': zambrettis_words = TEXT_ZAMBRETTI_Z; break;
      case '0': zambrettis_words = TEXT_ZAMBRETTI_0; break;
       default: zambrettis_words = TEXT_ZAMBRETTI_DEFAULT; break;
    }
  
  #endif
    
  return zambrettis_words;
}

//***************************************************************
// set default zambretti data file
//***************************************************************
boolean initZambrettiData()
{
  #if (WITH_ZAMBRETTI > 0)
    debugln("Starting initializing Zambretti data ...");
  
    #if (WITH_FLASH > 0)
      File file = SPIFFS.open(ZAMBRETTI_FILE, "w");
      if (!file)
      {
        debugln("Failed to open Zambretti data file for writing!");
        errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open Zambretti data file for writing!", false);
        return false;
      }
    #elif (WITH_SD > 0)
      File file = sd.open(ZAMBRETTI_FILE, O_WRONLY | O_CREAT | O_TRUNC);
      if (!file)
      {
        debugln("Failed to open Zambretti data file for writing!");
        errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open Zambretti data file for writing!", false);
        return false;
      }
    #else
      debugln("SD/FLASH not configured!");
      errorPublish(String(SYSTEM_SENSOR_ID), "SD/FLASH not configured!", false);
      return false;
    #endif
  
    file.println(getTimestamp());                   // Saving timestamp
    file.println("1");                              // Saving accuracy
    for ( int i = 0; i < 12; i++)
    {
      file.println(zambrettiData.barometer);        // Filling pressure array with current pressure
    }
  
    file.close();
    debugln("Saved initial Zamberetti data file.");
  #endif
  
  return true;
}

//***************************************************************
// write the zambretti data file
//***************************************************************
boolean writeZambrettiData()
{
  #if (WITH_ZAMBRETTI > 0)
    #if (WITH_FLASH > 0)
      File file = SPIFFS.open(ZAMBRETTI_FILE, "w");
      if (!file)
      {
        debugln("Failed to open Zambretti data file for writing!");
        errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open Zambretti data file for writing!", false);
        return false;
      }
    #elif (WITH_SD > 0)
      File file = sd.open(ZAMBRETTI_FILE, O_WRONLY | O_CREAT | O_TRUNC);
      if (!file)
      {
        debugln("Failed to open Zambretti data file for writing!");
        errorPublish(String(SYSTEM_SENSOR_ID), "Failed to open Zambretti data file for writing!", false);
        return false;
      }
    #else
      debugln("SD/FLASH not configured!");
      errorPublish(String(SYSTEM_SENSOR_ID), "SD/FLASH not configured!", false);
      return false;
    #endif
    
    debugln("Now writing to Zambretti data file...");
    
    file.println(zambrettiData.measurementTimestamp); // Saving timestamp to /data.txt
    file.println(zambrettiData.accuracy);             // Saving accuracy value to /data.txt
    
    for ( int i = 0; i <= 11; i++) {
      file.println(zambrettiData.savedBarometer[i]);  // Filling pressure array with updated values
    }
  
    file.close();
    debugln("Saved act Zamberetti data file.");
  
  #endif
  
  return true;
}

//***************************************************************
// Reads the zambretti data file
//***************************************************************
boolean readZambrettiData(boolean init = false)
{
  #if (WITH_ZAMBRETTI > 0)
    debugln("Load Zambretti data file ...");
    // Open file for reading
    #if (WITH_FLASH > 0)
      File file = SPIFFS.open(ZAMBRETTI_FILE, "r");
    #elif (WITH_SD > 0) // Open file for SD Card reading
      File file = sd.open(ZAMBRETTI_FILE, FILE_READ);
    #else
      return false;
    #endif
    
    if (!file)
    {
      debugln("Failed to open zambretti file");
      initZambrettiData();  // no file there -> initializing
    }
    else if (init)
    {
      initZambrettiData();  // init -> initializing
    }
    
    debugln("Now reading from Zambretti data file ...");
    
    String temp_data = file.readStringUntil('\n');  
    zambrettiData.measurementTimestamp = temp_data.toInt();
    debug("Last measurement timestamp from Zambretti file: ");
    debugln(zambrettiData.measurementTimestamp);
    
    temp_data = file.readStringUntil('\n');  
    zambrettiData.accuracy = temp_data.toInt();
    debug("Accuracy value read from Zambretti file: ");
    debugln(zambrettiData.accuracy);
  
    /*
    debugln("Last 12 saved pressure values from Zambretti file: ");
    for (int i = 0; i <= 11; i++) {
      temp_data = file.readStringUntil('\n');
      zambrettiData.savedBarometer[i] = temp_data.toInt();
      debugln(zambrettiData.savedBarometer[i]);
    }
    */
  
    file.close();
  
    debugln("Last Zambretti datas loaded.");

  #endif
  
  return true;
}

//***************************************************************
// update Zambretti Data
//***************************************************************
void updateZambrettiData()
{
  #if (WITH_ZAMBRETTI > 0)
    debugln();
    debugln("Updating Zambretti data ...");
    
    unsigned long actTimestamp = getTimestamp();
  
    zambrettiData.barometer = bme280Data.barometer;    
  
    readZambrettiData();
  
    if (zambrettiData.savedBarometer[0] < 900)
    {
      readZambrettiData(true);
    }
  
    if (actTimestamp - zambrettiData.measurementTimestamp > 21600)     // last save older than 6 hours -> re-initialize values
    {
      readZambrettiData(true);
    }
    else if (actTimestamp - zambrettiData.measurementTimestamp > 1800) // it is time for pressure update (1800 sec = 30 min)
    { 
      for (int i = 11; i >= 1; i = i -1)
      {
        zambrettiData.savedBarometer[i] = zambrettiData.savedBarometer[i-1];    // shifting values one to the right
      }
    
      zambrettiData.savedBarometer[0] = zambrettiData.barometer;
  
      if (zambrettiData.accuracy < 12)
      {
        zambrettiData.accuracy = zambrettiData.accuracy + 1;                    // one value more -> accuracy rises (up to 12 = 100%)
      }
  
      //zambrettiData.measurementTimestamp = getTimestamp();
      zambrettiData.measurementTimestamp = measurementTimestamp;
  
      writeZambrettiData();
    }
    else
    {
      writeZambrettiData();
    }
  #endif
}

//***************************************************************
// calculate Zambretti Data
//***************************************************************
void calculateZambrettiData()
{
  #if (WITH_ZAMBRETTI > 0)
    if ( stationVoltage > 3.3 )                                                  // check if batt is still ok
    {
      zambrettiData.ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
      zambrettiData.forecast_in_words = TEXT_ZAMBRETTI_FORECAST;
      zambrettiData.pressure_in_words = TEXT_AIR_PRESSURE;
      zambrettiData.accuracy_in_words = TEXT_ZAMBRETTI_ACCURACY;
    }
    else
    {
      zambrettiData.ZambrettisWords = ZambrettiSays('0');   // send Message that battery is empty
    }
  #endif
}

//***************************************************************
// publish Zambretti Data
//***************************************************************
boolean publishZambrettiData()
{
  #if (WITH_ZAMBRETTI > 0)
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;
  
    zambrettiData.accuracy_in_percent = zambrettiData.accuracy*94/12;            // 94% is the max predicion accuracy of Zambretti
  
    jsonBuffer["dateTime"] = measurementTimestamp;
    jsonBuffer["forecast"] = zambrettiData.ZambrettisWords;
    jsonBuffer["trend"] = zambrettiData.trend_in_words;
    jsonBuffer["accuracy"] = zambrettiData.accuracy_in_percent;
      
    if (zambrettiData.accuracy < 12)
    {
      jsonBuffer["reason"] = "Not enough weather data yet. We need " + String((12 - zambrettiData.accuracy) / 2) + " hours more to get sufficient data.";
    }
  
    char publishJson[1024];
    serializeJson(jsonBuffer, publishJson);
  
    if(sensorPublish(String(ZAMBRETTI_SENSOR_ID), publishJson, false))
    {
      zambrettiData.received = 1;
      zambrettiData.published = 1;
      return true;
    }
    else
    {
      return false;
    }
  #endif

  return true;
}

//***************************************************************
// print Zambretti Data
//***************************************************************
void printZambrettiData()
{
  #if (WITH_ZAMBRETTI > 0)
    zambrettiData.accuracy_in_percent = zambrettiData.accuracy*94/12;            // 94% is the max predicion accuracy of Zambretti
  
    debugln();
    debugln("************************** ZAMBRETTI *****************************");
  
    //String postStr = String("  " + zambrettiData.forecast_in_words + ": " + zambrettiData.ZambrettisWords + ". " + zambrettiData.pressure_in_words + " " + zambrettiData.trend_in_words + ". " + zambrettiData.accuracy_in_words + " " + zambrettiData.accuracy_in_percent + "%");
    //debugln(postStr);
  
    debug(zambrettiData.forecast_in_words + ": ");
    debugln(zambrettiData.ZambrettisWords + ".");
    debug(zambrettiData.pressure_in_words + ": ");
    debugln(zambrettiData.trend_in_words + ".");
    debug(zambrettiData.accuracy_in_words + ": ");
    debugln(String(zambrettiData.accuracy_in_percent) + "%");
  
    if (zambrettiData.accuracy < 12){
      debugln("  Reason: Not enough weather data yet.");
      debug("  We need ");
      debug((12 - zambrettiData.accuracy) / 2);
      debugln(" hours more to get sufficient data.");
    }
    
    debugln("******************************************************************");
    debugln();
  #endif
}

//***************************************************************
// Set RTC with values from NTP Server
//***************************************************************
void rtcSet()
{
  #if (WITH_RTC > 0)
    if(wifiIsConnected())
    {
      timeClient.begin();
      debugln("Set RTC DS1307 with NTP time.");
      timeClient.update();
      rtc.adjust(DateTime(timeClient.getEpochTime()));
      debugln();
    }
  #endif
}

//***************************************************************
// Initialize RTC
//***************************************************************
void rtcSetup()
{
  #if (WITH_RTC > 0)
    debugln("Initialize RTC DS1307.");
    
    if (!rtc.begin())
    {
      debugln("Couldn't find RTC DS1307");
      errorPublish(String(SYSTEM_SENSOR_ID), "Couldn't find RTC DS1307", false);
    }
    else
    {
      debugln("RTC DS1307 is initialized");
    }
  #endif
}

//***************************************************************
// Starts the OTA process
//***************************************************************
void otaStart()
{
  debugln("Starting OTA update...");

  #if (WITH_BROKER > 0)
    bitClear(stationActions, BIT_ACTION_REQUIRED_OTA);
    bitSet(stationActions, BIT_STATUS_ACTIVE_OTA);
    String stationTopic = String(ACTION_TOPIC_OTA);
  #endif

  String debugMessage = String("HTTPUpdateServer ready! ") + String("Open http://") + String(STATION_ID) + String(".local/update in your browser.");
  const char* host = STATION_ID;

  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);

  debugln(debugMessage);
  debugln("OTA is active now.");

  #if (WITH_BROKER > 0)
    brokerPublish(String(ACTION_TOPIC_OTA), "0", true);
    delay(200);
    debugPublish(String(SYSTEM_SENSOR_ID), debugMessage, false);
  
    debugPublish(String(SYSTEM_SENSOR_ID), "OTA is active now.", true);
  #endif
}

//***************************************************************
// Handle the OTA process
//***************************************************************
void otaHandler()
{
  httpServer.handleClient();
  MDNS.update();
}

//***************************************************************
// get formated date
//***************************************************************
String getFormattedDateTime()
{
  unsigned long rawTime = getTimestamp();

  //temporary, copied code from pull request https://github.com/arduino-libraries/NTPClient/pull/45
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);
  
  int daysMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31}; // array index starts by 0
  double operador = 60*60*24*365; //secons, minutes, hours, days
  double years = rawTime / operador;
  int leapYearsPast = floor(years / 4);
  double resto = years - floor(years);
  int Year = 1970 + floor(years);  // current year

  // if it's leap change february
  if ((Year%4 == 0) && ((Year%100 != 0) || (Year%400 == 0))) {daysMonth[1] = 29;} // put February in array

  // final leap
  int DaysOfYear = floor(resto * 365) - leapYearsPast; // days to end of year
  int Month = 0;
  int suma = 0;
  int Day = DaysOfYear;

  while ((suma + daysMonth[Month]) <= DaysOfYear)
  {
    suma += daysMonth[Month];
    Day -= daysMonth[Month];
    Month++;
  }

  Month++;
  String DayStr = Day < 10 ? "0" + String(Day) : String(Day);
  String MonthStr = Month < 10 ? "0" + String(Month) : String(Month);
  String YearStr = String(Year);

  return DayStr + "." + MonthStr + "." + YearStr + " " + hoursStr + ":" + minuteStr + ":" + secondStr;
}

//***************************************************************
// Start Interrupt to receive W132 datas
//***************************************************************
void w132Start()
{
  #if (WITH_W132 > 0)
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED) == 1)
    {
      debugln("Start W132.");
      w132LastDataReceived = 0;
      w132TempHumidityData.received = 0;
      w132TempHumidityData.published = 0;
      w132WindSpeedData.received = 0;
      w132WindSpeedData.published = 0;
      w132WindDirGustData.received = 0;
      w132WindDirGustData.published = 0;
      bitClear(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED);
      attachInterrupt(digitalPinToInterrupt(W132_DATAPIN), w132InterruptHandler, CHANGE);
    }
  #else
    bitSet(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED);
  #endif
}

//***************************************************************
// Stop Interrupt to receive W132 datas
//***************************************************************
void w132Stop()
{
  #if (WITH_W132 > 0)
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED) != 1)
    {
      detachInterrupt(digitalPinToInterrupt(W132_DATAPIN)); // Interrupts off
      bitSet(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED);
      debugln("W132 stopped!");
    }
  #else
    bitSet(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED);
  #endif
}

//***************************************************************
// Setup W132 sensor
//***************************************************************
void w132Setup()
{
  #if (WITH_W132 > 0)
    debugln("Initialize W132.");
    pinMode(W132_DATAPIN, INPUT);
  #endif
}

//***************************************************************
// check if w132 received crc is correct for received value
//***************************************************************
boolean w132CRCValid(unsigned long value, byte checksum)
{
  #if (WITH_W132 > 0)
    byte calculatedChecksum = 0xF;
    for (int i = 0 ; i < 8 ; i++) calculatedChecksum -= (byte)(value >> (i * 4));
    calculatedChecksum &= 0xF;
    return (calculatedChecksum == checksum);
  #endif
}

//***************************************************************
// Handle W132 received data telegram
//***************************************************************
ICACHE_RAM_ATTR void w132InterruptHandler()
{
  #if (WITH_W132 > 0)
    static long w132LineUp, w132LineDown;
    static unsigned long w132DataBits = 0;
    static byte w132CRCBits = 0;
    static byte w132BitsCounted = 0;
    long w132ZeroTime, w132OneTime;
  
    byte w132State = digitalRead(W132_DATAPIN); // current pin state
  
    if (w132State) // pin is now HIGH
    {
      w132LineUp = micros(); // line went HIGH after being LOW at this time
      w132ZeroTime = w132LineUp - w132LineDown; // calculate the LOW pulse time
      if (w132ZeroTime > VENTUS_SYNC - VENTUS_TOL && w132ZeroTime < VENTUS_SYNC + VENTUS_TOL)
      {
        // new start message
        w132DataBits = 0;
        w132CRCBits = 0;
        w132BitsCounted = 0;
      }
      else if (w132ZeroTime > VENTUS_ONE - VENTUS_TOL && w132ZeroTime < VENTUS_ONE + VENTUS_TOL)
      { // set the one bits
        if (w132BitsCounted < VENTUS_DATALEN)
          bitSet(w132DataBits, w132BitsCounted);
        else
          bitSet(w132CRCBits, w132BitsCounted - VENTUS_DATALEN);
        w132BitsCounted++;
      }
      else if (w132ZeroTime > VENTUS_ZERO - VENTUS_TOL && w132ZeroTime < VENTUS_ZERO + VENTUS_TOL)
      { // setting zero bits is not necessary, but count them
        w132BitsCounted++;
      }
      else // received bit is not a SYNC, ONE or ZERO bit, so restart
      {
        w132DataBits = 0;
        w132CRCBits = 0;
        w132BitsCounted = 0;
      }
  
      if (w132BitsCounted >= VENTUS_DATALEN + VENTUS_CRCLEN) // all bits received
      {
        // write valid value to buffer
        if (w132CRCValid(w132DataBits, w132CRCBits))
        {
          w132fifoWrite(w132DataBits);
        }
        else
        {
          w132fifoWrite(0); // write 0 to FIFO buffer (0 = invalid value received)
        }
        w132DataBits = 0;
        w132CRCBits = 0;
        w132BitsCounted = 0;
      }
    }
    else
    { // High values have no information with them
      w132LineDown = micros(); // line went LOW after being HIGH
      w132OneTime = w132LineDown - w132LineUp; // calculate the HIGH pulse time
    }
  #endif
}

//***************************************************************
// writes W132 received data telegram into fifo buffer
//***************************************************************
void w132fifoWrite(long item)
// write item into ring buffer
{
  #if (WITH_W132 > 0)
    w132fifoBuf[w132fifoWriteIndex] = item; // store the item
    if (!(w132fifoWriteIndex + 1 == w132fifoReadIndex || (w132fifoWriteIndex + 1 >= W132_FIFOSIZE && w132fifoReadIndex == 0)))
      w132fifoWriteIndex++;  // advance write pointer in ringbuffer
    if (w132fifoWriteIndex >= W132_FIFOSIZE) w132fifoWriteIndex = 0; // ring buffer is at its end
  #endif
}

//***************************************************************
// reads W132 received data telegram from fifo buffer
//***************************************************************
unsigned long w132fifoRead()
// always check first if item is available with fifoAvailable()
// before reading the ring buffer using this function
{
  #if (WITH_W132 > 0)
    detachInterrupt(digitalPinToInterrupt(W132_DATAPIN)); // Interrupts off while changing the read pointer for the ringbuffer
    unsigned long item;
    item = w132fifoBuf[w132fifoReadIndex];
    w132fifoBuf[w132fifoReadIndex] = 0;
    w132fifoReadIndex++;
    if (w132fifoReadIndex >= W132_FIFOSIZE) w132fifoReadIndex = 0;
    attachInterrupt(digitalPinToInterrupt(W132_DATAPIN), w132InterruptHandler, CHANGE); // Interrupts on again
    return item;
  #endif

  return 0;
}

//***************************************************************
// Returns if fifo buffer contain data
//***************************************************************
boolean w132fifoAvailable()
// item is available for reading if (fifoReadIndex!=fifoWriteIndex)
{
  #if (WITH_W132 > 0)
    return (w132fifoReadIndex != w132fifoWriteIndex);
  #endif
}

//***************************************************************
// Returns cardinal points
//***************************************************************
char* determineCardinalPoints (unsigned int windDirection)
{
  if(windDirection >= 315) return "NW";
  if(windDirection >= 270) return "W";
  if(windDirection >= 225) return "SW";
  if(windDirection >= 180) return "S";
  if(windDirection >= 135) return "SO";
  if(windDirection >= 90)  return "O";
  if(windDirection >= 45)  return "NO";
  if(windDirection >= 0)   return "N";
}

//***************************************************************
// Prints W312 datas to serial
//***************************************************************
void w132PrintResults(unsigned long value)
{
  #if (WITH_W132 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("********************* W132 *****************************");
  
    debug("Telegram: ");
    for (byte i = 0; i < 32; i++)
    {
      debug(bitRead(value, i));
    }
    debugln("");
  
    // Battery State
    byte batteryState = bitRead(value, 8);    // bit 8 is set if battery voltage is low
    debug("Battery Low: ");
    debugln(batteryState);
  
    // Trigger
    bool forcedSend = bitRead(value, 11); // bit 11 is set if manual send button was pressed
    debug("Trigger: ");
    debugln(forcedSend);
  
    // Temperature Trend
    byte trend = (value >> 9 & 0b11); // bit 9, 10
  
    // if the Trend value is 3, then it's not a temperature message, but a wind message
    if (trend != 3)
    {
      char tempTrend;
      if (trend == 0) tempTrend = '=';     // temp tendency steady
      else if (trend == 1) tempTrend = '-'; // temp tendency falling
      else if (trend == 2) tempTrend = '+'; // temp tendency rising

      debug("Timestamp.....: ");
      debugln(w132TempHumidityData.measurementTimestamp);
  
      debug("Temperature...: ");
      debug(w132TempHumidityData.temperature);
      debugln(" °C");
      
      debug("Temperature...: ");
      debug(w132TempHumidityData.tempf);
      debugln(" °F");
      
      debug("Trend.........: ");
      debugln(tempTrend);
  
      debug("Humidity......: ");
      debug(w132TempHumidityData.humidity);
      debugln(" %");
      
      debug("Dewpoint......: ");
      debug(w132TempHumidityData.dewpoint);
      debugln(" °C");

      debug("Dewpoint......: ");
      debug(w132TempHumidityData.dewptf);
      debugln(" °F");

      debug("spreadDewpoint: ");
      debug(w132TempHumidityData.spreadDewpoint);
      debugln(" °C");

      debug("spreadDewpoint: ");
      debug(w132TempHumidityData.spreadDewptf);
      debugln(" °F");

      debug("Heatindex.....: ");
      debug(w132TempHumidityData.heatindex);
      debugln(" °C");

      debug("Heatindex.....: ");
      debug(w132TempHumidityData.heatindexf);
      debugln(" °F");

      debug("Windchill.....: ");
      if (w132TempHumidityData.windchill != INVALID_FLOAT_VALUE)
      {
        debug(w132TempHumidityData.windchill);        
        debugln(" °C");
        debug("Windchill.....: ");
        debug(w132TempHumidityData.windchillf);        
        debugln(" °F");
      }
      else
      {
        debugln("W132 WindSpeed not available, Windchill not calculated");
      }
    }
    else
    {
      if ((value >> 12 & 0b111) != 7) // bit 12..14, != 111 ==> Wind Speed
      {
        debug("Timestamp.....: ");
        debugln(w132WindSpeedData.measurementTimestamp);

        debug("Wind Speed....: ");
        debug(w132WindSpeedData.windSpeed);
        debugln(" m/s");

        debug("Wind Speed....: ");
        debug(w132WindSpeedData.windSpeedKmh);
        debugln(" km/h");

        if (w132TempHumidityData.valid > 0)
        {
          debug("Temperature...: ");
          debug(w132TempHumidityData.temperature);
          debugln(" °C");
      
          debug("Temperature...: ");
          debug(w132TempHumidityData.tempf);
          debugln(" °F");
        }
        
        debug("Dewpoint......: ");
        if (w132WindSpeedData.dewpoint != INVALID_FLOAT_VALUE)
        {
          debug(w132WindSpeedData.dewpoint);        
          debugln(" °C");

          debug("Dewpoint......: ");
          debug(w132WindSpeedData.dewptf);        
          debugln(" °F");
        }
        else
        {
          debugln("W132 Temperature/Humidity not available, Dewpoint not calculated");
        }

        debug("spreadDewpoint: ");
        if (w132WindSpeedData.dewpoint != INVALID_FLOAT_VALUE)
        {
          debug(w132WindSpeedData.spreadDewpoint);        
          debugln(" °C");
  
          debug("spreadDewpoint: ");
          debug(w132WindSpeedData.spreadDewptf);        
          debugln(" °F");
        }
        else
        {
          debugln("W132 Temperature/Humidity not available, spreadDewpoint not calculated");
        }

        debug("Heatindex.....: ");
        if (w132WindSpeedData.heatindex != INVALID_FLOAT_VALUE)
        {
          debug(w132WindSpeedData.heatindex);        
          debugln(" °C");

          debug("Heatindex.....: ");
          debug(w132WindSpeedData.heatindexf);        
          debugln(" °F");
        }
        else
        {
          debugln("W132 Temperature/Humidity not available, Heatindex not calculated");
        }

        debug("Windchill.....: ");
        if (w132WindSpeedData.windchill != INVALID_FLOAT_VALUE)
        {
          debug(w132WindSpeedData.windchill);        
          debugln(" °C");

          debug("Windchill.....: ");
          debug(w132WindSpeedData.windchillf);        
          debugln(" °F");
        }
        else
        {
          debugln("W132 Temperature/Humidity not available, Windchill not calculated");
        }
      }
      else if ((value >> 12 & 0b111) == 7) // bit 12..14, = 111 ==> Wind Direction & Gust
      {
        debug("Timestamp.................: ");
        debugln(w132WindDirGustData.measurementTimestamp);

        debug("Wind Direction............: ");
        debugln(w132WindDirGustData.windDir);
        debug("Wind Direction transformed: ");
        debugln(w132WindDirGustData.windDirCardinalPoints);
  
        debug("Wind Gust.................: ");
        debug(w132WindDirGustData.windGust);
        debugln(" m/s");

        debug("Wind Gust.................: ");
        debug(w132WindDirGustData.windGustKmh);
        debugln(" km/h");
      }
    }
  
    debugln("********************************************************");
    debugln();
  #endif
}

//***************************************************************
// Decode received W132 data telegram
//***************************************************************
boolean w132DecodeResults(unsigned long value)
{
  #if (WITH_W132 > 0)
    //w132TempHumidityData.received = 0;
    //w132WindDirGustData.received = 0;
    //w132WindSpeedData.received = 0;
    
    //w132TempHumidityData.published = 0;
    //w132WindDirGustData.published = 0;
    //w132WindSpeedData.published = 0;

    // Battery State
    byte batteryLow = bitRead(value, 8);    // bit 8 is set if battery voltage is low
    
    // Trigger
    byte forcedSend = bitRead(value, 11); // bit 11 is set if manual send button was pressed
  
    // Temperature Trend
    byte trend = (value >> 9 & 0b11); // bit 9, 10
  
    // if the Trend value is 3, then it's not a temperature message, but a wind message
    if (trend != 3)
    {
      // Temperature Trend
      /*
      char tempTrend;
      if (trend == 0) tempTrend = '=';     // temp tendency steady
      else if (trend == 1) tempTrend = '-'; // temp tendency falling
      else if (trend == 2) tempTrend = '+'; // temp tendency rising
      */
      bitSet(ventusStatus, BIT_VENTUS_STATUS_W132_TEMPHUMIDITY_RECEIVED);
      w132TempHumidityData.valid = 1;
      w132TempHumidityData.received = 1;
      w132TempHumidityData.measurementTimestamp = getTimestamp();
      w132TempHumidityData.windBatteryStatus = batteryLow;
      w132TempHumidityData.trend = trend;

      // Temperature (C)
      int temp = (value >> 12 & 0b11111111111); // bit 12..22
  
      // bit 23 is sign
      if (bitRead(value, 23) == 1)
      {
        temp = -2048 + temp;
      }
      w132TempHumidityData.temperature = (float)temp / 10;
      w132TempHumidityData.tempf = (w132TempHumidityData.temperature * 9.0) / 5.0 + 32.0; // Farenheit
  
      // Humidity (%)
      byte humidityOnes = value >> 24 & 0b1111; // bit 24..27
      byte humidityTens = value >> 28 & 0b1111; // bit 28..31
      w132TempHumidityData.humidity = (humidityTens * 10) + humidityOnes;

      w132TempHumidityData.dewpoint = getDewPoint(w132TempHumidityData.temperature, w132TempHumidityData.humidity);
      w132TempHumidityData.dewptf = (w132TempHumidityData.dewpoint * 9.0) / 5.0 + 32.0; // Farenheit

      w132TempHumidityData.spreadDewpoint = (w132TempHumidityData.temperature - w132TempHumidityData.dewpoint);
      w132TempHumidityData.spreadDewptf = (w132TempHumidityData.tempf - w132TempHumidityData.dewptf);
      
      w132TempHumidityData.heatindex = getHeatIndex(w132TempHumidityData.temperature, w132TempHumidityData.humidity);
      w132TempHumidityData.heatindexf = (w132TempHumidityData.heatindex * 9.0) / 5.0 + 32.0; // Farenheit

      if (w132WindSpeedData.valid > 0)
      {
        w132TempHumidityData.windchill = getWindChill(w132TempHumidityData.temperature, w132WindSpeedData.windSpeedKmh);  
        w132TempHumidityData.windchillf = (w132TempHumidityData.windchill * 9.0) / 5.0 + 32.0; // Farenheit 
      }
      else
      {
        w132TempHumidityData.windchill = INVALID_FLOAT_VALUE;
        w132TempHumidityData.windchillf = INVALID_FLOAT_VALUE;
      }
    }
    else
    {
      if ((value >> 12 & 0b111) != 7) // bit 12..14, != 111 ==> Wind Speed
      {
        bitSet(ventusStatus, BIT_VENTUS_STATUS_W132_WINDSPEED_RECEIVED);

        w132WindSpeedData.received = 1;
        w132WindSpeedData.valid = 1;
        w132WindSpeedData.measurementTimestamp = getTimestamp();
        w132WindSpeedData.windBatteryStatus = batteryLow;

        // Wind Speed (m/s), bit 24..31
        w132WindSpeedData.windSpeed = (float)((value >> 24 & 0b11111111) * 0.2);
        w132WindSpeedData.windSpeedKmh = (float)(w132WindSpeedData.windSpeed * 3.6);

        if (w132TempHumidityData.valid > 0)
        {
          w132TempHumidityData.tempf = (w132TempHumidityData.temperature * 9.0) / 5.0 + 32.0; // Farenheit

          w132WindSpeedData.dewpoint = getDewPoint(w132TempHumidityData.temperature, w132TempHumidityData.humidity);
          w132WindSpeedData.dewptf = (w132TempHumidityData.dewpoint * 9.0) / 5.0 + 32.0; // Farenheit
    
          w132WindSpeedData.spreadDewpoint = (w132TempHumidityData.temperature - w132TempHumidityData.dewpoint);
          w132WindSpeedData.spreadDewptf = (w132TempHumidityData.tempf - w132TempHumidityData.dewptf);
          
          w132WindSpeedData.heatindex = getHeatIndex(w132TempHumidityData.temperature, w132TempHumidityData.humidity);
          w132WindSpeedData.heatindexf = (w132TempHumidityData.heatindex * 9.0) / 5.0 + 32.0; // Farenheit

          w132WindSpeedData.windchill = getWindChill(w132TempHumidityData.temperature, w132WindSpeedData.windSpeedKmh);  
          w132WindSpeedData.windchillf = (w132TempHumidityData.windchill * 9.0) / 5.0 + 32.0; // Farenheit 
        }
        else
        {
          w132WindSpeedData.dewpoint = INVALID_FLOAT_VALUE;
          w132WindSpeedData.dewptf = INVALID_FLOAT_VALUE;
          w132WindSpeedData.spreadDewpoint = INVALID_FLOAT_VALUE;
          w132WindSpeedData.spreadDewptf = INVALID_FLOAT_VALUE;
          w132WindSpeedData.heatindex = INVALID_FLOAT_VALUE;
          w132WindSpeedData.heatindexf = INVALID_FLOAT_VALUE;
          w132WindSpeedData.windchill = INVALID_FLOAT_VALUE;
          w132WindSpeedData.windchillf = INVALID_FLOAT_VALUE;
        }
      }
      else if ((value >> 12 & 0b111) == 7) // bit 12..14, = 111 ==> Wind Direction & Gust
      {
        bitSet(ventusStatus, BIT_VENTUS_STATUS_W132_WINDDIRGUST_RECEIVED);
        w132WindDirGustData.received = 1;
        w132WindDirGustData.measurementTimestamp = getTimestamp();
        w132WindDirGustData.windBatteryStatus = batteryLow;

        // Wind Direction (grad)
        unsigned int windDir = (value >> 15 & 0b111111111); // bit 15..23
        windDir = windDir + W132_WINDDIR_OFFSET;
        if (windDir > 360)
        {
          windDir = windDir - 360;
        }
        else if (windDir == 360)
        {
          windDir = 0;
        }
        w132WindDirGustData.windDir = windDir;
        w132WindDirGustData.windDirCardinalPoints = determineCardinalPoints(w132WindDirGustData.windDir);
  
        // Wind Gust (m/s), bit 24..31
        w132WindDirGustData.windGust = (float)((value >> 24 & 0b11111111) * 0.2);
        w132WindDirGustData.windGustKmh = (float)(w132WindDirGustData.windGust * 3.6);
      }
    }

    return true;
  #endif    

  return true;
}

//***************************************************************
// Publish W132 results to broker
//***************************************************************
boolean w132PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_W132 > 0)
    // Broker topic
    char publishJson[256];
  
    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
   
    // Temperature and humudity
    if (w132TempHumidityData.received == 1)
    {
      jsonBuffer["dateTime"] = w132TempHumidityData.measurementTimestamp;
      jsonBuffer["outTemp"] = round(w132TempHumidityData.temperature * 100) / 100;
      jsonBuffer["tempf"] = round(w132TempHumidityData.tempf * 100) / 100;
      jsonBuffer["outHumidity"] = round(w132TempHumidityData.humidity * 100) /100;

      if (w132TempHumidityData.dewpoint != INVALID_FLOAT_VALUE)
      {
        jsonBuffer["dewpoint"] = round(w132TempHumidityData.dewpoint * 100) /100;
        jsonBuffer["dewptf"] = round(w132TempHumidityData.dewptf * 100) /100;
        jsonBuffer["spreadDewpint"] = round(w132TempHumidityData.spreadDewpoint * 100) /100;
        jsonBuffer["spreadDewptf"] = round(w132TempHumidityData.spreadDewptf * 100) /100;
      }
      if (w132TempHumidityData.heatindex != INVALID_FLOAT_VALUE)
      {
        jsonBuffer["heatindex"] = round(w132TempHumidityData.heatindex * 100) /100;
        jsonBuffer["heatindexf"] = round(w132TempHumidityData.heatindexf * 100) /100;
      }
      if (w132TempHumidityData.windchill != INVALID_FLOAT_VALUE)
      {
        jsonBuffer["windchill"] = round(w132TempHumidityData.windchill * 100) /100;
        jsonBuffer["windchillf"] = round(w132TempHumidityData.windchillf * 100) /100;
      }

      jsonBuffer["temperatureTrend"] = w132TempHumidityData.trend;
      jsonBuffer["windBatteryStatus"] = w132TempHumidityData.windBatteryStatus;

      serializeJson(jsonBuffer, publishJson);
      if (!sensorPublish(String(W132_SENSOR_ID), publishJson, false))
      {
        return false;
      }
      
      w132TempHumidityData.published = 1;
      w132TempHumidityData.received = 0;
    }

    // Wind speed
    if (w132WindSpeedData.received == 1)
    {
      jsonBuffer["dateTime"] = w132WindSpeedData.measurementTimestamp;
      jsonBuffer["windSpeed"] = w132WindSpeedData.windSpeed;
      jsonBuffer["windSpeedKmh"] = w132WindSpeedData.windSpeedKmh;
      jsonBuffer["windBatteryStatus"] = w132WindSpeedData.windBatteryStatus;

      if (w132WindSpeedData.dewpoint != INVALID_FLOAT_VALUE)
      {
        jsonBuffer["dewpoint"] = round(w132WindSpeedData.dewpoint * 100) /100;
        jsonBuffer["dewptf"] = round(w132WindSpeedData.dewpoint * 100) /100;
        jsonBuffer["spreadDewpoint"] = round(w132WindSpeedData.spreadDewpoint * 100) /100;
        jsonBuffer["spreadDewptf"] = round(w132WindSpeedData.spreadDewptf * 100) /100;
      }
      if (w132WindSpeedData.heatindex != INVALID_FLOAT_VALUE)
      {
        jsonBuffer["heatindex"] = round(w132WindSpeedData.heatindex * 100) /100;
        jsonBuffer["heatindexf"] = round(w132WindSpeedData.heatindexf * 100) /100;
      }
      if (w132WindSpeedData.windchill != INVALID_FLOAT_VALUE)
      {
        jsonBuffer["windchill"] = round(w132WindSpeedData.windchill * 100) /100;
        jsonBuffer["windchillf"] = round(w132WindSpeedData.windchillf * 100) /100;
      }

      serializeJson(jsonBuffer, publishJson);
      if (!sensorPublish(String(W132_SENSOR_ID), publishJson, false))
      {
        return false;
      }
      
      w132WindSpeedData.published = 1;
      w132WindSpeedData.received = 0;
    }

    // Wind direction and wind gust
    if (w132WindDirGustData.received == 1)
    {
      jsonBuffer["dateTime"] = w132WindDirGustData.measurementTimestamp;
      jsonBuffer["windGust"] = w132WindDirGustData.windGust;
      jsonBuffer["windGustKmh"] = w132WindDirGustData.windGustKmh;
      jsonBuffer["windDir"] = w132WindDirGustData.windDir;
      jsonBuffer["windDirCardinalPoint"] = w132WindDirGustData.windDirCardinalPoints;
      jsonBuffer["windBatteryStatus"] = w132WindDirGustData.windBatteryStatus;

      serializeJson(jsonBuffer, publishJson);
      if (!sensorPublish(String(W132_SENSOR_ID), publishJson, false))
      {
        return false;
      }

      w132WindDirGustData.published = 1;
      w132WindDirGustData.received = 0;
    }

    return true;
  #endif

  return true;
}

//***************************************************************
// Start Interrupt to receive W174 datas
//***************************************************************
void w174Start()
{
  #if (WITH_W174 > 0)
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED) == 1)
    {
      debugln("Start W174.");
      w174LastDataReceived = 0;
      w174Data.received = 0;
      w174Data.published = 0;
      if ((getTimestamp() - w174Data.measurementTimestamp) > W174_MAX_OFFLINE)
      {
        bitSet(ventusStatus, BIT_VENTUS_STATUS_W174_IS_REQUIRED);
        debugln("W174 Rain data are required!");
      }
      bitClear(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED);
      attachInterrupt(digitalPinToInterrupt(W174_DATAPIN), w174InterruptHandler, CHANGE);
    }
  #else
    bitSet(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED);
  #endif
}

//***************************************************************
// Stop Interrupt to receive W174 datas
//***************************************************************
void w174Stop()
{
  #if (WITH_W174 > 0)
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED) != 1)
    {
      detachInterrupt(digitalPinToInterrupt(W174_DATAPIN)); // Interrupts off
      bitSet(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED);
      bitClear(ventusStatus, BIT_VENTUS_STATUS_W174_IS_REQUIRED);
      debugln("W174 stopped!");
    }
  #else
    bitSet(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED);
  #endif
}

//***************************************************************
// Setup W174 sensor
//***************************************************************
void w174Setup()
{
  #if (WITH_W174 > 0)
    debugln("Initialize W174.");
    pinMode(W174_DATAPIN, INPUT);
  #endif
}

//***************************************************************
// check if w174 received crc is correct for received value
//***************************************************************
boolean w174CRCValid(unsigned long value, byte checksum)
{
  #if (WITH_W174 > 0)
    byte calculatedChecksum = 0x7;
    for (int i = 0 ; i < 8 ; i++) calculatedChecksum += (byte)(value >> (i * 4));
    calculatedChecksum &= 0xF;
    return (calculatedChecksum == checksum);
  #endif
}

//***************************************************************
// Handle W174 received data telegram
//***************************************************************
ICACHE_RAM_ATTR void w174InterruptHandler()
{
  #if (WITH_W174 > 0)
    static long w174LineUp, w174LineDown;
    static unsigned long w174DataBits = 0;
    static byte w174CRCBits = 0;
    static byte w174BitsCounted = 0;
    long w174ZeroTime, w174OneTime;
  
    byte w174State = digitalRead(W174_DATAPIN); // current pin state
  
    if (w174State) // pin is now HIGH
    {
      w174LineUp = micros(); // line went HIGH after being LOW at this time
      w174ZeroTime = w174LineUp - w174LineDown; // calculate the LOW pulse time
      if (w174ZeroTime > VENTUS_SYNC - VENTUS_TOL && w174ZeroTime < VENTUS_SYNC + VENTUS_TOL)
      {
        // new start message
        w174DataBits = 0;
        w174CRCBits = 0;
        w174BitsCounted = 0;
      }
      else if (w174ZeroTime > VENTUS_ONE - VENTUS_TOL && w174ZeroTime < VENTUS_ONE + VENTUS_TOL)
      { // set the one bits
        if (w174BitsCounted < VENTUS_DATALEN)
          bitSet(w174DataBits, w174BitsCounted);
        else
          bitSet(w174CRCBits, w174BitsCounted - VENTUS_DATALEN);
        w174BitsCounted++;
      }
      else if (w174ZeroTime > VENTUS_ZERO - VENTUS_TOL && w174ZeroTime < VENTUS_ZERO + VENTUS_TOL)
      { // setting zero bits is not necessary, but count them
        w174BitsCounted++;
      }
      else // received bit is not a SYNC, ONE or ZERO bit, so restart
      {
        w174DataBits = 0;
        w174CRCBits = 0;
        w174BitsCounted = 0;
      }
  
      if (w174BitsCounted >= VENTUS_DATALEN + VENTUS_CRCLEN) // all bits received
      {
        // write valid value to buffer
        if (w174CRCValid(w174DataBits, w174CRCBits))
        {
          w174fifoWrite(w174DataBits);
        }
        else
        {
          w174fifoWrite(0); // write 0 to FIFO buffer (0 = invalid value received)
        }
        w174DataBits = 0;
        w174CRCBits = 0;
        w174BitsCounted = 0;
      }
    }
    else
    { // High values have no information with them
      w174LineDown = micros(); // line went LOW after being HIGH
      w174OneTime = w174LineDown - w174LineUp; // calculate the HIGH pulse time
    }
  #endif
}

//***************************************************************
// writes W174 received data telegram into fifo buffer
//***************************************************************
void w174fifoWrite(long item)
// write item into ring buffer
{
  #if (WITH_W174 > 0)
    w174fifoBuf[w174fifoWriteIndex] = item; // store the item
    if (!(w174fifoWriteIndex + 1 == w174fifoReadIndex || (w174fifoWriteIndex + 1 >= W174_FIFOSIZE && w174fifoReadIndex == 0)))
      w174fifoWriteIndex++;  // advance write pointer in ringbuffer
    if (w174fifoWriteIndex >= W174_FIFOSIZE) w174fifoWriteIndex = 0; // ring buffer is at its end
  #endif
}

//***************************************************************
// reads W174 received data telegram from fifo buffer
//***************************************************************
unsigned long w174fifoRead()
// always check first if item is available with fifoAvailable()
// before reading the ring buffer using this function
{
  #if (WITH_W174 > 0)
    detachInterrupt(digitalPinToInterrupt(W174_DATAPIN)); // Interrupts off while changing the read pointer for the ringbuffer
    unsigned long item;
    item = w174fifoBuf[w174fifoReadIndex];
    w174fifoBuf[w174fifoReadIndex] = 0;
    w174fifoReadIndex++;
    if (w174fifoReadIndex >= W174_FIFOSIZE) w174fifoReadIndex = 0;
    attachInterrupt(digitalPinToInterrupt(W174_DATAPIN), w174InterruptHandler, CHANGE); // Interrupts on again
    return item;
  #endif

  return 0;
}

//***************************************************************
// Returns if fifo buffer contain data
//***************************************************************
boolean w174fifoAvailable()
// item is available for reading if (fifoReadIndex!=fifoWriteIndex)
{
  #if (WITH_W174 > 0)
    return (w174fifoReadIndex != w174fifoWriteIndex);
  #endif
}

//***************************************************************
// Prints W174 datas to serial
//***************************************************************
void w174PrintResults(unsigned long value)
{
  #if (WITH_W174 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("*********************** W174 ***************************");
    debug("Telegram: ");
    for (byte i = 0; i < 32; i++)
    {
      debug(value >> i & 0b1);
    }
    debugln();
    
    debug("Timestamp..: ");
    debugln(w174Data.measurementTimestamp);
    debug("Battery Low: ");
    debugln(w174Data.rainBatteryStatus);
    debug("RAW........: ");
    debugln(w174Data.rainCounter);
    debug("Rain.......: ");
    debugln(w174Data.rain);
    debug("Rain total.: ");
    debugln(w174Data.rainTotal);
    debugln("********************************************************");
    debugln();
  #endif
}

//***************************************************************
// Decode received W174 data telegram
//***************************************************************
boolean w174DecodeResults(unsigned long value)
{
  #if (WITH_W174 > 0)
    w174Data.received = 1;
    w174Data.measurementTimestamp = getTimestamp();
  
    // Battery State
    w174Data.rainBatteryStatus = (value >> 8 & 0b1); // bit 8 is set if battery voltage is low

    // rain counter
    int rainCounter = (value >> 16 & 0b1111111111111111); // bits 16..31

    // Rain Total (mm)
    w174Data.rainTotal = (value >> 16 & 0b1111111111111111) * W174_BUCKET_SIZE; // bits 16..31

    // Rain Diff (mm)
    w174Data.rain = 0;
    if (w174Data.rainCounter >= 0)
    {
      if (rainCounter > w174Data.rainCounter)
      {
        w174Data.rain = (rainCounter - w174Data.rainCounter) * W174_BUCKET_SIZE;
      }
    }
    
    w174Data.rainCounter = rainCounter;
  #endif

  return true;
}

//***************************************************************
// Publish W174 results to broker
//***************************************************************
boolean w174PublishResults()
{ 
  #if (WITH_BROKER && WITH_W174 > 0)
    if (w174Data.received > 0)
    {
      // Broker topic
      char publishJson[512];
    
      // Create JsonBuffer
      StaticJsonDocument<256> jsonBuffer;

      jsonBuffer["dateTime"] = w174Data.measurementTimestamp;
      jsonBuffer["rain"] = w174Data.rain;
      jsonBuffer["rainCounter"] = w174Data.rainCounter;
      jsonBuffer["rainTotal"] = w174Data.rainTotal;
      jsonBuffer["rainBatteryStatus"] = w174Data.rainBatteryStatus;
    
      serializeJson(jsonBuffer, publishJson);
      
      if (sensorPublish(String(W174_SENSOR_ID), publishJson, false))
      {
        w174Data.published = 1;
        w174Data.received = 0;
        return true;
      }

      return false;
    }
  #endif

  return true;
}

//***************************************************************
// TSL2591 Sensor setup
//***************************************************************
void tsl2591Setup()
{
  #if WITH_TSL2591 > 0
    debugln("Starting Adafruit TSL2591 Test...");
    if (tsl2591.begin()) 
    {
      debugln("Found a TSL2591 sensor.");
    } 
    else 
    {
      debugln("No sensor found ... check your wiring!");
      while (1);
    }

    sensor_t sensor;
    tsl2591.getSensor(&sensor);
    debugln("------------------------------------");
    debug("Sensor:       "); debugln(sensor.name);
    debug("Driver Ver:   "); debugln(sensor.version);
    debug("Unique ID:    "); debugln(sensor.sensor_id);
    debug("Max Value:    "); debug(sensor.max_value); debugln(" lux");
    debug("Min Value:    "); debug(sensor.min_value); debugln(" lux");
    debug("Resolution:   "); debug(sensor.resolution, 4); debugln(" lux");  
    debugln("------------------------------------");
    debugln();
    delay(500);

    // You can change the gain on the fly, to adapt to brighter/dimmer light situations
    //tsl2591.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
    tsl2591.setGain(TSL2591_GAIN_MED);  // 25x gain
    //tsl2591.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
    // Changing the integration time gives you a longer time over which to sense light
    // longer timelines are slower, but are good in very low light situtations!
    // tsl2591.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    // tsl2591.setTiming(TSL2591_INTEGRATIONTIME_200MS);
    tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
    // tsl2591.setTiming(TSL2591_INTEGRATIONTIME_400MS);
    // tsl2591.setTiming(TSL2591_INTEGRATIONTIME_500MS);
    // tsl2591.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

    /* Display the gain and integration time for reference sake */  
    debugln("------------------------------------");
    debug("Gain:         ");
    tsl2591Gain_t gain = tsl2591.getGain();
    switch(gain)
    {
      case TSL2591_GAIN_LOW:
        debugln(F("1x (Low)"));
        break;
      case TSL2591_GAIN_MED:
        debugln(F("25x (Medium)"));
        break;
      case TSL2591_GAIN_HIGH:
        debugln(F("428x (High)"));
        break;
      case TSL2591_GAIN_MAX:
        debugln(F("9876x (Max)"));
        break;
    }
    debug("Timing:       ");
    debug((tsl2591.getTiming() + 1) * 100); 
    debugln(" ms");
    debugln("------------------------------------");
    debugln();
  #endif
}

//***************************************************************
// Sensor TSL2591 print data to serial
//***************************************************************
void tsl2591PrintResults()
{
  #if (WITH_TSL2591 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
    // That way you can do whatever math and comparisons you want!
    uint32_t lum = tsl2591.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
  
    debugln("****************** TLS2591 *******************");
    debug("IR.........: "); debugln(ir);
    debug("Full.......: "); debugln(full);
    debug("Visible....: "); debugln(full - ir);
    debug("Luminosity.: "); debugln(tsl2591.calculateLux(full, ir));
    debugln("***********************************************");
    debugln();
    delay(500);
  #endif
}

//***************************************************************
// Sensor TSL2591 publish data to broker
//***************************************************************
boolean tsl2591PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_TSL2591 > 0)
    // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
    // That way you can do whatever math and comparisons you want!
    uint32_t lum = tsl2591.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    
    tsl2591Data.measurementTimestamp = getTimestamp();
    tsl2591Data.luminosity = (float)tsl2591.calculateLux(full, ir);
    tsl2591Data.received = 1;
  
    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    
    // Add sensor datas
    jsonBuffer["dateTime"] = bh1750Data.measurementTimestamp;
    jsonBuffer["luminosity"] = tsl2591Data.luminosity;
    
    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if (sensorPublish(String(TSL2591_SENSOR_ID), publishJson, false))
    {
      tsl2591Data.published = 1;
      return true;
    }
    
    return false;
  #endif
  
  return true;
}
//***************************************************************
// BH1750 Sensor setup
//***************************************************************
void bh1750Setup()
{
  #if WITH_BH1750 > 0
    bh1750Data.received = 0;
    bh1750Data.published = 0;
  
    debugln("Initialize BH1750.");
    Wire.begin();
    LightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

    // After the measurement the MTreg value is changed according to the result:
    // lux > 40000 ==> MTreg =  32
    // lux < 40000 ==> MTreg =  69  (default)
    // lux <    10 ==> MTreg = 138
    // based on Code: https://github.com/claws/BH1750/tree/master/examples/BH1750autoadjust
    
    debugln("BH1750 Test begin");
    //we use here the maxWait option due fail save
    float luminosity = LightSensor.readLightLevel(true);
    debug("Luminosity: ");
    debug(luminosity);
    debugln(" lux");

    if (luminosity < 0)
    {
      debugln("BH1750: Error condition detected!");
      bh1750LevelSetup = "error";
    }
    else if (luminosity > 40000.0) 
    {
      // reduce measurement time - needed in direct sun light
      if (LightSensor.setMTreg(32))
      {
        debugln("BH1750: Setting MTReg to low value for high light environment.");
        bh1750LevelSetup = "high";
      }
      else 
      {
        debugln("BH1750: Error setting MTReg to low value for high light environment!");
        bh1750LevelSetup = "high error";
      }
    }
    else if (luminosity > 10.0) 
    {
      // typical light environment
      if (LightSensor.setMTreg(69))
      {
        debugln("BH1750: Setting MTReg to default value for normal light environment.");
        bh1750LevelSetup = "normal";
      }
      else
      {
        debugln("BH1750: Error setting MTReg to default value for normal light environment!");
        bh1750LevelSetup = "normal error";
      }
    }
    else if (luminosity <= 10.0) 
    {
      //very low light environment
      if (LightSensor.setMTreg(138))
      {
        debugln("BH1750: Setting MTReg to high value for low light environment.");
        bh1750LevelSetup = "low";
      }
      else
      {
        debugln("BH1750: Error setting MTReg to high value for low light environment!");
        bh1750LevelSetup = "low error";
      }
    }
  #endif

  #if WITH_BH1750B > 0
    debugln("Initialize BH1750b.");
    Wire.begin();
    Wire.beginTransmission (BH1750_Device);
    Wire.write(0x10); // Set resolution to 1 Lux
    Wire.endTransmission ();
    delay(200);
    debugln("BH1750b Sensor initialized.");
  #endif

  #if WITH_BH1750AS > 0
    debugln("Initialize BH1750AS.");
    Wire.begin();
    if(ASLightSensor.begin(RESOLUTION_AUTO_HIGH, true))
    {
      debugln("BH1750AS Sensor initialized.");
    } 
    else 
    {
      debugln("BH1750AS: Error condition detected!");
    }
  #endif
}

//***************************************************************
// Sensor BH1750b reading
//***************************************************************
unsigned int BH1750_Read()
{
  #if WITH_BH1750B > 0
    unsigned int i=0;
    Wire.beginTransmission (BH1750_Device);
    Wire.requestFrom (BH1750_Device, 2);
    while (Wire.available())
    {
      i <<=8;
      i|= Wire.read();
    }
    Wire.endTransmission();
    return i/1.2; // Convert to Lux
  #endif    
}

//***************************************************************
// Sensor BH1750 print data to serial
//***************************************************************
void bh1750PrintResults()
{
  #if (WITH_BH1750 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("******************* BH1750 *******************");
    debug("Luminosity: ");
    debugln(LightSensor.readLightLevel());
    debug("Setup.....: ");
    debugln(bh1750LevelSetup);
    debugln("*********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor BH1750b print data to serial
//***************************************************************
void bh1750bPrintResults()
{
  #if (WITH_BH1750B > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("****************** BH1750b *******************");

    int i;
    luminosity = BH1750_Read();
    FtCd = luminosity/10.764;
    Wattsm2 = luminosity/683.0;

    debug("Luminosity: ");
    debug(luminosity);
    debugln(" lx");
    debug("FtCd......: ");
    debug(FtCd);
    debugln(" FC");
    debug("Radiation.: ");
    debug(Wattsm2);
    debugln(" W/m^2");
    delay (100);
    debugln("**********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor BH1750 print data to serial
//***************************************************************
void bh1750asPrintResults()
{
  #if (WITH_BH1750AS > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("****************** BH1750AS *******************");
    debug("Luminosity: ");
    debugln(ASLightSensor.readLightLevel());
    debugln("**********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor BH1750 publish data to broker
//***************************************************************
boolean bh1750PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_BH1750 > 0)
    bh1750Data.measurementTimestamp = getTimestamp();
    bh1750Data.luminosity = LightSensor.readLightLevel();
    bh1750Data.received = 1;

    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    
    // Add sensor datas
    jsonBuffer["dateTime"] = bh1750Data.measurementTimestamp;
    jsonBuffer["luminosity"] = bh1750Data.luminosity;
    // jsonBuffer["setup"] = bh1750LevelSetup;
    
    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if (sensorPublish(String(BH1750_SENSOR_ID), publishJson, false))
    {
      bh1750Data.published = 1;
      return true;
    }
    return false;
  #endif

  return true;
}

//***************************************************************
// Sensor BH1750b publish data to broker
//***************************************************************
boolean bh1750bPublishResults()
{
  #if (WITH_BROKER > 0 && WITH_BH1750B > 0)
    bh1750Data.measurementTimestamp = getTimestamp();
    bh1750Data.luminosity = BH1750_Read();
    bh1750Data.fc =  bh1750Data.luminosity/10.764;
    bh1750Data.radiation = bh1750Data.luminosity/683.0;
    bh1750Data.received = 1;
  
     // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    
    // Add sensor datas
    jsonBuffer["dateTime"] = bh1750Data.measurementTimestamp;
    jsonBuffer["luminosity"] = bh1750Data.luminosity;
    jsonBuffer["fc"] = bh1750Data.fc;
    jsonBuffer["radiation"] = bh1750Data.radiation;
    
    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if (sensorPublish(String(BH1750B_SENSOR_ID), publishJson, false))
    {
      bh1750Data.published = 1;
      return true;
    }
    
    return false;
  #endif

  return true;
}

//***************************************************************
// Sensor BH1750AS publish data to broker
//***************************************************************
boolean bh1750asPublishResults()
{
  #if (WITH_BROKER > 0 && WITH_BH1750AS > 0)
    bh1750Data.measurementTimestamp = getTimestamp();
    bh1750Data.luminosity = ASLightSensor.readLightLevel();
    bh1750Data.received = 1;
  
    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    
    // Add sensor datas
    jsonBuffer["dateTime"] = bh1750Data.measurementTimestamp;
    jsonBuffer["luminosity"] = bh1750Data.luminosity;
    
    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if (sensorPublish(String(BH1750AS_SENSOR_ID), publishJson, false))
    {
      bh1750Data.published = 1;
      return true;
    }
    
    return false;
  #endif
  
  return true;
}

//***************************************************************
// BME280 Sensor setup
//***************************************************************
void bme280Setup()
{
  #if (WITH_BME280 > 0)
    debugln("Initialize BME280.");
    bme280Data.received = 0;
    bme280Data.published = 0;
    
    Wire.begin(); // D1 connected to SCL, D2 connected to SDA
    Wire.setClock(100000);
    if (!bme280.begin(0x76))
    {
      debugln("Could not find a valid BME280 sensor, check wiring!");
      while (1);
    }
    
    // Weather Station Scenario
    // forced mode, 1x temperature / 1x humidity / 1x pressure oversampling, filter off

    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);

    debugln("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    debugln("filter off");
    debugln();    

  #endif
}

//***************************************************************
// Sensor BME280 print data to serial
//***************************************************************
void bme280PrintResults()
{
  #if (WITH_BME280 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    bme280.takeForcedMeasurement();
    delay(10);
    
    float pressure = (bme280.readPressure() / 100.0F) + BME280_PRESSURE_OFFSET;
    float temperature = bme280.readTemperature() + BME280_TEMPERATURE_OFFSET;
    float humidity = bme280.readHumidity() + BME280_HUMIDITY_OFFSET;
    if (humidity > 100.0)
    {
      humidity = 100.0;
    }

    debugln();   
    debugln("******************* BME280 *******************");

    debugln("Sensor values");
    
    debug("  Temperature......: ");
    debug(temperature);
    debugln(" °C");

    debug("  Humidity.........: ");
    debug(humidity);
    debugln(" %");
    
    debug("  Pressure.........: ");
    debug(pressure);
    debugln(" hPa");

    debug("  Approx. Altitude.: ");
    debug(bme280.readAltitude(pressureSealevel));
    debugln(" m");

    debugln("Calculated values");

    debug("  Temperature.........: ");
    float tempf = (temperature * 9.0) / 5.0 + 32;
    debug(tempf);
    debugln(" °F");

    debug("  Pressure(absbaromin): ");
    debug(pressure * 0.02953);
    debugln(" inch");

    debug("  Barometer...........: ");
    float barometer = getBarometer(pressure);
    debug(barometer);
    debugln(" hPa");

    debug("  Barometer(baromin)..: ");
    debug(barometer * 0.02953);
    debugln(" inch");

    debug("  Dewpoint............: ");
    float dewpoint = getDewPoint(temperature, humidity);
    debug(dewpoint);
    debugln(" °C");

    debug("  Dewpoint(dewptf)....: ");
    float dewptf = (dewpoint * 9.0) / 5.0 + 32.0;
    debug(dewptf);
    debugln(" °F");

    // Calculate dewpoint spread (difference between actual temp and dewpoint -> the smaller the number: rain or fog

    debug("  Dewpoint Spread.....: ");
    debug(temperature - dewpoint);
    debugln(" °C");

    debug("  Dewptf Spread.......: ");
    debug(tempf - dewptf);
    debugln(" °F");

    debug("  Heatindex...........: ");
    debug(getHeatIndex(temperature, humidity));
    debugln(" °C");

    debug("  HeatindexF..........: ");
    debug((getHeatIndex(temperature, humidity) * 9.0) / 5.0 + 32.0);
    debugln(" °F");

    debugln("**********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor BME280 publish data to broker
//***************************************************************
boolean bme280PublishResults()
{
  #if(WITH_BROKER > 0 && WITH_BME280 > 0)
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;

    bme280.takeForcedMeasurement();
    delay(10);
    
    bme280Data.received = 1;
    bme280Data.measurementTimestamp = getTimestamp();
    bme280Data.temperature = bme280.readTemperature() + BME280_TEMPERATURE_OFFSET; // Celsius
    bme280Data.tempf = (bme280Data.temperature * 9.0) / 5.0 + 32.0; // Farenheit
    bme280Data.pressure = (bme280.readPressure() / 100.0F) + BME280_PRESSURE_OFFSET;
    bme280Data.absbaromin = bme280Data.pressure * 0.02953; // inch
    bme280Data.barometer = (int)(getBarometer(bme280Data.pressure)+.5); // hpa
    bme280Data.baromin = bme280Data.barometer * 0.02953; // inch
    bme280Data.humidity = bme280.readHumidity() + BME280_HUMIDITY_OFFSET; // %
    if (bme280Data.humidity > 100)
    {
      bme280Data.humidity = 100;
    }
    bme280Data.dewpoint = getDewPoint(bme280Data.temperature, bme280Data.humidity); // Celsius
    bme280Data.dewptf = (bme280Data.dewpoint * 9.0) / 5.0 + 32.0; // Farenheit
    bme280Data.spreadDewpoint = bme280Data.temperature - bme280Data.dewpoint; // Celsius
    bme280Data.spreadDewptf = bme280Data.tempf - bme280Data.dewptf; // Farenheit
    bme280Data.heatindex = getHeatIndex(bme280Data.temperature, bme280Data.humidity); // Celsius
    bme280Data.heatindexf = (bme280Data.heatindex * 9.0) / 5.0 + 32.0; // Farenheit
    bme280Data.approxAltitude = bme280.readAltitude(pressureSealevel);

    jsonBuffer["dateTime"] = bme280Data.measurementTimestamp;
    jsonBuffer["outTemp"] = (round(bme280Data.temperature * 100) / 100);
    jsonBuffer["tempf"] = (round(bme280Data.tempf * 100) / 100);
    jsonBuffer["outHumidity"] = round(bme280Data.humidity * 100) / 100;
    jsonBuffer["pressure"] = round(bme280Data.pressure * 100) / 100;
    jsonBuffer["baromin"] = round(bme280Data.baromin * 100) / 100;
    jsonBuffer["barometer"] = round(bme280Data.barometer * 100) / 100;
    jsonBuffer["absbaromin"] = round(bme280Data.absbaromin * 100) / 100;
    jsonBuffer["dewpoint"] = round(bme280Data.dewpoint * 100) / 100;
    jsonBuffer["dewptf"] = round(bme280Data.dewptf * 100) / 100;
    jsonBuffer["spreadDewpoint"] = round(bme280Data.spreadDewpoint * 100) / 100;
    jsonBuffer["spreadDewptf"] = round(bme280Data.spreadDewptf * 100) / 100;
    jsonBuffer["heatindex"] = round(bme280Data.heatindex * 100) / 100;
    jsonBuffer["heatindexf"] = round(bme280Data.heatindexf * 100) / 100;
    jsonBuffer["approxAltitude"] = round(bme280Data.approxAltitude * 100) / 100;
  
    char publishJson[1024];
    serializeJson(jsonBuffer, publishJson);

    if(sensorPublish(String(BME280_SENSOR_ID), publishJson, false))
    {
      bme280Data.published = 1;
      return true;
    }

    return false;
  #endif
  
  return true;
}

//***************************************************************
// Sensor BME680 setup
//***************************************************************
void bme680Setup()
{
  #if WITH_BME680 > 0
    debugln("Initialize BME680.");
    bme680Data.received = 0;
    bme680Data.published = 0;

    Wire.begin();
    if (!bme680.begin(0x77))
    {
      debugln("Could not find a valid BME680 sensor, check wiring!");
      while (1);
    }
  
    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150); // 320*C for 150 ms
  #endif
}

//***************************************************************
// Sensor BME680 print data to serial
//***************************************************************
boolean bme680PrintResults()
{
  #if (WITH_BME680 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("******************* BME680 *******************");
  
    debugln("reading BME680...");
    if (!bme680.performReading())
    {
      debugln("Failed to perform reading BME680!");
      return false;
    }

    float pressure = (bme680.pressure / 100.0F) + BME680_PRESSURE_OFFSET;
    float temperature = bme680.temperature + BME680_TEMPERATURE_OFFSET;
    float humidity = bme680.humidity + BME680_HUMIDITY_OFFSET;
    float gas = bme680.gas_resistance / 1000.0;
    if (humidity > 100.0)
    {
      humidity = 100.0;
    }
    
    debugln("reading BME680 finished.");
    debugln("Sensor values");
    
    debug("  Temperature......: ");
    debug(temperature);
    debugln(" °C");

    debug("  Humidity.........: ");
    debug(humidity);
    debugln(" %");
    
    debug("  Pressure.........: ");
    debug(pressure);
    debugln(" hPa");

    debug("  Gas..............: ");
    debug(gas);
    debugln(" KOhms");

    debug("  Approx. Altitude.: ");
    debug(bme280.readAltitude(pressureSealevel));
    debugln(" m");

    debugln("Calculated values");

    debug("  Temperature.........: ");
    float tempf = (temperature * 9.0) / 5.0 + 32;
    debug(tempf);
    debugln(" °F");

    debug("  Pressure(absbaromin): ");
    debug(pressure * 0.02953);
    debugln(" inch");

    debug("  Barometer...........: ");
    float barometer = getBarometer(pressure);
    debug(barometer);
    debugln(" hPa");

    debug("  Barometer(baromin)..: ");
    debug(barometer * 0.02953);
    debugln(" inch");

    debug("  Dewpoint............: ");
    float dewpoint = getDewPoint(temperature, humidity);
    debug(dewpoint);
    debugln(" °C");

    debug("  Dewpoint(dewptf)....: ");
    float dewptf = (dewpoint * 9.0) / 5.0 + 32.0;
    debug(dewptf);
    debugln(" °F");

    // Calculate dewpoint spread (difference between actual temp and dewpoint -> the smaller the number: rain or fog

    debug("  Dewpoint Spread.....: ");
    debug(temperature - dewpoint);
    debugln(" °C");

    debug("  Dewptf Spread.......: ");
    debug(tempf - dewptf);
    debugln(" °F");

    debug("  Heatindex...........: ");
    debug(getHeatIndex(temperature, humidity));
    debugln(" °C");

    debug("  HeatindexF..........: ");
    debug((getHeatIndex(temperature, humidity) * 9.0) / 5.0 + 32.0);
    debugln(" °F");

    debugln("**********************************************");
    debugln();
  #endif

  return true;
}

//***************************************************************
// Sensor BME680 publish data to broker
//***************************************************************
boolean bme680PublishResults()
{
  #if(WITH_BROKER > 0 && WITH_BME680 > 0)
    if (!bme680.performReading())
    {
      return false;
    }
    
    bme680Data.received = 1;
    bme680Data.measurementTimestamp = getTimestamp();
    bme680Data.temperature = bme680.temperature + BME680_TEMPERATURE_OFFSET;
    bme680Data.pressure = (bme680.pressure / 100.0F) + BME680_PRESSURE_OFFSET;
    bme680Data.barometer = (int)(getBarometer(bme680Data.pressure)+.5);
    bme680Data.humidity = bme680.humidity + BME280_HUMIDITY_OFFSET;
    if (bme680Data.humidity > 100)
    {
      bme680Data.humidity = 100;
    }
    bme680Data.dewpoint = getDewPoint(bme680Data.temperature, bme680Data.humidity);
    bme680Data.dewptf = (bme680Data.dewpoint * 9.0) / 5.0 + 32.0; // Farenheit
    bme680Data.spreadDewpoint = bme680Data.temperature - bme680Data.dewpoint;
    bme680Data.spreadDewptf = bme680Data.tempf - bme680Data.dewptf; // Farenheit
    bme680Data.heatindex = getHeatIndex(bme680Data.temperature, bme680Data.humidity);
    bme680Data.heatindexf = (bme680Data.heatindex * 9.0) / 5.0 + 32.0; // Farenheit
    bme680Data.gas = (bme680.gas_resistance / 1000.0);
    bme680Data.approxAltitude = bme680.readAltitude(pressureSealevel);

    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;
    
    jsonBuffer["dateTime"] = bme680Data.measurementTimestamp;
    jsonBuffer["outTemp"] = (round(bme680Data.temperature * 100) / 100);
    jsonBuffer["tempf"] = (round(bme680Data.tempf * 100) / 100);
    jsonBuffer["outHumidity"] = round(bme680Data.humidity * 100) / 100;
    jsonBuffer["pressure"] = round(bme680Data.pressure * 100) / 100;
    jsonBuffer["baromin"] = round(bme680Data.baromin * 100) / 100;
    jsonBuffer["barometer"] = round(bme680Data.barometer * 100) / 100;
    jsonBuffer["absbaromin"] = round(bme680Data.absbaromin * 100) / 100;
    jsonBuffer["dewpoint"] = round(bme680Data.dewpoint * 100) / 100;
    jsonBuffer["dewptf"] = round(bme680Data.dewptf * 100) / 100;
    jsonBuffer["spreadDewpoint"] = round(bme680Data.spreadDewpoint * 100) / 100;
    jsonBuffer["spreadDewptf"] = round(bme680Data.spreadDewptf * 100) / 100;
    jsonBuffer["heatindex"] = round(bme680Data.heatindex * 100) / 100;
    jsonBuffer["heatindexf"] = round(bme280Data.heatindexf * 100) / 100;
    jsonBuffer["gas"] = round(bme680Data.gas * 100) / 100;
    jsonBuffer["approxAltitude"] = round(bme680Data.approxAltitude * 100) / 100;
  
    char publishJson[1024];
    serializeJson(jsonBuffer, publishJson);

    if(sensorPublish(String(BME680_SENSOR_ID), publishJson, false))
    {
      bme680Data.published = 1;
      return true;
    }

    return false;
  #endif
  
  return true;
}

//***************************************************************
// Sensor MHRD init
//***************************************************************
void mhrdSetup()
{
  #if (WITH_MHRD > 0)
    mhrdData.received = 0;
    mhrdData.published = 0;
  #endif
}

//***************************************************************
// Sensor MHRD print data to serial
//***************************************************************
void mhrdPrintResults()
{
  #if (WITH_MHRD > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("******************* MH-RD ********************");
  
    // lowest and highest sensor readings:
    const int sensorMin = 0;     // sensor minimum
    const int sensorMax = 1024;  // sensor maximum
  
    // read the sensor on analog A0:
    int sensorReading = analogRead(A0);
    // map the sensor range (four options):
    // ex: 'long int map(long int, long int, long int, long int, long int)'
    int range = map(sensorReading, sensorMin, sensorMax, 0, 3);
  
    debug("Rain RAW........: ");
    debugln(sensorReading);
    debug("Rain mapped.....: ");
    debugln(range);
  
    // range value:
    switch (range)
    {
      case 0:    // Sensor getting wet
        debug("Rain transformed: ");
        debugln("Flood");
        break;
      case 1:    // Sensor getting wet
        debug("Rain transformed: ");
        debugln("Rain Warning");
        break;
      case 2:    // Sensor dry
        debug("Rain transformed: ");
        debugln("Not Raining");
        break;
      case 3:    // Sensor dry
        debug("Rain transformed: ");
        debugln("Not Raining");
        break;
    }
    debugln("**********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor MHRD publish data to broker
//***************************************************************
boolean mhrdPublishResults()
{
  #if(WITH_BROKER > 0 && WITH_MHRD > 0)
    // read the sensor on analog A0:
    mhrdData.rainState = analogRead(A0);
  
    // map the sensor range (four options):
    // ex: 'long int map(long int, long int, long int, long int, long int)'
    mhrdData.received = 1;
    mhrddData.measurementTimestamp = getTimestamp();

    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;

    // Add sensor datas
    jsonBuffer["dateTime"] = mhrddData.measurementTimestamp;
    jsonBuffer["rainState"] = mhrddData.rainState;

    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if sensorPublish(String(MHRD_SENSOR_ID), publishJson, false)
    {
      mhrdData.published = 1;
      return true;
    }

    return false;
  #endif
}

//***************************************************************
// ML8511 Sensor setup
//***************************************************************
void ml8511Setup()
{
  #if WITH_ML8511 > 0
    debugln("Initialize ML8511.");
    pinMode(ML8511_DATAPIN, INPUT);
    ml8511Data.received = 0;
    ml8511Data.published = 0;
  #endif
}

//***************************************************************
// Sensor ML8511 print data to serial
//***************************************************************
void ml8511PrintResults()
{
  #if (WITH_ML8511 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("******************* ML8511 *******************");
  
    int uvLevel = averageAnalogRead(ML8511_DATAPIN);
   
    float outputVoltage = 3.3 * uvLevel/1024;
    float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
   
    debug("UV Intensity: ");
    debug(uvIntensity);
    debugln(" mW/cm^2"); 
    debugln("**********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor ML8511 publish data to broker
//***************************************************************
boolean ml8511PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_ML8511 > 0)
    int uvLevel = averageAnalogRead(ML8511_DATAPIN);
   
    float outputVoltage = 3.3 * uvLevel/1024;
    ml8511Data.uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
    
    ml8511Data.received = 1;
    ml8511Data.measurementTimestamp = getTimestamp();

    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;  
    
    // Add sensor datas
    jsonBuffer["dateTime"] = ml8511Data.measurementTime;
    jsonBuffer["uvIntensity"] = ml8511Data.uvIntensity;

    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if sensorPublish(String(ML8511_SENSOR_ID), publishJson, false)
    {
      ml8511Data.published = 1;
      return true;
    }

    return false;
  #endif

  return true;
}

//***************************************************************
//
//***************************************************************
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//***************************************************************
//Takes an average of readings on a given pin
//Returns the average
//***************************************************************
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);  
}

//***************************************************************
// VEML6070 Sensor setup
//***************************************************************
void veml6070Setup()
{
  #if WITH_VEML6070 > 0
    debugln("Initialize VEML6070.");
    
    //VEML6070_HALF_T ~62.5ms
    //VEML6070_1_T ~125ms
    //VEML6070_2_T ~250ms
    //VEML6070_4_T ~500ms
    //https://www.vishay.com/docs/84310/designingveml6070.pdf

    Wire.begin();
    uv6070.begin(VEML6070_1_T);  // pass in the integration time constant

    veml6070Data.received = 0;
    veml6070Data.published = 0;
  #endif
}

//***************************************************************
// VEML6075 Sensor full setup
//***************************************************************
void veml6075FullSetup()
{
  #if WITH_VEML6075 > 0
    debugln("Initialize VEML6075.");

    Wire.begin();
    if (!uv6075.begin()) 
    {
      debugln("Failed to communicate with VEML6075 sensor, check wiring?");
    }
    debugln("Found VEML6075 sensor");

    // Set the integration constant
    uv6075.setIntegrationTime(VEML6075_100MS);
    
    // Get the integration constant and print it!
    debug("Integration time set to ");
    switch (uv6075.getIntegrationTime()) {
      case VEML6075_50MS: debug("50"); break;
      case VEML6075_100MS: debug("100"); break;
      case VEML6075_200MS: debug("200"); break;
      case VEML6075_400MS: debug("400"); break;
      case VEML6075_800MS: debug("800"); break;
    }
    debugln("ms");

    // Set the high dynamic mode
    uv6075.setHighDynamic(true);
    
    // Get the mode
    if (uv6075.getHighDynamic())
    {
      debugln("High dynamic reading mode");
    }
    else
    {
      debugln("Normal dynamic reading mode");
    }

    // Set the mode
    uv6075.setForcedMode(false);
    
    // Get the mode
    if (uv6075.getForcedMode())
    {
      debugln("Forced reading mode");
    }
    else
    {
      debugln("Continuous reading mode");
    }

    // Set the calibration coefficients
    uv6075.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                           2.95, 1.74,  // UVB_C and UVB_D coefficients
                           0.001461, 0.002591); // UVA and UVB responses

    veml6075Data.received = 0;
    veml6075Data.published = 0;
  #endif
}

//***************************************************************
// VEML6075 Sensor setup
//***************************************************************
void veml6075SimpleSetup()
{
  #if WITH_VEML6075 > 0
    debugln("Initialize VEML6075.");

    Wire.begin();
    if (!uv6075.begin()) 
    {
      debugln("Failed to communicate with VEML6075 sensor, check wiring?");
      while (1) { delay(100); }
    }
    debugln("Found VEML6075 sensor");

    veml6075Data.received = 0;
    veml6075Data.published = 0;
  #endif
}

//***************************************************************
// Sensor VEML6070 print data to serial
//***************************************************************
void veml6070PrintResults()
{
  #if (WITH_VEML6070 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    byte i, uvlevel;
    // https://www.vishay.com/docs/84310/designingveml6070.pdf
    short uv_mapping[5] = {2055, 1494, 1121, 561, 0};
    char *risk_mapping[] = {"extreme", "very high", "high", "moderate", "low"};
      
    debugln("****************** VEML6070 *******************");
  
    uvlevel = uv6070.readUV();
    debug("UV level: ");
    debugln(uvlevel);
  
    for (i = 0; i < 5; i++)
    {
      if (uvlevel >= uv_mapping[i])
      {
        break;
      }
    }
    
    debug("UV risk: ");
    debugln(risk_mapping[i]);
    debugln("***********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor VEML6075 print data to serial
//***************************************************************
void veml6075PrintResults()
{
  #if (WITH_VEML6075 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("****************** VEML6075 *******************");
    debug("Raw UVA reading:  "); debugln(uv6075.readUVA());
    debug("Raw UVB reading:  "); debugln(uv6075.readUVB());
    debug("UV Index reading: "); debugln(uv6075.readUVI());
    debugln("***********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor VEML6070 publish data to broker
//***************************************************************
boolean veml6070PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_VEML6070 > 0)
    byte i;
    
    // https://www.vishay.com/docs/84310/designingveml6070.pdf
    short level_mapping1[5] = {2055, 1494, 1121, 561, 0};
    char  *risk_mapping[] = {"extreme", "very high", "high", "moderate", "low"};

    short level_mapping2[12] = {2244, 2056, 1869, 1682, 1495, 1308, 1121, 938, 750, 561, 375, 188};
    short index_mapping[12] = {12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
    
    veml6070Data.uvLevel = uv6070.readUV();
    veml6070Data.uvRisk = "low";
    for (i = 0; i < 5; i++)
    {
      if (veml6070Data.uvLevel >= level_mapping1[i])
      {
        veml6070.uvRisk = risk_mapping[i];
        break;
      }
    }

    veml6070Data.UV = 0;
    for (i = 0; i < 12; i++)
    {
      if (veml6070Data.uvLevel >= level_mapping2[i])
      {
        veml6070Data.UV = index_mapping[i];
        break;
      }
    }

    veml6070Data.received = 1;
    veml6070Data.measurementTimestamp = getTimestamp();

    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;  
    
    // Add sensor datas
    jsonBuffer["dateTime"] = veml6070Data.measurementTime;
    jsonBuffer["UV"] = veml6070Data.UV;
    jsonBuffer["UVRisk"] = veml6070Data.uvRisk;
    jsonBuffer["UVLevel"] = veml6070Data.uvLevel;    

    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if sensorPublish(String(VEML6070_SENSOR_ID), publishJson, false)
    {
      veml6070Data.published = 1;
      return true;
    }

    return false;

  #endif

  return true;
}

//***************************************************************
// Sensor VEML6075 publish data to broker
//***************************************************************
boolean veml6075PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_VEML6075 > 0)
    veml6075Data.measurementTimestamp = getTimestamp();
    veml6075Data.UV = uv6075.readUVI();
    veml6075Data.UVA = uv6075.readUVA();
    veml6075Data.UVB = uv6075.readUVB();
    veml6075Data.received = 1;

    // Create JsonBuffer
        // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer; 

    // Add sensor datas
    jsonBuffer["dateTime"] = veml6075Data.measurementTimestamp;
    jsonBuffer["UVA"] = veml6075Data.UVA;
    jsonBuffer["UVB"] = veml6075Data.UVB;
    jsonBuffer["UV"] = veml6075Data.UV;
    
    char publishJson[256];
    serializeJson(jsonBuffer, publishJson);

    if(sensorPublish(String(VEML6075_SENSOR_ID), publishJson, false))
    {
      veml6075Data.published = 1;
      return true;
    }

    return false;
  #endif

  return true;
}


//***************************************************************
// DS18B20 Sensor setup
//***************************************************************
void ds18b20Setup()
{
  #if (WITH_DS18B20 > 0)
    debugln("Initialize DS18B20.");
    // Start up the library
    sensorDS18B20.begin();

    ds18b20Data.received = 0;
    ds18b20Data.published = 0;
  #endif
}

//***************************************************************
// Sensor DS18B20 print data to serial
//***************************************************************
void ds18b20PrintResults()
{
  #if (WITH_DS18B20 > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln("****************** DS18B20 ********************");

    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    debugln("Requesting temperatures...");
    sensorDS18B20.requestTemperatures(); // Send the command to get temperatures
    debugln("DONE");

    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    float temperature = sensorDS18B20.getTempCByIndex(0);
    
    debug("Temperature: ");
    debugln(round(temperature * 100) / 100);
    debugln("***********************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor DS18B20 publish data to broker
//***************************************************************
boolean ds18b20PublishResults()
{
  #if (WITH_BROKER > 0 && WITH_DS18B20 > 0)
    sensorDS18B20.requestTemperatures(); // Send the command to get temperatures

    ds18b20Data.measurementTimestamp = getTimestamp();
    ds18b20Data.temperature = sensorDS18B20.getTempCByIndex(0);
    ds18b20Data.received = 1;

    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;

    // Add sensor datas
    jsonBuffer["dateTime"] = ds18b20Data.measurementTimestamp;
    jsonBuffer["outTemp"] = round(ds18b20Data.temperature * 100) / 100;

    char publishJson[512];
    serializeJson(jsonBuffer, publishJson);
    
    if (sensorPublish(String(DS18B20_SENSOR_ID), publishJson, false))
    {
      ds18b20Data.published = 1;
      return true;
    }

    return false;
  #endif

  return true;
}

//***************************************************************
// Sensor VOLTAGE init
//***************************************************************
void voltageSetup()
{
  voltageData.received = 0;
  voltageData.published = 0;
}

//***************************************************************
// Sensor VOLTAGE print data to serial
//***************************************************************
void voltagePrintResults()
{
  #if (WITH_VOLTAGE > 0 && WITH_DEBUG_TO_SERIAL > 0 && WITH_DEBUG_SENSORS > 0)
    debugln();
    debugln("******************** VOLTAGE ************************");
    debug("Voltage: ");
    debug(stationVoltage, 2);
    debugln(" V");
    debugln("*****************************************************");
    debugln();
  #endif
}

//***************************************************************
// Sensor VOLTAGE publish data to broker
//***************************************************************
boolean voltagePublishResults()
{
  #if (WITH_BROKER > 0 && WITH_VOLTAGE > 0)
    voltageData.stationVoltage = stationVoltage;
    voltageData.received = 1;
    voltageData.measurementTimestamp = getTimestamp();

    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    
    jsonBuffer["dateTime"] = voltageData.measurementTimestamp;
    jsonBuffer["stationVoltage"] = round(voltageData.stationVoltage * 100) / 100;
    
    char publishJson[512];
    serializeJson(jsonBuffer, publishJson);
    
    if(sensorPublish(String(VOLTAGE_SENSOR_ID), publishJson, false))
    {
      voltageData.published = 1;
      return true;
    }

    return false;
  #endif

  return true;
}

//***************************************************************
// Mount Flash or SD
//***************************************************************
boolean mountFileSystem()
{
  #if (WITH_FLASH > 0)
    debugln("Mount configured SPIFFS (Flash) file system...");

    if (!SPIFFS.begin())
    {
      debugln("Failed to mount SPIFFS (Flash) file system!");
      errorPublish(String(SYSTEM_SENSOR_ID), "Failed to mount SPIFFS (Flash) file system, formating...", false);
      SPIFFS.format();
      debugln("SPIFFS formatted...");
      if (!SPIFFS.begin())
      {
        debugln("Failed to mount SPIFFS (Flash) file system!");
        errorPublish(String(SYSTEM_SENSOR_ID), "Failed to mount SPIFFS (Flash) file system!", false);
        return false;
      }
    }
  #elif (WITH_SD > 0)
    debugln("Mount configured SD Card file system...");
    const int chipSelect = SS;
    if (!sd.begin(chipSelect))
    {
      debugln("Failed to mount SD Card file system!");
      errorPublish(String(SYSTEM_SENSOR_ID), "Failed to mount SD Card file system!", false);
      return false;
    }
  #else
    return false;
  #endif
  
  debugln("Configured file system mounted.");
  debugln();

  return true;
}

//***************************************************************
// AS3935 Timer Callback Rountine
//***************************************************************
#if (WITH_TIMER > 0)
void timerInterruptHandler(void *pArg)
{
  timerActive = true;
  *((int *) pArg) += 1;
}
#endif

//***************************************************************
// Initialize Timer
//***************************************************************
void timerSetup()
{
  #if (WITH_TIMER > 0)
    timerActive = false;
    os_timer_setfn(&blitzTimer, timerInterruptHandler, &Counter);
    debugln("Timer initialized.");
  #endif
}

//***************************************************************
// Start Timer
//***************************************************************
void timerStart()
{
  #if (WITH_TIMER > 0)
    os_timer_arm(&blitzTimer, (getSleepSeconds() * 1000), true);
    debugln("Timer started.");
  #endif  
}

//***************************************************************
// Stop Timer
//***************************************************************
void timerStop()
{
  #if (WITH_TIMER > 0)
    os_timer_disarm(&blitzTimer);
    debugln("Timer stopped!");
  #endif  
}

//***************************************************************
// Initialize AS3935MI
//***************************************************************
void as3935miSetup()
{
  #if (WITH_AS3935MI > 0)
    pinMode(AS3935MI_PIN_IRQ, INPUT);
    SPI.begin();

    //begin() checks the Interface passed to the constructor and resets the AS3935 to 
    //default values.
    if (!as3935mi.begin())
    {
      debugln("AS3935MI begin() failed. check your AS3935MI Interface setting.");
      errorPublish(String(AS3935MI_SENSOR_ID), "AS3935MI begin() failed. check your AS3935MI Interface setting.", true);
      while (1);
    }

    //check SPI connection.
    if (!as3935mi.checkConnection())
    {
      debugln("AS3935MI checkConnection() failed. check your SPI connection and SPI chip select pin. ");
      errorPublish(String(AS3935MI_SENSOR_ID), "AS3935MI checkConnection() failed. check your SPI connection and SPI chip select pin. ", true);
      while (1);
    }
    else
    {
      debugln("AS3935MI SPI connection check passed. ");
      debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI SPI connection check passed. ", false);
    }

    //check the IRQ pin connection.
    if (!as3935mi.checkIRQ())
    {
      debugln("AS3935MI checkIRQ() failed. check if the correct IRQ pin was passed to the AS3935SPI constructor. ");
      errorPublish(String(AS3935MI_SENSOR_ID), "AS3935MI checkIRQ() failed. check if the correct IRQ pin was passed to the AS3935SPI constructor. ", true);
      while (1);
    }
    else
    {
      debugln("AS3935MI IRQ pin connection check passed. ");
      debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI IRQ pin connection check passed. ", false);
    }

    //calibrate the resonance frequency. failing the resonance frequency could indicate an issue 
    //of the sensor. resonance frequency calibration will take about 1.7 seconds to complete.
    int32_t frequency = 0;
    if (!as3935mi.calibrateResonanceFrequency(frequency))
    {
      debug("AS3935MI Resonance Frequency Calibration failed: is ");
      debug(frequency);
      debugln(" Hz, should be 482500 Hz - 517500 Hz");
      errorPublish(String(AS3935MI_SENSOR_ID), "AS3935MI Resonance Frequency Calibration failed: is " + String(frequency) + " Hz, should be 482500 Hz - 517500 Hz", true);
      //while (1);
    }
    else
    {
      debugln("AS3935MI Resonance Frequency Calibration passed. ");
      debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI Resonance Frequency Calibration passed. ", false);
    }

    debug("AS3935MI Resonance Frequency is ");
    debug(frequency);
    debugln("Hz");

    int diff = abs(500000 - frequency);
    float proz = ((diff * 100) / 500000);
    debug("Difference to optimal value (500000 Hz) is ");
    debug(diff);
    debug("Hz (");
    debug(proz);
    debugln("%)");

    debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI Resonance Frequency is " + String(frequency) + "Hz (Diff: " + String(diff) + "Hz, " + String(proz) + "%)", false);

    //calibrate the RCO.
    if (!as3935mi.calibrateRCO())
    {
      debugln("AS3935MI RCP Calibration failed. ");
      errorPublish(String(AS3935MI_SENSOR_ID), "AS3935MI RCP Calibration failed.", true);
      while (1);
    }
    else
    {
      debugln("AS3935MI RCO Calibration passed. ");
      debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI RCO Calibration passed.", false);
    }

    //set the analog front end to outdoors/indors
    //as3935mi.writeAFE(AS3935MI::AS3935_INDOORS);
    as3935mi.writeAFE(AS3935MI::AS3935_OUTDOORS);
  
    //set default value for noise floor threshold
    as3935mi.writeNoiseFloorThreshold(AS3935MI::AS3935_NFL_2);
  
    //set the default Watchdog Threshold
    as3935mi.writeWatchdogThreshold(AS3935MI::AS3935_WDTH_2);
  
    //set the default Spike Rejection 
    as3935mi.writeSpikeRejection(AS3935MI::AS3935_SREJ_2);
  
    //write default value for minimum lightnings (1)
    as3935mi.writeMinLightnings(AS3935MI::AS3935_MNL_1);
  
    //do not mask disturbers
    as3935mi.writeMaskDisturbers(false);
  
    debugln("AS3935MI Initialization complete.");
    debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI Initialization complete.", false);
  #endif
}

//***************************************************************
// Start AS3935MI Interrupt Service 
//***************************************************************
void as3935miStart()
{
  #if (WITH_AS3935MI > 0)
    //the AS3935MI will pull the interrupt pin HIGH when an event is registered and will keep it 
    //pulled high until the event register is read.
    attachInterrupt(digitalPinToInterrupt(AS3935MI_PIN_IRQ), as3935miInterruptHandler, RISING);
    debugln("AS3935MI Interrupt detection started, waiting for events...");
    debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI Interrupt detection started, waiting for events...", false);
  #endif
}

//***************************************************************
// Stop AS3935MI Interrupt Service 
//***************************************************************
void as3935miStop()
{
  #if (WITH_AS3935MI > 0)
    detachInterrupt(digitalPinToInterrupt(AS3935MI_PIN_IRQ)); // Interrupts off
    debugln("AS3935MI Interrupt detection stopped.");
    debugPublish(String(AS3935MI_SENSOR_ID), "AS3935MI Interrupt detection stopped.", false);
  #endif
}

//***************************************************************
// AS3935MI Interrupt Service Routine. 
// This function is called each time the AS3935MI reports 
// an event by pulling the IRQ pin high.
//***************************************************************
#if (WITH_AS3935MI > 0)
ICACHE_RAM_ATTR void as3935miInterruptHandler()
{
  as3935miInterrupt = true;
}
#endif

//***************************************************************
// AS3935MI Decode Results
//***************************************************************
void as3935miDecodeResults(String &debugMessage)
{
  #if (WITH_AS3935MI > 0)
    //query the interrupt source from the AS3935MI
    as3935miData.event = as3935mi.readInterruptSource();
    
    //send a report if the noise floor is too high. 
    if (as3935miData.event == AS3935MI::AS3935_INT_NH)
    {
      debugMessage = "Noise floor too high. attempting to increase noise floor threshold. ";

      //if the noise floor threshold setting is not yet maxed out, increase the setting.
      //note that noise floor threshold events can also be triggered by an incorrect
      //analog front end setting.
      if (as3935mi.increaseNoiseFloorThreshold())
      {
        debugMessage += "Increased noise floor threshold."; 
      }
      else
      {
        debugMessage += "Noise floor threshold already at maximum.";
      }

      as3935miData.noise = 1;
      as3935miData.noise_sum += 1;
      as3935miData.disturber = 0;
      as3935miData.strike = 0;
      as3935miData.distance = 0;
      as3935miData.energy = 0;
      as3935miData.unknown = 0;
    }

    //send a report if a disturber was detected. if disturbers are masked with as3935mi.writeMaskDisturbers(true);
    //this event will never be reported.
    else if (as3935miData.event == AS3935MI::AS3935_INT_D)
    {
      debugMessage = "Disturber detected. attempting to increase noise floor threshold. ";

      //increasing the Watchdog Threshold and / or Spike Rejection setting improves the AS3935MIs resistance 
      //against disturbers but also decrease the lightning detection efficiency (see AS3935 datasheet)
      uint8_t wdth = as3935mi.readWatchdogThreshold();
      uint8_t srej = as3935mi.readSpikeRejection();

      if ((wdth < AS3935MI::AS3935_WDTH_10) || (srej < AS3935MI::AS3935_SREJ_10))
      {
        //alternatively increase spike rejection and watchdog threshold 
        if (srej < wdth)
        {
          if (as3935mi.increaseSpikeRejection())
          {
            debugMessage += "Increased spike rejection ratio.";
          }
          else
          {
            debugMessage += "Spike rejection ratio already at maximum.";
          }
        }
        else
        {
          if (as3935mi.increaseWatchdogThreshold())
          {
            debugMessage += "Increased watchdog threshold.";
          }
          else
          {
            debugMessage += "Watchdog threshold already at maximum.";
          }
        }

      }
      else
      {
        debugMessage = "ERROR: Watchdog Threshold and Spike Rejection settings are already maxed out.";
      }
      
      as3935miData.noise = 0;
      as3935miData.disturber = 1;
      as3935miData.disturber_sum += 1;
      as3935miData.strike = 0;
      as3935miData.distance = 0;
      as3935miData.energy = 0;
      as3935miData.unknown = 0;
    }

    else if (as3935miData.event == AS3935MI::AS3935_INT_L)
    {
      float dist = as3935mi.readStormDistance();
      float en = as3935mi.readEnergy();

      debugMessage = "Lightning detected! Storm Front is ";
      debugMessage += String(dist);
      debugMessage += "km away. Energy is ";
      debugMessage += String(en);
      
      as3935miData.noise = 0;
      as3935miData.disturber = 0;
      as3935miData.strike = 1;
      as3935miData.strike_sum += 1;
      as3935miData.distance = dist;
      as3935miData.distance_sum += dist;
      as3935miData.energy = en;
      as3935miData.energy_sum += en;
      as3935miData.unknown = 0;
    }

    else
    {
      debugMessage = "AS3935MI unknown Interrupt source!";

      as3935miData.noise = 0;
      as3935miData.disturber = 0;
      as3935miData.strike = 0;
      as3935miData.distance = 0;
      as3935miData.energy = 0;
      as3935miData.unknown = 1;
      as3935miData.unknown_sum += 1;
    }
  #endif
}
//***************************************************************
// AS3935MI print Results to Serial
//***************************************************************
void as3935miPrintResults(String debugMessage)
{
  #if (WITH_AS3935MI > 0)

    debugln("**************************** AS3935MI ***********************************");
    debugln(debugMessage);
    debugln("***********************************************************************");
    debugln();
    
  #endif
}

//***************************************************************
// AS3935MI publish Results to Broker
//***************************************************************
boolean as3935miPublishResults(String debugMessage)
{
  #if (WITH_BROKER > 0 && WITH_AS3935MI > 0)
    // Create JsonBuffer
    StaticJsonDocument<256> jsonBuffer;
    char publishJson[256];

    measurementTimestamp = getTimestamp();
    jsonBuffer["dateTime"] = measurementTimestamp;

    if (as3935miData.event == AS3935MI::AS3935_INT_NH)
    {
      jsonBuffer["lightning_noise_count"] = 1;
    }

    else if (as3935miData.event == AS3935MI::AS3935_INT_D)
    {
      jsonBuffer["lightning_disturber_count"] = 1;
    }

    else if (as3935miData.event == AS3935MI::AS3935_INT_L)
    {
      jsonBuffer["lightning_strike_count"] = 1;
      jsonBuffer["lightning_distance"] = as3935miData.distance;
      jsonBuffer["lightning_energy"] = as3935miData.energy;
    }

    else
    {
      jsonBuffer["lightning_unknown_count"] = 1;
    }

    serializeJson(jsonBuffer, publishJson);
    sensorPublish(String(AS3935MI_SENSOR_ID), publishJson, false);
    debugPublish(String(AS3935MI_SENSOR_ID), debugMessage, false);

  #endif
}

//***************************************************************
// Following function sends AS3935MI sensor data to WeeWX
//***************************************************************
boolean as3935miWeewxUploadResults()
{
  #if (WITH_AS3935MI > 0 && WITH_WEEWX_UPLOAD > 0)
    if (bitRead(stationActions, BIT_STATUS_STOPPED_SEND_WEEWX) == 1)
    {
      debugln("Transfer to WeeWX is stopped!");
      return true;
    }
    else
    {
      debugln("Transfer to WeeWX is enabled.");
    }

    String WeeWX_URL = String(WEEWX_SERVER_PATH);
    WeeWX_URL += "?ID=";
    WeeWX_URL += WEEWX_ID;
    WeeWX_URL += "&PASSWORD=";
    WeeWX_URL += WEEWX_PASSWORD;
    WeeWX_URL += "&dateutc=now";

    if (as3935miData.event == AS3935MI::AS3935_INT_NH)
    {
      WeeWX_URL += "&lightning_noise_count=1";
    }

    else if (as3935miData.event == AS3935MI::AS3935_INT_D)
    {
      WeeWX_URL += "&lightning_disturber_count=1";
    }

    else if (as3935miData.event == AS3935MI::AS3935_INT_L)
    {
      WeeWX_URL += "&lightning_strike_count=1";
      WeeWX_URL += "&lightning_distance=" + String(as3935miData.distance);
      WeeWX_URL += "&lightning_energy=" + String(as3935miData.energy);
    }

    else
    {
      WeeWX_URL += "&lightning_unknown_count=1";
    }

    WeeWX_URL += "&softwaretype=BlitzWeatherStation&action=updateraw&realtime=1&rtfreq=";
    WeeWX_URL += String(actSleepSeconds);
  
    debugln("Connecting to WeeWX Server...");
    if (!weewxClient.connect(WEEWX_SERVER, WEEWX_SERVER_PORT))
    {
      debugln("Connection failed to WeeWX Server!");
      errorPublish(String(SYSTEM_SENSOR_ID), "Connection failed to WeeWX Server!", false);
      return false;
    }
    debugln("Connected.");

    debugln("Requesting: " + WeeWX_URL);
    weewxClient.print(String("GET ") + String(WeeWX_URL) + " HTTP/1.1\r\n" +
               "Host: " + String(WEEWX_SERVER) + "\r\n" +
               "User-Agent: BlitzWeatherStation\r\n" +
               "Connection: close\r\n\r\n");
               
    WeeWX_URL = "";
    debug("Request sent: ");
    while (weewxClient.connected())
    {
      String line = weewxClient.readStringUntil('\n');
      if (line == "\r") 
      {
        debugln("Headers received");
        break;
      }
    }
    
    String line = weewxClient.readStringUntil('\n');
    weewxClient.stop();
    
    boolean Status = true;
    if (line == "success") line = "Server confirmed all data received";
    if (line == "INVALIDPASSWORDID|Password or key and/or id are incorrect")
    {
      line = "Invalid PWS/User data entered in the ID and PASSWORD or GET parameters";
      Status = false;
    }
    if (line == "RapidFire Server")
    {
      line = "The minimum GET parameters of ID, PASSWORD, action and dateutc were not set correctly";
      Status = false;
    }
    debugln("Server Response: "+line);
    debugln("Status: Closing connection");

    return Status;
  #endif

  return false;
}

//***************************************************************
// Returns sleep seconds for deepsleep depends on actual hour
//***************************************************************
unsigned int getSleepSeconds()
{
  unsigned int actHour = timeClient.getHours();

  debug("actual Hour: ");
  debugln(actHour);

  if (actHour >= SLEEP_NIGHT_HOUR || actHour < SLEEP_DAY_HOUR)
  {
    debug("actual sleep/timer seconds (night): ");
    debugln(SLEEP_NIGHT_SECONDS);
    return SLEEP_NIGHT_SECONDS;
  }
  else
  {
    debug("actual sleep/timer seconds (day): ");
    debugln(SLEEP_DAY_SECONDS);
    return SLEEP_DAY_SECONDS;
  }
}

//***************************************************************
// Following function sends sensor data to WeeWX
//***************************************************************
boolean loopWeewxUploadResults()
{
  #if (WITH_WEEWX_UPLOAD > 0)
    if (bitRead(stationActions, BIT_STATUS_STOPPED_SEND_WEEWX) == 1)
    {
      debugln("Transfer to WeeWX is stopped!");
      return true;
    }
    else
    {
      debugln("Transfer to WeeWX is enabled.");
    }

    // TODO: Buffer to file if not connected
  
    debugln("Connecting to WeeWX Server...");
    if (!weewxClient.connect(WEEWX_SERVER, WEEWX_SERVER_PORT))
    {
      debugln("Connection failed");
      return false;
    }
    debugln("Connected.");

    String WeeWX_URL = String(WEEWX_SERVER_PATH);
    WeeWX_URL += "?ID=";
    WeeWX_URL += WEEWX_ID;
    WeeWX_URL += "&PASSWORD=";
    WeeWX_URL += WEEWX_PASSWORD;
    WeeWX_URL += "&dateutc=now";

    #if (STATION_NUM_ID == 1) //Ventus
      #if (WITH_DS18B20 > 0)
        WeeWX_URL += "&ventus_temperature=";
        WeeWX_URL += round(ds18b20Data.temperature * 100) / 100;
      #endif
      #if (WITH_BME280 > 0)
        WeeWX_URL += "&ventus_temperature1=";
        WeeWX_URL += round(bme280Data.temperature * 100) / 100;
        WeeWX_URL += "&ventus_humidity=";
        WeeWX_URL += bme280Data.humidity;
        WeeWX_URL += "&ventus_pressure=";
        WeeWX_URL += round(bme280Data.pressure * 100) /100;
        WeeWX_URL += "&ventus_dewpoint=";
        WeeWX_URL += round(bme280Data.dewpoint * 100) / 100;
        WeeWX_URL += "&ventus_heatindex=";
        WeeWX_URL += round(bme280Data.heatindex * 100) / 100;
        WeeWX_URL += "&ventus_barometer=";
        WeeWX_URL += round(bme280Data.barometer * 100) / 100;
      #endif
      #if (WITH_BME680 > 0)
        WeeWX_URL += "&ventus_temperature1=";
        WeeWX_URL += round(bme680Data.temperature * 100) / 100;
        WeeWX_URL += "&ventus_humidity=";
        WeeWX_URL += bme680Data.humidity;
        WeeWX_URL += "&ventus_pressure=";
        WeeWX_URL += round(bme680Data.pressure * 100) /100;
        WeeWX_URL += "&ventus_dewpoint=";
        WeeWX_URL += round(bme680Data.dewpoint * 100) / 100;
        WeeWX_URL += "&ventus_heatindex=";
        WeeWX_URL += round(bme680Data.heatindex * 100) / 100;
        WeeWX_URL += "&ventus_barometer=";
        WeeWX_URL += round(bme680Data.barometer * 100) / 100;
      #endif
      #if (WITH_W132 > 0)
        if (w132TempHumidityData.published > 0)
        {
          WeeWX_URL += "&ventus_temperature2=";
          WeeWX_URL += round(w132Data.temperature * 100) / 100;
          
        }
      #endif
      
      WeeWX_URL += "&ventus_stationVoltage=";
      WeeWX_URL += round(voltageData.stationVoltage * 100) / 100;
      WeeWX_URL += "&softwaretype=VentusWeatherStation";
    #endif
    
    #if (STATION_NUM_ID == 2) //Solar
      WeeWX_URL += "&solar_temperature=";
      WeeWX_URL += round(bme280Data.temperature * 100) / 100;
      WeeWX_URL += "&solar_dewpoint=";
      WeeWX_URL += round(bme280Data.dewpoint * 100) / 100;
      WeeWX_URL += "&solar_humidity=";
      WeeWX_URL += bme280Data.humidity;
      WeeWX_URL += "&solar_barometer=";
      WeeWX_URL += round(bme280Data.barometer * 100) / 100;
      WeeWX_URL += "&solar_pressure=";
      WeeWX_URL += round(bme280Data.pressure * 100) /100;
      WeeWX_URL += "&solar_heatindex=";
      WeeWX_URL += round(bme280Data.heatindex * 100) / 100;
      WeeWX_URL += "&solar_stationVoltage=";
      WeeWX_URL += round(voltageData.stationVoltage * 100) / 100;
      WeeWX_URL += "&softwaretype=SolarWeatherStation";
    #endif

    #if (STATION_NUM_ID == 3) //Blitz
      #if (WITH_BME280 > 0)
        WeeWX_URL += "&blitz_temperature=";
        WeeWX_URL += round(bme280Data.temperature * 100) / 100;
        WeeWX_URL += "&blitz_dewpoint=";
        WeeWX_URL += round(bme280Data.dewpoint * 100) / 100;
        WeeWX_URL += "&blitz_humidity=";
        WeeWX_URL += bme280Data.humidity;
        WeeWX_URL += "&blitz_barometer=";
        WeeWX_URL += round(bme280Data.barometer * 100) / 100;
        WeeWX_URL += "&blitz_pressure=";
        WeeWX_URL += round(bme280Data.pressure * 100) /100;
        WeeWX_URL += "&blitz_heatindex=";
        WeeWX_URL += round(bme280Data.heatindex * 100) / 100;
      #endif
      WeeWX_URL += "&blitz_stationVoltage=";
      WeeWX_URL += round(voltageData.stationVoltage * 100) / 100;
      WeeWX_URL += "&softwaretype=BlitzWeatherStation";
    #endif

    WeeWX_URL += "&action=updateraw&realtime=1&rtfreq=";
    WeeWX_URL += String(actSleepSeconds);

    debugln("Requesting: " + WeeWX_URL);
    weewxClient.print(String("GET ") + WeeWX_URL + " HTTP/1.1\r\n" +
               "Host: " + WEEWX_SERVER + "\r\n" +
               "User-Agent: WeatherStation\r\n" +
               "Connection: close\r\n\r\n");
    WeeWX_URL = ""
    debug("Request sent: ");
    while (weewxClient.connected())
    {
      String line = weewxClient.readStringUntil('\n');
      if (line == "\r") 
      {
        debugln("Headers received");
        break;
      }
    }
    
    String line = weewxClient.readStringUntil('\n');
    weewxClient.stop();
    
    boolean Status = true;
    if (line == "success") line = "Server confirmed all data received";
    if (line == "INVALIDPASSWORDID|Password or key and/or id are incorrect")
    {
      line = "Invalid PWS/User data entered in the ID and PASSWORD or GET parameters";
      Status = false;
    }
    if (line == "RapidFire Server")
    {
      line = "The minimum GET parameters of ID, PASSWORD, action and dateutc were not set correctly";
      Status = false;
    }
    debugln("Server Response: "+line);
    debugln("Status: Closing connection");

    return Status;
  #endif

  return true;
}

//***************************************************************
// Publish loop results to broker
//***************************************************************
boolean loopPublishResults()
{ 
  #if (WITH_BROKER > 0)
    char publishJson[1024];
      
    // Create JsonBuffer
    StaticJsonDocument<1024> jsonBuffer;

    #if (STATION_NUM_ID == 1) //Ventus
      jsonBuffer["dateTime"] = measurementTimestamp;
      jsonBuffer["usUnits"] = 16; //WeeWX metric
      jsonBuffer["stationId"] = STATION_NUM_ID; //Ventus
  
      #if (WITH_W132 > 0)
        if (w132TempHumidityData.published > 0)
        {
          jsonBuffer["w132-outTemp"] = round(w132TempHumidityData.temperature * 100) / 100;
          jsonBuffer["w132-tempf"] = round(w132TempHumidityData.tempf * 100) / 100;
          jsonBuffer["w132-outHumidity"] = round(w132TempHumidityData.humidity * 100) /100;
    
          if (w132TempHumidityData.dewpoint != INVALID_FLOAT_VALUE)
          {
            jsonBuffer["w132-dewpoint"] = round(w132TempHumidityData.dewpoint * 100) /100;
            jsonBuffer["w132-dewptf"] = round(w132TempHumidityData.dewptf * 100) /100;
            jsonBuffer["w132-spreadDewpint"] = round(w132TempHumidityData.spreadDewpoint * 100) /100;
            jsonBuffer["w132-spreadDewptf"] = round(w132TempHumidityData.spreadDewptf * 100) /100;
          }
    
          if (w132TempHumidityData.heatindex != INVALID_FLOAT_VALUE)
          {
            jsonBuffer["w132-heatindex"] = round(w132TempHumidityData.heatindex * 100) /100;
            jsonBuffer["w132-heatindexf"] = round(w132TempHumidityData.heatindexf * 100) /100;
          }
          
          if (w132TempHumidityData.windchill != INVALID_FLOAT_VALUE)
          {
            jsonBuffer["w132-windchill"] = round(w132TempHumidityData.windchill * 100) /100;
            jsonBuffer["w132-windchillf"] = round(w132TempHumidityData.windchillf * 100) /100;
          }
          
          jsonBuffer["w132-tempTrend"] = w132TempHumidityData.trend;
          jsonBuffer["w132-windBatteryStatus"] = w132TempHumidityData.windBatteryStatus;
        }
        else if (w132WindSpeedData.published > 0)
        {
          jsonBuffer["w132-windSpeed"] = w132WindSpeedData.windSpeed;
          jsonBuffer["w132-windSpeedKmh"] = w132WindSpeedData.windSpeedKmh;
    
          if (w132WindSpeedData.dewpoint != INVALID_FLOAT_VALUE)
          {
            jsonBuffer["w132-dewpoint"] = round(w132WindSpeedData.dewpoint * 100) /100;
            jsonBuffer["w132-dewptf"] = round(w132WindSpeedData.dewpoint * 100) /100;
            jsonBuffer["w132-spreadDewpoint"] = round(w132WindSpeedData.spreadDewpoint * 100) /100;
            jsonBuffer["w132-spreadDewptf"] = round(w132WindSpeedData.spreadDewptf * 100) /100;
          }
          
          if (w132WindSpeedData.heatindex != INVALID_FLOAT_VALUE)
          {
            jsonBuffer["w132-heatindex"] = round(w132WindSpeedData.heatindex * 100) /100;
            jsonBuffer["w132-heatindexf"] = round(w132WindSpeedData.heatindexf * 100) /100;
          }
          
          if (w132WindSpeedData.windchill != INVALID_FLOAT_VALUE)
          {
            jsonBuffer["w132-windchill"] = round(w132WindSpeedData.windchill * 100) /100;
            jsonBuffer["w132-windchillf"] = round(w132WindSpeedData.windchillf * 100) /100;
          }
          
          jsonBuffer["w132-windBatteryStatus"] = w132WindSpeedData.windBatteryStatus;
        }
      #endif
  
      #if (WITH_W174 > 0)
        if (w174Data.published > 0)
        {
          jsonBuffer["w174-rain"] = w174Data.rain;
        }
        else if (w174Data.valid > 0)
        {
          jsonBuffer["w174-rain"] = 0;
        }
        jsonBuffer["w174-rainCounter"] = w174Data.rainCounter;
        jsonBuffer["w174-rainTotal"] = w174Data.rainTotal;
        jsonBuffer["w174-rainBatteryStatus"] = w174Data.rainBatteryStatus;
      #endif
  
      #if (WITH_BME280 > 0)
        jsonBuffer["bme280-outTemp"] = (round(bme280Data.temperature * 100) / 100);
        jsonBuffer["bme280-outHumidity"] = round(bme280Data.humidity * 100) / 100;
        jsonBuffer["bme280-pressure"] = round(bme280Data.pressure * 100) / 100;
        jsonBuffer["bme280-barometer"] = round(bme280Data.barometer * 100) / 100;
        jsonBuffer["bme280-dewpoint"] = round(bme280Data.dewpoint * 100) / 100;
        jsonBuffer["bme280-spreadDewpoint"] = round(bme280Data.spreadDewpoint * 100) / 100;
        jsonBuffer["bme280-heatindex"] = round(bme280Data.heatindex * 100) / 100;
        jsonBuffer["bme280-approxAltitude"] = round(bme280Data.approxAltitude * 100) / 100;
      #endif
  
      #if (WITH_BME680 > 0)
        jsonBuffer["bme680-outTemp"] = (round(bme680Data.temperature * 100) / 100);
        jsonBuffer["bme680-outHumidity"] = round(bme680Data.humidity * 100) / 100;
        jsonBuffer["bme680-pressure"] = round(bme680Data.pressure * 100) / 100;
        jsonBuffer["bme680-barometer"] = round(bme680Data.barometer * 100) / 100;
        jsonBuffer["bme680-dewpoint"] = round(bme680Data.dewpoint * 100) / 100;
        jsonBuffer["bme680-spreadDewpoint"] = round(bme680Data.spreadDewpoint * 100) / 100;
        jsonBuffer["bme680-heatindex"] = round(bme680Data.heatindex * 100) / 100;
        jsonBuffer["bme680-gas"] = round(bme680Data.gas * 100) / 100;
        jsonBuffer["bme680-approxAltitude"] = round(bme680Data.approxAltitude * 100) / 100;
      #endif
  
      #if (WITH_DS18B20 > 0)
        jsonBuffer["ds18b20-outTemp"] = round(ds18b20Data.temperature * 100) / 100;
      #endif
  
      #if (WITH_BH1750 > 0)
        jsonBuffer["bh1750-luminosity"] = bh1750Data.luminosity;
      #endif
  
      #if (WITH_BH1750B > 0)
        jsonBuffer["bh1750b-luminosity"] = bh1750Data.luminosity;
        jsonBuffer["bh1750b-fc"] = bh1750Data.fc;
        jsonBuffer["bh1750b-radiation"] = bh1750Data.radiation;
      #endif
  
      #if (WITH_BH1750AS > 0)
        jsonBuffer["bh1750as-luminosity"] = bh1750Data.luminosity;
      #endif
  
      #if (WITH_TSL2591 > 0)
          jsonBuffer["tsl2591-luminosity"] = tsl2591Data.luminosity;
      #endif
  
      #if (WITH_VEML6070 > 0)
        jsonBuffer["veml6070-UV"] = veml6070Data.UV;
        jsonBuffer["veml6070-UVRisk"] = veml6070Data.uvRisk;
        jsonBuffer["veml6070-UVLevel"] = veml6070Data.uvLevel;
      #endif
  
      #if (WITH_VEML6075 > 0)
        jsonBuffer["veml6075-UV"] = veml6075Data.UV;
        jsonBuffer["veml6075-UVA"] = veml6075Data.UVA;
        jsonBuffer["veml6075-UVB"] = veml6075Data.UVB;
      #endif
  
      #if (WITH_VOLTAGE > 0)
        jsonBuffer["stationVoltage"] = round(voltageData.stationVoltage * 100) / 100;
      #endif
    #endif //Ventus
  
    #if (STATION_NUM_ID == 2) //Solar
      jsonBuffer["dateTime"] = measurementTimestamp;
      jsonBuffer["usUnits"] = 16; //WeeWX metric
      jsonBuffer["stationId"] = STATION_NUM_ID; //Solar
      
      #if (WITH_BME280 > 0)
        jsonBuffer["outTemp"] = (round(bme280Data.temperature * 100) / 100);
        jsonBuffer["outHumidity"] = round(bme280Data.humidity * 100) / 100;
        jsonBuffer["pressure"] = round(bme280Data.pressure * 100) / 100;
        jsonBuffer["barometer"] = round(bme280Data.barometer * 100) / 100;
        jsonBuffer["dewpoint"] = round(bme280Data.dewpoint * 100) / 100;
        jsonBuffer["spreadDewpoint"] = round(bme280Data.spreadDewpoint * 100) / 100;
        jsonBuffer["heatindex"] = round(bme280Data.heatindex * 100) / 100;
        jsonBuffer["approxAltitude"] = round(bme280Data.approxAltitude * 100) / 100;
      #elif (WITH_BME680 > 0)
        jsonBuffer["outTemp"] = (round(bme680Data.temperature * 100) / 100);
        jsonBuffer["outHumidity"] = round(bme680Data.humidity * 100) / 100;
        jsonBuffer["pressure"] = round(bme680Data.pressure * 100) / 100;
        jsonBuffer["barometer"] = round(bme680Data.barometer * 100) / 100;
        jsonBuffer["dewpoint"] = round(bme680Data.dewpoint * 100) / 100;
        jsonBuffer["spreadDewpoint"] = round(bme680Data.spreadDewpoint * 100) / 100;
        jsonBuffer["heatindex"] = round(bme680Data.heatindex * 100) / 100;
        jsonBuffer["gas"] = round(bme680Data.gas * 100) / 100;
        jsonBuffer["approxAltitude"] = round(bme680Data.approxAltitude * 100) / 100;
      #endif
  
      #if (WITH_VOLTAGE > 0)
        jsonBuffer["stationVoltage"] = round(voltageData.stationVoltage * 100) / 100;
      #endif
    #endif //Solar
        
    #if (STATION_NUM_ID == 3) //Blitz
      jsonBuffer["dateTime"] = measurementTimestamp;
      jsonBuffer["usUnits"] = 16; //WeeWX metric
      jsonBuffer["stationId"] = STATION_NUM_ID; //Blitz
      
      #if (WITH_BME280 > 0)
        jsonBuffer["outTemp"] = (round(bme280Data.temperature * 100) / 100);
        jsonBuffer["outHumidity"] = round(bme280Data.humidity * 100) / 100;
        jsonBuffer["pressure"] = round(bme280Data.pressure * 100) / 100;
        jsonBuffer["barometer"] = round(bme280Data.barometer * 100) / 100;
        jsonBuffer["dewpoint"] = round(bme280Data.dewpoint * 100) / 100;
        jsonBuffer["spreadDewpoint"] = round(bme280Data.spreadDewpoint * 100) / 100;
        jsonBuffer["heatindex"] = round(bme280Data.heatindex * 100) / 100;
        jsonBuffer["approxAltitude"] = round(bme280Data.approxAltitude * 100) / 100;
      #elif (WITH_BME680 > 0)
        jsonBuffer["outTemp"] = (round(bme680Data.temperature * 100) / 100);
        jsonBuffer["outHumidity"] = round(bme680Data.humidity * 100) / 100;
        jsonBuffer["pressure"] = round(bme680Data.pressure * 100) / 100;
        jsonBuffer["barometer"] = round(bme680Data.barometer * 100) / 100;
        jsonBuffer["dewpoint"] = round(bme680Data.dewpoint * 100) / 100;
        jsonBuffer["spreadDewpoint"] = round(bme680Data.spreadDewpoint * 100) / 100;
        jsonBuffer["heatindex"] = round(bme680Data.heatindex * 100) / 100;
        jsonBuffer["gas"] = round(bme680Data.gas * 100) / 100;
        jsonBuffer["approxAltitude"] = round(bme680Data.approxAltitude * 100) / 100;
      #endif

      #if (WITH_AS3935MI > 0)
        jsonBuffer["strike_sum"] = as3935miData.strike_sum;
        jsonBuffer["noise_sum"] = as3935miData.noise_sum;
        jsonBuffer["disturber_sum"] = as3935miData.disturber_sum;
        jsonBuffer["unknown_sum"] = as3935miData.unknown_sum;

        float dist = 0.0;
        float en = 0.0;
        if (as3935miData.strike_sum > 0)
        {
          if (as3935miData.distance_sum > 0)
          {
            dist = round((as3935miData.distance_sum / as3935miData.strike_sum) * 100) / 100;
            jsonBuffer["distance_avg"] = dist;
          }
          if (as3935miData.energy > 0)
          {
            en = round((as3935miData.energy_sum / as3935miData.strike_sum) * 100) / 100;
            jsonBuffer["energy_avg"] = en;    
          }
        }
        
        as3935miData.event = 0;
        as3935miData.strike_sum = 0;
        as3935miData.distance_sum = 0;
        as3935miData.energy_sum = 0;
        as3935miData.noise_sum = 0;
        as3935miData.disturber_sum = 0;
        as3935miData.unknown_sum = 0;
      #endif

      #if (WITH_VOLTAGE > 0)
        jsonBuffer["stationVoltage"] = round(voltageData.stationVoltage * 100) / 100;
      #endif
    #endif //Blitz
  
    serializeJson(jsonBuffer, publishJson);
        
    return sensorPublish(String(BROKER_COLLECTED_VALUES), publishJson, false);
  #endif

  return true;
}

//***************************************************************
// Weather Station deepsleep
//***************************************************************
void stationDeepSleep(unsigned long startTime, boolean sensorDataAvaiable = true, boolean configAvailable = true)
{
  debugln("going to deepsleep...");
  
  // write config file
  if (configAvailable)
  {
    writeConfiguration();
  }

  if (sensorDataAvaiable)
  {
    // loop Results to Broker
    loopPublishResults();

    // loop Results to WeeWX
    loopWeewxUploadResults();

    // Zambretti Data, require pressure/barometer from BME280
    #if (WITH_BME280 > 0)
      if (bme280Data.received)
      {
        updateZambrettiData();
        calculateZambrettiData();
        printZambrettiData();
        publishZambrettiData();
      }
    #endif
  }
  
  // disconnect Broker
  brokerDisconnect("deepsleep");
      
  // disconnect WiFi
  wifiDisconnect();

  // calculate remaining sleep time
  realSleep = (realSleep - ((millis() - startTime) / 1000));
  if (realSleep <= 0 || realSleep > actSleepSeconds)
  {
    realSleep = actSleepSeconds;
  }

  #if (WITH_DEEPSLEEP > 0)
    debug("deepsleep ");
    debug(realSleep);
    debugln(" seconds...");
    ESP.deepSleep(realSleep * 1000000, WAKE_RF_DEFAULT); // xx secs.
    delay(1000);
  #else
    debug("sleep ");
    debug(realSleep);
    debugln(" seconds...");
    delay(realSleep * 1000);
    ESP.restart();
  #endif  
}

//***************************************************************
//***************************************************************
// Weather Station Setup
//***************************************************************
//***************************************************************
void setup()
{
  unsigned long startTime = millis();

  debugStart(SERIALSPEED);
  debugln();
  debug("Start Weather Station ");
  debugln(STATION_ID);

  // inits
  #if (WITH_AS3935MI > 0)
    as3935miData.strike = 0;
    as3935miData.distance = 0;
    as3935miData.energy = 0;
    as3935miData.noise = 0;
    as3935miData.disturber = 0;
    as3935miData.unknown = 0;
  #endif
  ventusStatus = 0;

  // Battery Voltage
  stationVoltage = batteryVoltage();

  // RTC
  rtcSetup();

  // ventus
  w174Stop();
  w132Stop();
  
  // WiFi 
  wifiConnect();

  // NTP 
  rtcSet();

  // Sleeps
  actSleepSeconds = getSleepSeconds();
  realSleep = actSleepSeconds;

  // Broker
  #if (WITH_BROKER > 0)
    stationActions = 0;
    brokerStatus = 0;

    if (!brokerConnect(false))
    {
      debugln("Broker not connected, loop will be not started!");
      stationDeepSleep(startTime, false, false);
      return;
    }

    // receive subscribed topics to check, if OTA is required or publishing is disabled or reboot ....
    if (!brokerReceiveAllSubscribedTopics())
    {
      debugln("Not all subcribed topics received, loop will be not started!");
      errorPublish(String(SYSTEM_SENSOR_ID), "Not all subcribed topics received, loop will be not started!", false);
      delay(100);
      stationDeepSleep(startTime, false, false);
      return;
    }

    if (bitRead(stationActions, BIT_ACTION_REQUIRED_REBOOT) == 1)
    {
      bitClear(stationActions, BIT_ACTION_REQUIRED_REBOOT);
      brokerPublish(ACTION_TOPIC_REBOOT, "0", true);
      delay(200);
      debugln("Setup: Reboot is required, rebooting now.");
      debugPublish(String(SYSTEM_SENSOR_ID), "Setup: Reboot is required, rebooting now.", false);
      delay(200);
      ESP.restart();
      return;
    }

    if (bitRead(stationActions, BIT_ACTION_REQUIRED_OTA) == 1)
    {
      debugln("Setup: OTA is required.");
      debugPublish(String(SYSTEM_SENSOR_ID), "Setup: OTA is required.", false);
      otaStart();
      return;
    }

    // publish saved Topics
    brokerPublishBuffer();
  #endif
  
  // File System
  if (!mountFileSystem())
  {
    debugln("Error mountFileSystem, loop will be not started!");
    errorPublish(String(SYSTEM_SENSOR_ID), "Error mountFileSystem, loop will be not started!", false);
    delay(100);
    stationDeepSleep(startTime, false, false);
    return;      
  }
  
  // Load ConfigData
  if (!readConfiguration())
  {
    debugln("Error readConfiguration, loop will be not started!");
    errorPublish(String(SYSTEM_SENSOR_ID), "Error readConfiguration, loop will be not started!", false);
    stationDeepSleep(startTime, false, false);
    return;
  }

  //Global Measurement Timestamp
  measurementTimestamp = getTimestamp();
  
  // Voltage
  voltageSetup();
  voltagePrintResults();
  voltagePublishResults();

  // sensor data
  veml6070Setup();
  veml6070PrintResults();
  veml6070PublishResults();

  veml6075FullSetup();
  //veml6075SimpleSetup();
  veml6075PrintResults();
  veml6075PublishResults();

  bme280Setup();
  bme280PrintResults();
  bme280PublishResults();

  bme680Setup();
  bme680PrintResults();
  bme680PublishResults();

  bh1750Setup();
  bh1750PrintResults();
  bh1750PublishResults();

  bh1750bPrintResults();
  bh1750bPublishResults();

  bh1750asPrintResults();
  bh1750asPublishResults();

  ds18b20Setup();
  ds18b20PrintResults();
  ds18b20PublishResults();

  ml8511Setup();
  ml8511PrintResults();
  ml8511PublishResults();

  tsl2591Setup();
  tsl2591PrintResults();
  tsl2591PublishResults();

  mhrdSetup();
  mhrdPrintResults();
  mhrdPublishResults();

  if (STATION_ID == "blitz")
  {
    // loop Results to Broker
    loopPublishResults();

    // loop Results to WeeWX
    loopWeewxUploadResults();
  }

  // AS3935MI Setup
  as3935miSetup();

  // Timer Setup
  timerSetup();
  
  // Ventus Setups 
  w132Setup();
  w174Setup();

  // Ventus/AS3935MI Interrups start
  w132Start();
  w174Start();
  timerStart();
  as3935miStart();

  debugln("start loop...");
  debugln();
  loopFirstStartTime = millis();

  // calculate Sleep time
  realSleep = (realSleep - ((millis() - startTime) / 1000));
}

//***************************************************************
//***************************************************************
// Weather Station main loop
//***************************************************************
//***************************************************************
void loop()
{
  unsigned long startTime = millis();

  #if (WITH_BROKER > 0)
    if (bitRead(stationActions, BIT_STATUS_ACTIVE_OTA) == 1)
    {
      otaHandler();
      return;
    }
    
    if (bitRead(stationActions, BIT_ACTION_REQUIRED_OTA) == 1)
    {
      debugln("Loop: OTA is required");
      debugPublish(String(SYSTEM_SENSOR_ID), "Loop: OTA is required.", false);
      w132Stop();
      w174Stop();
      as3935miStop();
      timerStop();
      otaStart();
      return;
    }
  
    if (bitRead(stationActions, BIT_ACTION_REQUIRED_REBOOT) == 1)
    {
      w132Stop();
      w174Stop();
      as3935miStop();
      timerStop();
      bitClear(stationActions, BIT_ACTION_REQUIRED_REBOOT);
      brokerPublish(ACTION_TOPIC_REBOOT, "0", true);
      delay(200);
      debugln("Loop: Reboot is required, rebooting now.");
      debugPublish(String(SYSTEM_SENSOR_ID), "Loop: Reboot is required, rebooting now.", false);
      delay(200);
      
      ESP.restart();
      yield();
      return;
    }
  #endif

  #if (WITH_BROKER > 0)
    if(brokerIsConnected())
    {
      mqttclient.loop();
    }
    else
    {
      w132Stop();
      w174Stop();
      as3935miStop();
      timerStop();

      if (!brokerConnect(true))
      {
        debugln("Broker can not be connected, loop will be canceled!");
        stationDeepSleep(startTime, true, true);
        return;
      }

      // receive subscribed topics to check, if OTA is required or publishing is disabled or reboot ....
      if (!brokerReceiveAllSubscribedTopics())
      {
        debugln("Not all subcribed topics received, loop will be be canceled!");
        errorPublish(String(SYSTEM_SENSOR_ID), "Not all subcribed topics received, loop will be be canceled!", false);
        delay(100);
        stationDeepSleep(startTime, false, false);
        return;
      }

      w132Start();
      w174Start();
      as3935miStart();
      timerStart();
    }
  #endif
  
  #if (WITH_W132 > 0 || WITH_W174 > 0)
    // try to get values from W132/W174, but time is money, kill W132, W174 afer 1 minute without values
    if (millis() - loopFirstStartTime > ventusMaxReceiveMilliSeconds)
    {
      debugPublish(String(SYSTEM_SENSOR_ID), "Time is over", false);
      
      #if (WITH_W132 > 0)
        w132Stop();
      #endif
      #if (WITH_W174 > 0)
        w174Stop();
      #endif
    }
  #endif
  
  #if (WITH_W132 > 0)
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED) != 1 && w132fifoAvailable())
    {
      w132DataReceived = w132fifoRead();

      #if (WITH_DEBUG_TO_BROKER > 0)
        if(w132DataReceived > 0)
        {
          /*
          String telegram = "";
          for (byte i = 0; i < 32; i++)
          {
            telegram = telegram + String(bitRead(w132DataReceived, i));
          }
          debugPublish(String(SYSTEM_SENSOR_ID), "W132 Telegram: " + telegram, false);
          */
        }
      #endif
      
      if (w132DataReceived > 0 && w132DataReceived != w132LastDataReceived)
      {
        w132LastDataReceived = w132DataReceived;

        if (w132DecodeResults(w132DataReceived))
        {
          w132PrintResults(w132DataReceived);

          if (w132PublishResults())
          {
            if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_WINDSPEED_RECEIVED) == 1 && bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_WINDDIRGUST_RECEIVED) == 1)
            {
              w132Stop();
            }
            else if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_TEMPHUMIDITY_RECEIVED) == 1)
            {
              w132Stop();
            }

            #if (WITH_W174 > 0)
              if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W174_IS_REQUIRED) != 1 && bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED) == 1)
              {
                w174Stop();
              }
            #endif
          }
        }
      }
    }
  #endif

  #if (WITH_W174 > 0)
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED) != 1 && w174fifoAvailable())
    {
      w174DataReceived = w174fifoRead();

      #if (WITH_DEBUG_TO_BROKER > 0)
        if(w174DataReceived > 0)
        {
          /*
          String telegram = "";
          for (byte i = 0; i < 32; i++)
          {
            telegram = telegram + String(bitRead(w174DataReceived, i));
          }
          debugPublish(String(SYSTEM_SENSOR_ID), "W174 Telegram: " + telegram, false);
          */
        }
      #endif
            
      if (w174DataReceived > 0 && w174DataReceived != w174LastDataReceived)
      {
        w174LastDataReceived = w174DataReceived;

        if (w174DecodeResults(w174DataReceived))
        {
          w174PrintResults(w174DataReceived);

          if (w174PublishResults())
          {
            w174Stop();  
          }
        }
      }
    }
  #endif

  #if ((WITH_W132 > 0) || (WITH_W174 > 0))
    if (bitRead(ventusStatus, BIT_VENTUS_STATUS_W132_IS_STOPPED) == 1 && bitRead(ventusStatus, BIT_VENTUS_STATUS_W174_IS_STOPPED) == 1)
    {
      // go to deepsleep
      stationDeepSleep(startTime, true, true);
    }
  #elif (WITH_AS3935MI != 1)
    // go to deepsleep
    stationDeepSleep(startTime, true, true);
  #endif

  #if (WITH_AS3935MI > 0)
    if (as3935miInterrupt)
    {
      //reset the interrupt variable
      as3935miInterrupt = false;

      debugln("AS3935MI Interrupt fired.");
      debugPublish(String(SYSTEM_SENSOR_ID), "AS3935MI Interrupt fired.", false);

      // TODO??? publish saved Topics
      // brokerPublishBuffer();

      // Stop AS3935MI Interrupt
      //as3935miStop();

      String debugMessage;
      as3935miDecodeResults(debugMessage);
      as3935miWeewxUploadResults();

      // Start AS3935MI Interrupt again
      //as3935miStart();

      as3935miPrintResults(debugMessage);
      as3935miPublishResults(debugMessage);

      debugln("Back to loop.");
    }
  #endif

  #if (WITH_TIMER > 0)
    if (timerActive)
    {
      //reset the interrupt variable
      timerActive = false;

      debugln("Timer Interrupt fired.");
      debugPublish(String(SYSTEM_SENSOR_ID), "Timer Interrupt fired.", false);

      // Battery Voltage
      stationVoltage = batteryVoltage();           
      voltagePrintResults();
      voltagePublishResults();

      bme280PrintResults();
      bme280PublishResults();

      // publish saved Topics
      brokerPublishBuffer();
    
      // loop Results to Broker
      loopPublishResults();

      // loop Results to WeeWX
      loopWeewxUploadResults();

      debugln("Back to loop.");
    }
  #endif

  // Give a time for ESP
  yield();
}
