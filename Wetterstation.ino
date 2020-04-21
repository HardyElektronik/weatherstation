/*
 * --------------------------------------------------------------------------------
 *  ESP8266 based weatherstation with 
 *    - DHT21 (AM2301) temperature, humidity sensor OR
 *    - BME280 temperature, humidity, pressure sensor
 *    
 *    - data pushed to mySQL database via PHP script
 *    - HMI based on bootstrap
 * --------------------------------------------------------------------------------   
 */


 /* 
  * --------------------------------------------------------------------------------
  *  INCLUDES
  * --------------------------------------------------------------------------------
*/
  // sensor libs
  #include "DHT.h"                // DHT21 / AM2301 sensor lib
  #include <Wire.h>               // I2C lib
  #include <Adafruit_BME280.h>    // BME280 sensor lib

  // ESP WiFi lib
  #include <ESP8266WiFi.h>    
  
  // ESP HTTP client lib
  #include <ESP8266HTTPClient.h> 

  // NTP-client lib
  #include <EasyNTPClient.h>      // NTP lib for data timestamp
  #include <WiFiUdp.h>            // UDP lib for NTP client

  // date / time functions
  #include <Time.h>              
  #include <TimeLib.h>


/* 
 * --------------------------------------------------------------------------------    
 * DEFINES
 * --------------------------------------------------------------------------------
 */
  #define DHTPIN 0                              // DHT pin 0 = D3 on ESP8266
  #define DHTTYPE DHT21                         // DHT 21 (AM2301)
  
  #define SERIAL_DEBUG 1                        // debug on serial port (can be switched on / off by TRUE / FALSE)

  #define DEBUG_LED 2                           // debug LED on ESP8266 board (blinks for fun only ;-) )

  #define SEALEVEL_PRESSURE_HPA (1013.25)       // air pressure on sea level (hPa)

  // what sensor is used? (0 = DHT / 1 = BME)
  /* 
   *  Two different stations are used. One equipped with DHT and BME sensor. One with BME sensor only
   */
  #define SENSOR_TEMPERATURE 1
  #define SENSOR_HUMIDITY    1   
  #define SENSOR_PRESSURE    1


 /* --------------------------------------------------------------------------------
  *  CONST
  * --------------------------------------------------------------------------------
  */
  // WiFi SSID / PASSWORD
  const char* c_SSID = "INSERT_YOUR_WIFI_SSID_HERE";              // SSID WLAN
  const char* c_PASSWD = "INSERT_YOUR_WIFI_KEY_HERE";   // WiFi password

  // Host running php-script and mySQL database
  const char* c_HOST = "http://192.168.178.50/index.php";  

  // Client ID - define multiple client IDs to identify data in the database - must be unique!
  const byte c_CLIENT_ID = 2;

  // Refresh cycle in seconds
  const int c_REFRESH_CYCLE = 60;  


/* -------------------------------------------------------------------------------- */
  
  // sensors
  DHT dht(DHTPIN, DHTTYPE);              
  Adafruit_BME280 bme;                   

  // WiFi-UDP for NTP connection 
  WiFiUDP Udp;                   

  // NTP-client
  EasyNTPClient ntpClient(Udp, "pool.ntp.org", 3600);

  // HTTP-client
  HTTPClient http; 

  // date / time functions
  time_t Uhrzeit;         


/* --------------------------------------------------------------------------------
 *  define global variables
 * --------------------------------------------------------------------------------
 */
  bool DHT_OK          = false;
  bool BME_OK          = false;

  bool WiFi_Connected  = false;
  
  bool Temp_C_OK       = false;
  bool Humidity_OK     = false;
  bool Pressure_OK     = false;
  bool ElevationNN_OK  = false;
  
  bool BME_Status      = false;
  bool DHT_Status      = false;
  bool BME_Ready       = false;

  int NTPtime          = 0;

  float Temp_C         = 0.0;     // temperature in °C
  float Humidity       = 0.0;     // Humidity (% rel.)
  float Temp_HeatIndex = 0.0;     // calculated heat index in °C
  float Pressure       = 0.0;     // air pressure in hPa
  float ElevationNN    = 0.0;     // calculated elevation above sea level (works like sh...)


  /* --------------------------------------------------------------------------------
   *  SETUP routine
   * --------------------------------------------------------------------------------
   */
  void setup()
  {
    // define LED pin on ESP board and switch off LED
    pinMode(DEBUG_LED, OUTPUT);                
    digitalWrite(DEBUG_LED, HIGH);             


    // initialize serial connection, if debug is ON
    if (SERIAL_DEBUG)
    {
      Serial.begin(9600);
      
      Serial.println();
      Serial.println("W e a t h e r s t a t i o n");
      Serial.println("---------------------------");
      Serial.println();
      
      delay(100);
    }


    // initialize DHT sensor
    if ((SENSOR_TEMPERATURE == 0 ) && (SENSOR_HUMIDITY == 0))
    {
      /* +++ DEBUG +++ */
      if (SERIAL_DEBUG)
        Serial.println("[INIT] - DHT sensor...");
      /* +++ DEBUG +++ */
    
      dht.begin();

      // DHT sensor not ready or not found
      if (!DHT_Status)
      {
        // +++ DEBUG +++
        if (SERIAL_DEBUG)
          Serial.println("[ERROR] - DHT sensor not found");
        // +++ DEBUG +++

        DHT_OK = false;
      }

      // DHT sensor ready
      if (DHT_Status)
        DHT_OK = true;

      delay(100);     // wait a little bit...
    }


    // initialize BME sensor
    if ((SENSOR_TEMPERATURE == 1) || (SENSOR_HUMIDITY == 1) || (SENSOR_PRESSURE == 1))
    {
      /* +++ DEBUG +++ */
      if (SERIAL_DEBUG)
        Serial.println("[INIT] - BME-Sensor...");
      /* +++ DEBUG +++ */

      BME_Status = bme.begin(0x76);
      
      if (!BME_Status)
      {
        // +++ DEBUG +++
        if (SERIAL_DEBUG)
          Serial.println("[ERROR] - BME sensor not found");
        // +++ DEBUG +++

        BME_OK = false;
      }

      if (BME_Status)
      {
        BME_OK = true;
      }

      delay(100);     // wait a little bit...
    }


    // initialize WiFi connection
    /* +++ DEBUG +++ */ 
    if (SERIAL_DEBUG)
      Serial.println("[INIT] - WiFi connection...");
    /* +++ DEBUG +++ */

    WiFi.begin(c_SSID, c_PASSWD);

    // wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED)     // WiFi not connected
    {
      delay(500);
      
      /* +++ DEBUG +++ */
      if (SERIAL_DEBUG)
        Serial.print(".");
      /* +++ DEBUG +++ */
    }

    // WiFi connection established
    if (WiFi.status() == WL_CONNECTED)        // WiFi connected
    {
      /* +++ DEBUG +++ */
      if (SERIAL_DEBUG)
      {
        Serial.println();
        Serial.print("[INFO] - ESP IP address: "); 
        Serial.println(WiFi.localIP());
      }

      WiFi_Connected = true;                  // WiFi connected
    }
  }


/* -------------------------------------------------------------------------------- */


void SendData(int timestamp, byte SensorID, float Temp_C, float humidity, float Temp_heat_index, float pressure, float elevationNN)
/*
 * INPUT
 * timestamp       - INT   - unix timestamp
 * SensorID        - BYTE  - ID of weather station
 * TEMP_C          - FLOAT - temperature
 * humidity        - FLOAT - humidity
 * Temp_heat_index - FLOAT - calculated heat index
 * pressure        - FLOAT - air pressure
 * elevationNN     - FLOAT - elevation above sea level
 * 
 * RETURN
 * nothing...
 */
{
  // define local variables
  String uri;
  String strData;
  String payload;
  bool   httpResult;

  int httpCode;

  // define URL
  uri = c_HOST;

  // build string to be sent to host
  // data = ?Zeitstempel=<Zeitstempel>&Station_ID=<Stations ID>&Temp_C=00.0&Feuchte=00.0&Temp_C_gef=00.0
  strData = String("?Zeitstempel=") + timestamp + "&Station_ID=" + SensorID + "&Temp_C=" + Temp_C + "&Feuchte=" + humidity + "&Temp_C_gef=" + Temp_heat_index + "&Druck=" + pressure + "&HNN=" + elevationNN;
   
  httpResult = http.begin(uri + strData);
  
  if (!httpResult)
  {
    // +++ DEBUG +++
    if (SERIAL_DEBUG)
    {
      Serial.println("[ERROR] - Invalid HTTP request:");
      Serial.println(uri + strData);
    }
  }


  httpCode = http.GET();
  
  if (httpCode > 0)
  { 
    // Request has been made
    payload = http.getString();
    
    // +++ DEBUG +++
    if (SERIAL_DEBUG)
    {
      Serial.print("[INFO] - HTTP status:"); 
      Serial.println(httpCode);
      Serial.println();
      Serial.println(payload);
    }
    // +++ DEBUG +++
  } 
  else
  { 
    // Request could not be made
    // +++ DEBUG +++
    if (SERIAL_DEBUG)
    {
      Serial.print("[ERROR] - HTTP request failed. Error: ");
      Serial.println(http.errorToString(httpCode).c_str());
    }
    // +++ DEBUG +++
  }
  
  http.end();
}


/* -------------------------------------------------------------------------------- */


  /* --------------------------------------------------------------------------------
   *  MAIN LOOP routine
   * --------------------------------------------------------------------------------
   */
  void loop()
  {
    // get time with NTP request
    if (WiFi_Connected == true)
    {
      NTPtime = ntpClient.getUnixTime();

      // +++ DEBUG +++
      if (SERIAL_DEBUG)
      {
        Serial.print("[INFO] - NTP timestamp: ");
        Serial.println(NTPtime);
      }
      // +++ DEBUG +++
    }

    // --------------------------------------------------------------------------------
    // read data from sensors
    // --------------------------------------------------------------------------------

    // reset status
    // Status-Variablen auf FALSE setzen
    Temp_C_OK   = false;
    Humidity_OK = false;
    Pressure_OK = false;


    // read temperature - DHT sensor
    if (SENSOR_TEMPERATURE == 0)
      Temp_C = dht.readTemperature();

    // read temperatur - BME sensor
    if (SENSOR_TEMPERATURE == 1)
      Temp_C = bme.readTemperature();

    // check if temperature value is OK
    // if not... set -99.0 as temperature
    if (isnan(Temp_C))  // temperature is not OK
    {
      Temp_C = -99.0;

      // +++ DEBUG +++
      if (SERIAL_DEBUG)
        Serial.println("[ERROR] - Temperature value not OK!");
      // +++ DEBUG +++
    }
    else // temperature value is OK
      Temp_C_OK = true;


    // read humidity sensor - DHT sensor
    if (SENSOR_HUMIDITY == 0)
      Humidity = dht.readHumidity();

    // read humidity sensor - BME sensor
    if (SENSOR_HUMIDITY == 1) 
      Humidity = bme.readHumidity();

    // check if humidity value is OK
    if (isnan(Humidity)) // humidity value is OK
    {
      Humidity = -99.0;

      // +++ DEBUG +++
      if (SERIAL_DEBUG)
        Serial.println("[ERROR] - Humidity value is not OK!");
      // +++ DEBUG +++
    }
    else // humidity value is not OK
      Humidity_OK = true;


    // read pressure sensor
    if (SENSOR_PRESSURE == 1)
      Pressure = bme.readPressure() / 100.0F;
    else
      Pressure = -99.0;

    // check if pressure value is OK
    if (isnan(Pressure)) // pressure value is not OK
    {
      Pressure = -99.0;

      // +++ DEBUG +++
      if (SERIAL_DEBUG)
        Serial.println("[ERROR] - Pressure value is not OK!");
    }
    else // pressure value is OK
    {
      Pressure_OK = true;
    }


    // --------------------------------------------------------------------------------
    // calculated values
    // --------------------------------------------------------------------------------

    // calculate heat index
    if ((SENSOR_TEMPERATURE == 0) && (SENSOR_HUMIDITY == 0) && (Temp_C_OK == true) && (Humidity_OK == true))
    {
      Temp_HeatIndex = dht.computeHeatIndex(Temp_C, Humidity, false);
    }
    else
    {
      Temp_HeatIndex = -99.0;
    }

    // calculate elevation above sea level
    ElevationNN = bme.readAltitude(SEALEVEL_PRESSURE_HPA);

    // check if elevation value is OK
    if (isnan(ElevationNN)) // value is not OK
    {
      ElevationNN = -99.0;
    }
    else // value is OK
    {
      ElevationNN_OK = true;
    }


    // +++ DEBUG +++
    if (SERIAL_DEBUG)
    {
      Serial.println("C o l l e c t e d   d a t a");
      Serial.println("---------------------------");
      Serial.print("[INFO] - Timestamp: "); Serial.println(NTPtime);
      Serial.print("[INFO] - HOUR - "); Serial.println(hour(NTPtime));
      Serial.print("[INFO] - MINUTE - "); Serial.println(minute(NTPtime));
      Serial.print("[INFO] - SECOND - "); Serial.println(second(NTPtime));
      Serial.println();
      Serial.print("[INFO] - Temperature: "); Serial.print(Temp_C);         Serial.println(" °C");
      Serial.print("[INFO] - Heat index : "); Serial.print(Temp_HeatIndex); Serial.println(" °C");
      Serial.print("[INFO] - Humidity   : "); Serial.print(Humidity);       Serial.println(" % (rel.)");
      Serial.print("[INFO] - Pressure   : "); Serial.print(Pressure);       Serial.println(" hPA");
      Serial.print("[INFO] - Elevation  : "); Serial.print(ElevationNN);    Serial.println(" m");
      Serial.println();
    }


    // --------------------------------------------------------------------------------
    // Transmit data to host
    // --------------------------------------------------------------------------------

    SendData(NTPtime, c_CLIENT_ID, Temp_C, Humidity, Temp_HeatIndex, Pressure, ElevationNN);


    // --------------------------------------------------------------------------------
    // Some cool blinken lights for entertainment :-)
    // --------------------------------------------------------------------------------

    for (int i = 0; i <= 5; i++)
    {
      digitalWrite(DEBUG_LED, LOW);
      delay(100);
      digitalWrite(DEBUG_LED, HIGH);
      delay(100);
    }


    // --------------------------------------------------------------------------------
    // activate DEEP SLEEP MODE to save energy
    // 
    // The ESP board supports DEEP SLEEP MODE to save energy.
    // Sleep time is in ns. 
    // --------------------------------------------------------------------------------
    // +++ DEBUG +++
    if (SERIAL_DEBUG)
      Serial.println("ESP tired... ESP needs some sleep...");
    // +++ DEBUG +++

    ESP.deepSleep(c_REFRESH_CYCLE * 1000000);
  
  }
