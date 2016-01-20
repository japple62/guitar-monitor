/***************************************************
  Adafruit MQTT Library CC3000 Example

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <Adafruit_SleepyDog.h>
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include "utility/debug.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_CC3000.h"
#include "DHT.h"
#include <LiquidCrystal.h>

/*************************** CC3000 Pins ***********************************/

#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  5  // VBAT & CS can be any digital pins.
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "SID"  // can't be longer than 32 characters!
#define WLAN_PASS       "PASSWD"
#define WLAN_SECURITY   WLAN_SEC_WPA2  // Can be: WLAN_SEC_UNSEC, WLAN_SEC_WEP,
//         WLAN_SEC_WPA or WLAN_SEC_WPA2

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "AUSER"
#define AIO_KEY         "AKEY"

// Temp sensor configuration
#define     HUMIDITY_WARN_LIMIT 40
#define     DHTTYPE DHT22   // DHT 22  (AM2302)
#define     TEMP_SENSOR            2         // Digital pin that is hooked up to the Temp sensor.
#define     HUMIDITY_CORRECTION    -9        // Calibration
#define     TEMP_CORRECTION        -1        // Calibration

#define     CHECK_SECONDS     5         // How long to wait (in seconds) between pet food dish level checks.
#define     TIMEOUT_MS        15000     // How long to wait (in milliseconds) for a server connection to respond (for both AWS and NTP calls).
#define     PUBLISH_LIMIT_SECONDS  900     // How long to wait (in seconds) between publish calls.  This will prevent too many messages from being published in a short period.

/************ Global State (you don't need to change this!) ******************/

// Setup the main CC3000 class, just like a normal CC3000 sketch.
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT);

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the CC3000 MQTT class by passing in the CC3000 class and MQTT server and login details.
Adafruit_MQTT_CC3000 mqtt(&cc3000, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

// You don't need to change anything below this line!
//#define halt(s) { Serial.println(F( s )); while(1);  }

// CC3000connect is a helper function that sets up the CC3000 and connects to
// the WiFi network. See the cc3000helper.cpp tab above for the source!
//boolean CC3000connect(const char* wlan_ssid, const char* wlan_pass, uint8_t wlan_security);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char HUMIDITY_FEED[] PROGMEM = AIO_USERNAME "/feeds/guitar-humidity";
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);


/*************************** Sketch Code ************************************/

unsigned long lastPolledTime = 0;   // Last value retrieved from time server.
unsigned long sketchTime = 0;       // CPU milliseconds since last time server query.

unsigned long lastTempCheck = 0;    // The last time the bowl status was checked.
unsigned long lastIOPublish = 0;   // The last time an SNS message was published.

DHT dht(TEMP_SENSOR, DHTTYPE);
LiquidCrystal lcd(7, 8, 9, 6, 11, 12);


void setup() {


  // setup LCD
  lcd.begin(16, 2);

  // Set up the input
  pinMode(TEMP_SENSOR, INPUT);

  Serial.begin(115200);

  //Serial.print(F("in setup"));

  // Set up the CC3000, connect to the access point, and get an IP address.
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while (1);
  }
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while (1);
  }
  Serial.println(F("Connected!"));
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100);
  }

  // Get an initial time value by querying an NTP server.
  unsigned long t = getTime();
  while (t == 0) {
    // Failed to get time, try again in a minute.
    delay(60 * 1000);
    t = getTime();
  }
  lastPolledTime = t;
  sketchTime = millis();


  //Serial.print("leaving setup\n");
}

uint32_t x = 0;


void loop() {
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Update time
  // Update the current time.
  unsigned long currentTime = lastPolledTime + (millis() - sketchTime) / 1000;
  //Serial.print("CurrentTime"); Serial.println(currentTime);


  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();


  float h = dht.readHumidity();
  float temp = dht.readTemperature();

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(temp) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    /*Serial.print("Humidity: ");
    Serial.print(h + HUMIDITY_CORRECTION);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print( (temp * 9.0) / 5.0 + 32.0);
    Serial.println(" *F");*/

    lcd.setCursor(0, 0);
    lcd.print("Humidity:");
    lcd.print(h + HUMIDITY_CORRECTION);
    lcd.print(" %");

    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    lcd.print( ((temp * 9.0) / 5.0 + 32.0) + TEMP_CORRECTION);
    lcd.print(" F");


  }

  // Now we can publish stuff!

  Serial.print(h + HUMIDITY_CORRECTION);


  if (currentTime - lastIOPublish >= PUBLISH_LIMIT_SECONDS) {

    Serial.print(F("\nSending humidity  "));

    if (!humidity.publish(h + HUMIDITY_CORRECTION)) {
      Serial.println(F("Publish Failed"));
    } else {
      Serial.println(F("OK!"));
    }

  }

  // ping the server to keep the mqtt connection alive
  if (! mqtt.ping()) {
    Serial.println(F("MQTT Ping failed."));
  }

  delay(100);

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    //Serial.print("Already connected to MQTT... ");
    return;
  }


  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    if (ret < 0)
      /*if (!cc3000.begin())
      {
        Serial.println(F("Couldn't begin()! Check your wiring?"));
        while (1);
      }*/
      if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
        Serial.println(F("Failed!"));
        while (1);
      }
    Serial.println(F("Connected!"));

    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}


// getTime function adapted from CC3000 ntpTest sketch.
// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {
  Adafruit_CC3000_Client client;
  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if (cc3000.getHostByName("pool.ntp.org", &ip)) {
    static const char PROGMEM
    timeReqA[] = { 227,  0,  6, 236 },
                 timeReqB[] = {  49, 78, 49,  52 };
    //Serial.print(ip);
    startTime = millis();
    do {
      client = cc3000.connectUDP(ip, 123);
    } while ((!client.connected()) &&
             ((millis() - startTime) < TIMEOUT_MS));

    if (client.connected()) {
      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      client.write(buf, sizeof(buf));

      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while ((!client.available()) &&
             ((millis() - startTime) < TIMEOUT_MS));
      if (client.available()) {
        client.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
             (unsigned long)buf[43]) - 2208988800UL;
      }
      client.close();
    }
  }
  //Serial.print(t); Serial.println(" leaving get time");
  return t;
}
