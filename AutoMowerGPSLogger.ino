#include <WiFi.h>
#include "secrets.h"
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

TaskHandle_t Task1; // GPS things
TaskHandle_t Task2; // HTTP post things

// Array holding GPS positions
// latitude, longitude, accuracyM, fixType, gpsTs
//float gpsLocations[60][5];

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <Wire.h> // Needed for I2C to GNSS
//#define myWire Wire // This will work on the Redboard Artemis and the Artemis Thing Plus using Qwii
SFE_UBLOX_GNSS myGNSS;
//long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

const int nmeaMaxStrings = 240;
String nmeaStrings[nmeaMaxStrings];
volatile int nmeaStringCounter = 0;
unsigned int nmeaTotalCounter = 0;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Define hardware serials to be used for GPS communication and serial Bluetooth for debugging
HardwareSerial serialGNSS(1); //TX on 17, RX on 16
HardwareSerial serialBT(2); //TX on 17, RX on 16


void setup() {
  Serial.begin(115200);
  serialGNSS.begin(115200, SERIAL_8N1, 27, 26);
  serialGNSS.setRxBufferSize(1024);

  serialBT.begin(57600, SERIAL_8N1, 32, 33);

  Wire.begin(21, 22);
  while (myGNSS.begin(Wire) == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    serialBT.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    delay(1000);
  }

  // Check that this platform supports 64-bit (8 byte) double
  if (sizeof(double) < 8)
  {
    Serial.println(F("Warning! Your platform does not support 64-bit double."));
    Serial.println(F("The latitude and longitude will be inaccurate."));
  }

  //myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

  myGNSS.setNavigationFrequency(1); //Set output in Hz.

  byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);
  serialBT.print("Current update rate: ");
  serialBT.println(rate);

  Serial.print(F("Connecting to local WiFi"));
  serialBT.print(F("Connecting to local WiFi"));

  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.print(F("."));
    serialBT.print(F("."));
    delay(1000);
  }
  Serial.println();

  Serial.print(F("WiFi connected with IP: "));
  Serial.println(WiFi.localIP());

  serialBT.print(F("WiFi connected with IP: "));
  serialBT.println(WiFi.localIP());


  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    serialBT.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
    serialBT.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    serialBT.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);

  //Intrrupt WTD
  esp_int_wdt_init();

  //TWDT
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(Task2);
}

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ) {
  delay(1000);
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  WiFiClient ntripClient;
  long rtcmCount = 0;

  long timeSinceLastCasterConnectAttempt = 0;

  String nmea;
  //String gngst;

  for (;;) {

    while (serialGNSS.available() > 0) {
      char a = serialGNSS.read();
      if (a == '\n') {
        if (nmea.startsWith("$GNGGA")) {

          Serial.print("$GNGGA: ");
          Serial.println(nmea);
          serialBT.print("$GNGGA: ");
          serialBT.println(nmea);

          nmeaStrings[nmeaStringCounter] = nmea;
          //nmeaStrings[nmeaStringCounter] = nmea + gngst;

          nmeaTotalCounter++;

          // roll-over if needed
          if (nmeaStringCounter >= (nmeaMaxStrings - 1)) {
            nmeaStringCounter = 0;
          } else {
            nmeaStringCounter++;
          }
        }

        // reset string
        nmea = "";
      }
      else if (a == '\r') {
      }
      else {
        nmea += a;
      }
    }

    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false && timeSinceLastCasterConnectAttempt < (millis() - 5000))
    {
      timeSinceLastCasterConnectAttempt = millis();
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("Subscribing to Caster."));
        Serial.print(F("Opening socket to "));
        Serial.println(casterHost);

        serialBT.println(F("Subscribing to Caster."));
        serialBT.print(F("Opening socket to "));
        serialBT.println(casterHost);

        if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
        {
          Serial.println(F("Connection to caster failed"));
          serialBT.println(F("Connection to caster failed"));
          //return;
        }
        else
        {
          Serial.print(F("Connected to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(casterPort);

          Serial.print(F("Requesting NTRIP Data from mount point "));
          Serial.println(mountPoint);

          serialBT.print(F("Connected to "));
          serialBT.print(casterHost);
          serialBT.print(F(": "));
          serialBT.println(casterPort);

          serialBT.print(F("Requesting NTRIP Data from mount point "));
          serialBT.println(mountPoint);

          const int SERVER_BUFFER_SIZE  = 512;
          char serverRequest[SERVER_BUFFER_SIZE];

          snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                   mountPoint);

          char credentials[512];
          if (strlen(casterUser) == 0)
          {
            strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
          }
          else
          {
            //Pass base64 encoded user:pw
            char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
            snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

            Serial.print(F("Sending credentials: "));
            Serial.println(userCredentials);
            serialBT.print(F("Sending credentials: "));
            serialBT.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
            //Encode with ESP32 built-in library
            base64 b;
            String strEncodedCredentials = b.encode(userCredentials);
            char encodedCredentials[strEncodedCredentials.length() + 1];
            strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
            snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
            //Encode with nfriendly library
            int encodedLen = base64_enc_len(strlen(userCredentials));
            char encodedCredentials[encodedLen]; //Create array large enough to house encoded data
            base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif
          }
          strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
          strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

          Serial.print(F("serverRequest size: "));
          Serial.print(strlen(serverRequest));
          Serial.print(F(" of "));
          Serial.print(sizeof(serverRequest));
          Serial.println(F(" bytes available"));

          Serial.println(F("Sending server request:"));
          Serial.println(serverRequest);

          serialBT.print(F("serverRequest size: "));
          serialBT.print(strlen(serverRequest));
          serialBT.print(F(" of "));
          serialBT.print(sizeof(serverRequest));
          serialBT.println(F(" bytes available"));

          serialBT.println(F("Sending server request:"));
          serialBT.println(serverRequest);

          ntripClient.write(serverRequest, strlen(serverRequest));

          //Wait for response
          unsigned long timeout = millis();

          //if (ntripClient.available() == 0)
          while (ntripClient.available() == 0)
          {
            if (millis() - timeout > 5000)
            {
              Serial.println(F("Caster timed out!"));
              serialBT.println(F("Caster timed out!"));
              ntripClient.stop();
              delay(1000);
              goto bailout;
              //return;
            }
            delay(10);
          }

          //Check reply
          bool connectionSuccess = false;
          char response[512];
          int responseSpot = 0;
          while (ntripClient.available())
          {
            if (responseSpot == sizeof(response) - 1) break;

            response[responseSpot++] = ntripClient.read();
            if (strstr(response, "200") > 0) //Look for 'ICY 200 OK'
              connectionSuccess = true;
            if (strstr(response, "401") > 0) //Look for '401 Unauthorized'
            {
              Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
              serialBT.println(F("Hey - your credentials look bad! Check you caster username and password."));
              connectionSuccess = false;
            }
          }
          response[responseSpot] = '\0';

          Serial.print(F("Caster responded with: "));
          Serial.println(response);
          serialBT.print(F("Caster responded with: "));
          serialBT.println(response);

          if (connectionSuccess == false)
          {
            Serial.print(F("Failed to connect to "));
            Serial.print(casterHost);
            Serial.print(F(": "));
            Serial.println(response);

            serialBT.print(F("Failed to connect to "));
            serialBT.print(casterHost);
            serialBT.print(F(": "));
            serialBT.println(response);
            //return;
          }
          else
          {
            Serial.print(F("Connected to "));
            Serial.println(casterHost);
            serialBT.print(F("Connected to "));
            serialBT.println(casterHost);
            lastReceivedRTCM_ms = millis(); //Reset timeout
          }
        } //End attempt to connect

      } else {
        //Serial.println(F("Caster needs WiFi!."));
      }
    } //End connected == false

    if (ntripClient.connected() == true)
    {
      uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available())
      {
        //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData)) break;
      }

      if (rtcmCount > 0)
      {
        lastReceivedRTCM_ms = millis();

        //Push RTCM to GNSS module over I2C
        myGNSS.pushRawData(rtcmData, rtcmCount, false);
        Serial.print(F("RTCM pushed to ZED: "));
        Serial.println(rtcmCount);
        serialBT.print(F("RTCM pushed to ZED: "));
        serialBT.println(rtcmCount);
      }
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms && ntripClient.connected() == true)
    {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      //if (ntripClient.connected() == true) {
      Serial.println("ntripClient.stop()");
      serialBT.println(F("RTCM timeout. Disconnecting..."));
      serialBT.println("ntripClient.stop()");

      ntripClient.stop();
      //delay(10000);
      //}
      //return;
    }
bailout:
    delay(10);
  }
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ) {
  delay(1000);
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  unsigned int sentStringsWatermark = 0;
  unsigned int tempWatermark;
  int maxNemaPost = 30;

  volatile int currPos = 0; //nmeaMaxStrings

  int delayMillis = 10000;
  bool skipDelay = false;

  for (;;) {
    Serial.println("1");
    serialBT.println("1");

    // Maintain watchdog. Triggerd if HTTP Post is hung
    esp_task_wdt_reset();

    long startMillis = millis();

    if ((WiFi.status() == WL_CONNECTED)) {

      tempWatermark = sentStringsWatermark;

      String httpRequestData = "[\"";
      int nmeaCounter = 0;

      //for (long i = tempWatermark; i < nmeaTotalCounter; i++) {
      serialBT.print("sentStringsWatermark: ");
      serialBT.println(sentStringsWatermark);
      serialBT.print("nmeaTotalCounter: ");
      serialBT.println(nmeaTotalCounter);
      //serialBT.print("tempWatermark: ");
      //serialBT.println(tempWatermark);

      // Loop thru the number of rows that has added since last time
      for (int i = 0; i < nmeaTotalCounter - sentStringsWatermark ; i++) {
        nmeaCounter++;

        if (nmeaStrings[currPos].length() == 0)
          break;

        Serial.print("Adding NMEA String: ");
        Serial.println(nmeaStrings[currPos]);
        serialBT.print("Adding NMEA String: ");
        serialBT.println(nmeaStrings[currPos]);
        serialBT.print("currPos: ");
        serialBT.println(currPos);

        httpRequestData += nmeaStrings[currPos] + "\",\"";

        tempWatermark++;
        currPos++;

        if (currPos == nmeaMaxStrings) {
          currPos = 0;
        }

        // break before the data that we want to send becomes to big
        if (nmeaCounter == maxNemaPost) {
          //skipDelay = true;
          break;
        }
      }

      int lastIndex = httpRequestData.length() - 3;
      httpRequestData.remove(lastIndex, 3);
      httpRequestData += "\"]";

      // Maintain watchdog. Triggerd if HTTP Post is hung
      esp_task_wdt_reset();

      Serial.println("Starting HTTP Post function");
      serialBT.println("Starting HTTP Post function");
      HTTPClient http;
      http.setConnectTimeout(4000);
      http.begin(serverName);
      http.setTimeout(4000);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("API-KEY", apiKey);

      // Send HTTP POST request
      int httpResponseCode = http.POST(httpRequestData);

      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        serialBT.print("HTTP Response code: ");
        serialBT.println(httpResponseCode);
        if (httpResponseCode == 200) {
          sentStringsWatermark = tempWatermark;

          // If more items needs to be sent, reduce the delay to the next HTTP POST
          if (nmeaCounter == maxNemaPost) {
            skipDelay = true;
          } else {
            skipDelay = false;
          }
        }
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
        serialBT.print("Error code: ");
        serialBT.println(httpResponseCode);
        skipDelay = false;
      }

      // Free resources
      http.end();
      Serial.print("HTTP Post took: ");
      Serial.println(millis() - startMillis);
      serialBT.print("HTTP Post took: ");
      serialBT.println(millis() - startMillis);

      // Maintain watchdog. Triggerd if HTTP Post is hung
      esp_task_wdt_reset();

      if (skipDelay == true) {
        delay(500);
      } else {
        delay(delayMillis - (millis() - startMillis));
      }
    }

    // Else reconnect to wifi
    else {
      Serial.println("Reconnecting Wifi");
      serialBT.println("Reconnecting Wifi");
      WiFi.reconnect();
      delay(5000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();
}
