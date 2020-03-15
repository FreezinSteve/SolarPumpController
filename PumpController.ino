//====================================================================
// Inverter controller to ensure failover from battery to mains.
// Inverter can handle both pumps running at simultaneously so
// all we need to do is monitor battery voltage and switch to mains
// when the battery voltage drops too low.
//
// If it's night then switch on mains charging, else rely on solar charging
// Use 24V supply for pressure switches, use the switch status to trigger
// relays to switch motors independant of the controller so we can simply
// plug pumps directly into mains if required without controller (will still
// require 24V though!).
//
//
// We need to monitor:
// House pump pressure switch
// Garden pump pressure switch
// Solar battery voltage
//
// We need to control:
// House pump
// Garden pump
// Inverter
// Dump control valve (OPEN)
// Dump control valve (CLOSE)
//
// We need to support:
// NTP time
// No need for low power mode, wifi always on
// OTA Updates
//
// Use modbus control board
//
// A0 - Solar battery
// D0
// D1
// D2 - Test failover switch (or Auto / Mains selection switch)
// D3
// D4 (GPIO2) - Debug output (TX only UART1)
// D5
// D6
// D7
// RX - MODBUS module
// TX - MODBUS module
// RST

// MODBUS module I/O
// IN0  - House pump pressure switch (monitor only, hardwired to operate without controller)
// IN1  - Garden pump pressure switch (monitor only, hardwired to operate without controller)
// IN2  - Inverter override ON switch (or can we use the switch on the inverter itself? We only need to be able to switch to mains on flat battery
// IN3
// IN4
// IN5
// IN6
// IN7
// OUT0 - Inverter ON
// OUT1 - Inverter mains enable
// OUT2 - Backup mains enable
// OUT3 -
// OUT4 -
// OUT5 -
// OUT6 -
// OUT7 -

//=========================================================================
// WiFi
#include <ESP8266WiFi.h>
#include "credentials.h"
//const char* ssid     = "xxxx";
//const char* password = "xxxx";
IPAddress staticIP(192, 168, 1, 71);
IPAddress gateway(192, 168, 1, 250);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);  //DNS
//================================================================
// Neon REST Service
#include "RestClient.h"
#include <ArduinoJson.h>
#include <EEPROM.h>
const char* proxyAddr = "192.168.1.130";
int proxyPort = 9000;
const char* neonURL = "restservice-neon.niwa.co.nz";
//Included in "credentials.h" which is not included in the GIT repository
//const char* neonUser = "xxxxxx";
//const char* neonPassword = "xxxxx";
const char* contentType = "application/json";
const char* importDataPath = "/NeonRESTService.svc/ImportData/5025?LoggerType=1";
const char* getSessionPath = "/NeonRESTService.svc/PostSession";
const int LOG_PERIOD = 900;   // 900 second / 15 minute push rate

// Save battery and state
const byte NEON_CHAN_LEN = 8;      // No channels should be > than this length 'xxx.xx\0'
const byte NEON_CHAN_COUNT = 2;
char mNeonData[NEON_CHAN_COUNT][NEON_CHAN_LEN];
long nextLogTime = 0;
//================================================================
// NTP time synch
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <ESP8266WiFi.h>
static const char ntpServerName[] = "192.168.1.130";
const int timeZone = 0;     // UTC
const unsigned int localPort = 8888;  // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
//================================================================
// OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//=========================================================================

int relayInit = 0;
const int analogSamples = 50;
float battery = 0;
byte in0 = 0;
byte in1 = 0;
byte in2 = 0;
byte in3 = 0;
byte in4 = 0;
byte in5 = 0;
byte in6 = 0;
byte in7 = 0;

byte out[] = {0, 0, 0, 0, 0, 0, 0, 0};
const int RELAY_INVERTER = 0;
const int RELAY_SOLAR = 1;
const int RELAY_MAINS = 2;

byte inverterTimer = 0;
const int OFF_DELAY = 30;   // Keep inverter ON for 30 seconds after use

const float BATT_LOW_VOLTS = 20.0;    // Trip out if battery drops below this
const float BATT_OK_VOLTS = 26.0;     // Resume battery operation when voltage rises above this
const int BATT_STATE_INIT = 0;
const int BATT_STATE_LOW = 1;
const int BATT_STATE_OK = 2;
int batteryState = BATT_STATE_LOW;

const int PIN_TEST_FAILOVER = D2;

// Debug output via Serial1
#define DEBUG Serial1

void setup() {

  // Use Serial0 for Modbus
  Serial.begin(9600);
  // Debug via Tx only UART on GPIO2 (D4).
  Serial1.begin(9600);

  // Test / auto / manual switch
  pinMode(PIN_TEST_FAILOVER, INPUT);

  // Start WiFi
  connectWifi();
  // Init OTA for over the air updates
  initOTA();
  // Init NTP and time
  initTime();
  // init relay module to all off
  initRelays();
  // Delay for when we're reprogramming it
  delay(2000);

  setNextLogTime();
}

void loop() {
  DEBUG.println(getISO8601Time(false));

  // Read battery voltage
  readBattery();

  if (relayInit > 0)
  {
    readModuleInputs();

    checkBattery();
  }
  if (millis() >= nextLogTime)
  {
    loadDataArray();
    pushToNeon();
    setNextLogTime();
  }

  DEBUG.println("");
  delay(1000);
}

//=========================================================================
// Program routines
//=========================================================================
void setNextLogTime()
{
  long secs = millis() / 1000;
  long currInterval = (secs % LOG_PERIOD) * LOG_PERIOD;
  nextLogTime = (currInterval + LOG_PERIOD) * 1000;
}

void readBattery()
{
  long raw = 0;
  for (int i = 0; i < analogSamples; i++)
  {
    raw += analogRead(A0);
  }
  int rawBits = raw / analogSamples;

  // To scale to mV
  // 1:10 divider
  // 0-33000mV = 0 to 3200mV
  // 0-1024bits
  int battmV = map(rawBits, 0, 914, 0, 31600);
  battery = (float)battmV / 1000;
  DEBUG.print("A0: ");
  DEBUG.print(rawBits);
  DEBUG.print("bits,  Battery: ");
  DEBUG.print(battery);
  DEBUG.println("V");

  int test_failover = digitalRead(PIN_TEST_FAILOVER);
  if (test_failover == 1)
  {
    DEBUG.println("Testing failover...");
    battery = BATT_LOW_VOLTS - 1;
  }
}

void checkBattery()
{
  if (batteryState == BATT_STATE_INIT)
  {
    DEBUG.println("Initlialising battery state");
    if (battery < BATT_LOW_VOLTS)
    {
      batteryState = BATT_STATE_LOW;
      DEBUG.println("Battery LOW");
      shutdownInverter();
    }
    else
    {
      batteryState = BATT_STATE_OK;
      DEBUG.println("Battery OK");
      restartInverter();
    }
  }
  else if (batteryState == BATT_STATE_OK)
  {
    if (battery < BATT_LOW_VOLTS)
    {
      batteryState = BATT_STATE_LOW;
      shutdownInverter();
    }
    else
    {
      DEBUG.println("Battery OK");
    }
  }
  else
  {
    if (battery > BATT_OK_VOLTS)
    {
      batteryState = BATT_STATE_OK;
      restartInverter();
    }
    else
    {
      DEBUG.println("Battery LOW, waiting for re-charge");
    }
  }
}

void readModuleInputs()
{
  int state = GetInputStates();
  if (state >= 0)
  {
    // Split out bitmapped value into individual state channels
    in0 = (state & 1) / 1;
    in1 = (state & 2) / 2;
    in2 = (state & 4) / 4;
    in3 = (state & 8) / 8;
    in4 = (state & 16) / 16;
    in5 = (state & 32) / 32;
    in6 = (state & 64) / 64;
    in7 = (state & 128) / 128;
    delay(100);
  }
  else
  {
    Serial1.println("Error reading input states");
  }
}

// Perform a staged safe shutdown
void shutdownInverter()
{
  out[RELAY_INVERTER] = 0;
  out[RELAY_SOLAR] = 0;
  out[RELAY_MAINS] = 1;

  DEBUG.println("Isolating inverter");
  SetRelayState(RELAY_SOLAR, out[RELAY_SOLAR]);
  delay(1000);

  DEBUG.println("Shutting down inverter");
  SetRelayState(RELAY_INVERTER, out[RELAY_INVERTER]);
  delay(1000);

  DEBUG.println("Switching to mains");
  SetRelayState(RELAY_MAINS, out[RELAY_MAINS]);
  delay(100);
}

void restartInverter()
{
  out[RELAY_INVERTER] = 1;
  out[RELAY_SOLAR] = 1;
  out[RELAY_MAINS] = 0;

  DEBUG.println("Isolating mains");
  SetRelayState(RELAY_MAINS, out[RELAY_MAINS]);
  delay(1000);

  DEBUG.println("Switching to inverter");
  SetRelayState(RELAY_SOLAR, out[RELAY_SOLAR]);
  delay(1000);

  DEBUG.println("Starting inverter");
  SetRelayState(RELAY_INVERTER, out[RELAY_INVERTER]);
  delay(100);
}

void loadDataArray()
{
  // Battery to 0, state to 1
  sprintf(mNeonData[0], "%.2f", battery);
  sprintf(mNeonData[1], "%d", batteryState);
}

//=========================================================================
// Wifi routines
//=========================================================================
bool connectWifi() {

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  // Connect to Wifi.
  Serial1.println();
  Serial1.println();
  Serial1.print("Connecting to ");
  Serial1.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long wifiConnectStart = millis();

  while (WiFi.status() != WL_CONNECTED) {
    // Check to see if
    if (WiFi.status() == WL_CONNECT_FAILED) {
      DEBUG.println("Failed to connect to WiFi. Please verify credentials: ");
      delay(10000);
    }

    delay(500);
    DEBUG.println("...");
    // Only try for 5 seconds.
    if (millis() - wifiConnectStart > 15000) {
      DEBUG.println("Failed to connect to WiFi");
      return false;
    }
  }

  DEBUG.println("");
  DEBUG.println("WiFi connected");
  DEBUG.println("IP address: ");
  DEBUG.println(WiFi.localIP());
  DEBUG.println();
  DEBUG.println("Connected!");
  DEBUG.printf("RSSI: %d dBm\n", WiFi.RSSI());
  return true;
}

//=========================================================================
// OTA Routines
//=========================================================================
void initOTA()
{
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("pump_controller");

  ArduinoOTA.onStart([]() {
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  });
  ArduinoOTA.onEnd([]() {
    //
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //
  });
  ArduinoOTA.begin();
}

//==============================================================================
// NTP Routines
//==============================================================================
void initTime()
{
  setSyncProvider(getNtpTime);
  setSyncInterval(3600);
}

time_t getNtpTime()
{
  WiFiUDP Udp;
  Udp.begin(localPort);
  IPAddress ntpServerIP; // NTP server's ip address
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  sendNTPpacket(ntpServerIP, Udp);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      DEBUG.println("NTP started");
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  DEBUG.println("NTP failed to start");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress & address, WiFiUDP &Udp )
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

char* getTime(int offset)
{
  static char timeString[9];

  char buff[3];
  int hr = hour() + offset;
  if (hr >= 24)
  {
    hr -= 24;
  }
  int mi = minute();
  int se = second();

  itoa(hr, buff, 10);
  if (hr < 10)
  {
    strcpy(timeString, "0");
    strcat(timeString, buff);
  }
  else
  {
    strcpy(timeString, buff);
  }
  strcat(timeString, ":");

  itoa(mi, buff, 10);
  if (mi < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  strcat(timeString, ":");

  itoa(se, buff, 10);
  if (se < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  return timeString;
}

char* getISO8601Time(boolean zeroSeconds)
{
  static char timeString[20];

  char buff[5];
  int yr = year();
  int mo = month();
  int da = day();
  int hr = hour();
  int mi = minute();
  int se;
  if (zeroSeconds)
  {
    se = 0;
  }
  else
  {
    se = second();
  }

  itoa(yr, buff, 10);
  strcpy(timeString, buff);
  strcat(timeString, "-");
  itoa(mo, buff, 10);
  if (mo < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  strcat(timeString, "-");
  itoa(da, buff, 10);
  if (da < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  strcat(timeString, "T");
  itoa(hr, buff, 10);
  if (hr < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  strcat(timeString, ":");
  itoa(mi, buff, 10);
  if (mi < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  strcat(timeString, ":");
  itoa(se, buff, 10);
  if (se < 10)
  {
    strcat(timeString, "0");
  }
  strcat(timeString, buff);
  return timeString;
}

//==============================================================================
// Relay routines
//==============================================================================
void initRelays()
{
  byte ret = GetBoardAddress();
  if (ret == 0)
  {
    DEBUG.println("Failed to get MODBUS relay board address!");
    relayInit = 0;
    return;
  }
  else
  {
    DEBUG.print("Relay board address=");
    DEBUG.println(ret);
    delay(100);
  }
  // Start with all relays off
  SetAllRelayState(0);
  relayInit = 1;
}

//==============================================================================
// REST Methods
//==============================================================================
int pushToNeon()
{
  RestClient client = RestClient(neonURL, proxyAddr, proxyPort);
  client.setContentType(contentType);
  char sessionHeader[70];
  int httpStatus = getSessionToken(client, sessionHeader);
  if (httpStatus == 200)
  {
    httpStatus = pushData(client, sessionHeader);
  }
}

int getSessionToken(RestClient &client, char* sessionHeader)
{
  DynamicJsonBuffer sendJsonBuffer;
  JsonObject& cred = sendJsonBuffer.createObject();
  cred["Username"] = neonUser;
  cred["Password"] = neonPassword;
  char json[100];
  cred.printTo(json);
  String response;
  int statusCode = client.post(getSessionPath, json, &response);
  if (statusCode == 200)
  {
    DynamicJsonBuffer recvJsonBuffer;
    JsonObject& root = recvJsonBuffer.parseObject(response);
    strcpy(sessionHeader, "X-Authentication-Token: ");
    strcat(sessionHeader, root.get<String>("Token").c_str());
    client.setHeader(sessionHeader);
  }
  DEBUG.print("#GetSessionToken status code = %d\n");
  DEBUG.println(statusCode);
  return statusCode;
}

int pushData(RestClient &client, char* sessionHeader)
{
  DynamicJsonBuffer jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  JsonArray& Data = root.createNestedArray("Data");

  for (int i = 0; i < NEON_CHAN_COUNT; i++)
  {
    JsonObject& item = Data.createNestedObject();
    item["SensorNumber"] = String(i);
    item["ImportType"] = "0";

    JsonArray& itemSamples = item.createNestedArray("Samples");

    JsonObject& itemSample = itemSamples.createNestedObject();
    itemSample["Time"] = getISO8601Time(true);
    itemSample["Value"] = mNeonData[i];
  }

  char jsonData[1024];
  root.printTo((char*)jsonData, root.measureLength() + 1);

  int statusCode = client.post(importDataPath, (char*)jsonData);
  DEBUG.print("#ImportData status code = %d\n");
  DEBUG.println(statusCode);
  return statusCode;
}
