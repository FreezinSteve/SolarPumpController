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
//
// Use modbus control board
//
// A0 - Solar battery
// D0
// D1 (I2C SCL)
// D2 (I2C SDA)
// D3 (GPIO0) - Test failover switch (or Auto / Mains selection switch)
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
// OUT1 - ON = Solar, OFF = Mains
// OUT2 - Isolate load (load disconnected before switching source)
// OUT3 - Red LED = On MAINS
// OUT4 - Green LED = On SOLAR
// OUT5 - Green LED = Comms state
// OUT6 - Cooling fan
// OUT7 - MAINS Charger

//=========================================================================
// WiFi
#include <ESP8266WiFi.h>
#include "credentials.h"
//const char* ssid     = "xxxx";
//const char* password = "xxxx";
//IPAddress staticIP(192, 168, 1, 71);
//IPAddress gateway(192, 168, 1, 250);
//IPAddress subnet(255, 255, 255, 0);
//IPAddress dns(8, 8, 8, 8);  //DNS
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
const byte NEON_CHAN_COUNT = 3;
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

//===========================================================
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
const int RELAY_SOURCE = 1;
const int RELAY_ISOLATE = 2;
const int RELAY_MAINS_LED = 3;
const int RELAY_SOLAR_LED = 4;
const int RELAY_COMMS_LED = 5;
const int RELAY_FAN = 6;
const int RELAY_CHARGER = 7;

const float BATT_LOW_VOLTS = 23.5;    // Trip out if battery drops below this
const float BATT_OK_VOLTS = 28.0;     // Resume battery operation when voltage rises above this
const float MAIN_CHARGE_START = 24.0;  // Start mains charger below this
const int BATT_STATE_INIT = 0;
const int BATT_STATE_LOW = 1;
const int BATT_STATE_OK = 2;
const int CHARGE_STATE_INIT = 0;
const int CHARGE_STATE_ON = 1;
const int CHARGE_STATE_OFF = 2;

int batteryState = BATT_STATE_INIT;
int chargerState = CHARGE_STATE_INIT;
float battMin = 30;       // minimum measured battery for this log interval
float battAvgTotal = 0;   // Accumulator for battery mean
int battAvgCount = 0;
int switchDelayTimer = 0;           // Countdown timer to delay switch from mains to solar
int chargeDelayTimer = 0;           // Countdown timer to delay mains charger from switching off
const int DELAY_TIME = 300;        // Leave switched to mains for at least 300 seconds before switching back to solar. This allows time for the battery to charge without excessive switching
const int CHARGE_TIME = 300;      // Leave mains charger on for at lease 300 seconds
const int PIN_TEST_FAILOVER = D3;
// Debug output via Serial1
#define DEBUG Serial1
//================================================================
// TMP102 temperature monitoring
#include "Wire.h"
#define TMP102_I2C_ADDRESS 72 /* This is the I2C address for our chip. This value is correct if you tie the ADD0 pin to ground. See the datasheet for some other values. */
float cabinetTemp = 0.0;
int fanState = 0;
const int FAN_STATE_INIT = 0;
const int FAN_STATE_OFF = 1;
const int FAN_STATE_ON = 2;
const float TEMP_FAN_ON = 30.0;
const float TEMP_FAN_OFF = 25.0;

void setup() {

  // Use Serial0 for Modbus
  Serial.begin(9600);
  // Debug via Tx only UART on GPIO2 (D4).
  Serial1.begin(9600);
  DEBUG.println("");
  // Test / auto / manual switch
  pinMode(PIN_TEST_FAILOVER, INPUT_PULLUP);

  Wire.begin();

  startWiFi();

  // Init NTP and time
  initTime();

  // init relay module to all off
  initRelays();
  // Delay for when we're reprogramming it
  delay(2000);

  setNextLogTime();

  stopWiFi();
}

void loop() {
  DEBUG.println(getISO8601Time(false));

  // Read battery voltage
  readBattery();
  readTMP102();

  if (relayInit > 0)
  {
    readModuleInputs();
    checkBattery();
  }

  checkTemperature();
  checkCharger();

  DEBUG.print("Seconds till Neon Push: ");
  DEBUG.println((nextLogTime - millis()) / 1000);
  if (millis() >= nextLogTime)
  {
    if (startWiFi())
    {
      // Force refresh of NTP time Autosync doesn't work very well when we have an intermittent connection
      setSyncProvider(getNtpTime);
      loadDataArray();
      pushToNeon();
      stopWiFi();
    }
    else
    {
      DEBUG.println("WiFi failed, data push missed");
    }
    setNextLogTime();
    stopWiFi();
  }

  DEBUG.println("");
  delay(1000);
}

//=========================================================================
// Program routines
//=========================================================================
void setNextLogTime()
{
  int secs = minute() * 60 + second();
  int secsIntoInterval = (secs % LOG_PERIOD);
  DEBUG.print("Seconds into interval: ");
  DEBUG.println(secsIntoInterval);
  nextLogTime = millis() + (LOG_PERIOD - secsIntoInterval) * 1000;
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
  // Update totals for averaging
  battAvgTotal += battery;
  battAvgCount ++;
  // Capture minimum
  if (battMin > battery)
  {
    battMin = battery;
  }

  DEBUG.print("A0: ");
  DEBUG.print(rawBits);
  DEBUG.print("bits,  Battery: ");
  DEBUG.print(battery);
  DEBUG.println("V");

  int test_failover = digitalRead(PIN_TEST_FAILOVER);
  if (test_failover == 0)   // Pull low to test
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
      switchDelayTimer = DELAY_TIME;
      shutdownInverter();
    }
    else
    {
      DEBUG.println("Battery OK");
    }
  }
  else    // BATT_STATE_LOW
  {
    if (switchDelayTimer > 0)
    {
      switchDelayTimer--;
      DEBUG.print("Battery LOW, waiting for switch timer to expire:");
      DEBUG.println(switchDelayTimer);
    }
    else if (battery > BATT_OK_VOLTS)
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

void readTMP102()
{
  /* Reset the register pointer (by default it is ready to read temperatures)
    You can alter it to a writeable register and alter some of the configuration -
    the sensor is capable of alerting you if the temperature is above or below a specfied threshold. */

  Wire.beginTransmission(TMP102_I2C_ADDRESS); //Say hi to the sensor.
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(TMP102_I2C_ADDRESS, 2);
  Wire.endTransmission();

  byte b1 = Wire.read();
  byte b2 = Wire.read();

  int temp16 = (b1 << 4) | (b2 >> 4);    // builds 12-bit value
  DEBUG.print("Integer data before conversion: ");
  DEBUG.println(temp16);
  cabinetTemp = temp16 * 0.0625;
  DEBUG.print("Cabinet temperature = ");
  DEBUG.print(cabinetTemp, 2);
  DEBUG.println("degC");
}

void checkTemperature()
{
  if (fanState == FAN_STATE_INIT)
  {
    // OFF on first run or error
    SetRelayState(RELAY_FAN, 0);
    fanState = FAN_STATE_OFF;
  }
  else if (fanState == FAN_STATE_OFF)
  {
    if (cabinetTemp > TEMP_FAN_ON)
    {
      if (fanState != FAN_STATE_ON)
      {
        SetRelayState(RELAY_FAN, 1);
        fanState = FAN_STATE_ON;
      }
    }
  }
  else
  {
    if (cabinetTemp < TEMP_FAN_OFF)
    {
      if (fanState != FAN_STATE_OFF)
      {
        SetRelayState(RELAY_FAN, 0);
        fanState = FAN_STATE_OFF;
      }
    }
  }
}

void checkCharger()
{
  // Check to see if we should turn on the mains charger
  int hr = hour() + 12;   // UTC OFFSET
  if (hr > 24) {
    hr -= 24;
  }
  if (hr >= 20 || hr < 6)
  {
    // 8pm to 6am mains charger on
    if (chargerState != CHARGE_STATE_ON)
    {
      SetRelayState(RELAY_CHARGER, 1);
      chargerState = CHARGE_STATE_ON;
      chargeDelayTimer = 0;
      DEBUG.println("Night time, starting mains charger");
    }
  }
  else
  {
    if (chargeDelayTimer == 0)
    {
      // Turn on for "n" seconds if battery drops below BATT_MAIN_CHARGE
      if (battery < MAIN_CHARGE_START)
      {
        if (chargerState == CHARGE_STATE_OFF)
        {
          DEBUG.println("Batt low, starting mains charger");
          SetRelayState(RELAY_CHARGER, 1);
          chargerState = CHARGE_STATE_ON;
          chargeDelayTimer = CHARGE_TIME;
        }
      }
    }
    else
    {
      if (battery < MAIN_CHARGE_START)
      {
        // Still low
        chargeDelayTimer = CHARGE_TIME;
      }
      else
      {
        chargeDelayTimer--;
        DEBUG.print("Mains charger delay timer = ");
        DEBUG.print(chargeDelayTimer);
        if (chargeDelayTimer == 0)
        {
          SetRelayState(RELAY_CHARGER, 0);
          chargerState = CHARGE_STATE_OFF;
          DEBUG.println("Battery recovered, timer expired, stopping mains charger");
        }
      }
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
  DEBUG.println("Isolating load");
  SetRelayState(RELAY_ISOLATE, 1);
  delay(500);

  DEBUG.println("Shutting down inverter");
  SetRelayState(RELAY_INVERTER, 0);
  delay(500);

  DEBUG.println("Switching to mains");
  SetRelayState(RELAY_SOURCE, 0);
  delay(500);

  DEBUG.println("Reconnecting load");
  SetRelayState(RELAY_ISOLATE, 0);
  delay(500);

  SetRelayState(RELAY_MAINS_LED, 1);
  delay(100);
  SetRelayState(RELAY_SOLAR_LED, 0);

  // Final state
  out[RELAY_INVERTER] = 0;
  out[RELAY_SOURCE] = 0;
  out[RELAY_ISOLATE] = 0;
}

void restartInverter()
{
  DEBUG.println("Isolating load");
  SetRelayState(RELAY_ISOLATE, 1);
  delay(500);

  DEBUG.println("Switching to inverter");
  SetRelayState(RELAY_SOURCE, 1);
  delay(500);

  DEBUG.println("Starting inverter");
  SetRelayState(RELAY_INVERTER, 1);
  delay(500);

  DEBUG.println("Reconnecting load");
  SetRelayState(RELAY_ISOLATE, 0);
  delay(500);

  SetRelayState(RELAY_MAINS_LED, 0);
  delay(100);
  SetRelayState(RELAY_SOLAR_LED, 1);

  // Final state
  out[RELAY_INVERTER] = 1;
  out[RELAY_SOURCE] = 1;
  out[RELAY_ISOLATE] = 0;
}

void loadDataArray()
{
  // calculate mean battery voltage for storage
  float avgBatt = battAvgTotal / battAvgCount;
  battAvgTotal = 0;
  battAvgCount = 0;

  // Battery to 0, state to 1
  sprintf(mNeonData[0], "%.2f", avgBatt);
  sprintf(mNeonData[1], "%d", batteryState);
  sprintf(mNeonData[2], "%.2f", battMin);

  battMin = 30;
}

//=========================================================================
// Wifi routines
//=========================================================================
bool startWiFi()
{
  // Wake up WiFi and connnect
  WiFi.forceSleepWake();
  DEBUG.println("WiFi awake");
  delay( 1 );
  if (WiFi.status() == WL_CONNECTED) {
    flashConnect();
    return true;
  }

  // Connect to Wifi.
  WiFi.mode(WIFI_STA);

  for (int retry = 0; retry < 2; retry++)
  {
    DEBUG.print("Connecting to: ");
    if (retry == 0)
    {
      WiFi.begin(ssid, password);
      DEBUG.print(ssid);
    }
    else
    {
      WiFi.begin(backup_ssid, backup_password);
      DEBUG.print(backup_ssid);
    }
    unsigned long wifiConnectStart = millis();
    while (true) {
      // Check to see if
      if (WiFi.status() == WL_CONNECT_FAILED) {
        DEBUG.println("connection failed...");
        delay(1000);
        break;
      }
      else if (WiFi.status() == WL_CONNECTED)
      {
        break;
      }
      delay(500);
      DEBUG.println("...");
      // Only try for 'n' seconds.
      if (millis() - wifiConnectStart > 15000) {
        DEBUG.println("connection timed out...");
        break;
      }
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      break;
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG.println("failed to connect to WiFi");
    //Flash RED LED
    flashFailConnect();
    return false;
  }

  DEBUG.println("");
  DEBUG.println("WiFi connected");
  DEBUG.println("IP address: ");
  DEBUG.println(WiFi.localIP());
  DEBUG.println();
  DEBUG.println("Connected!");
  DEBUG.printf("RSSI: %d dBm\n", WiFi.RSSI());

  flashConnect();
  return true;

}

void stopWiFi()
{
  // Power down wifi
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
  DEBUG.println("WiFi power down");
}

bool connectWifi() {

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  // Connect to Wifi.
  WiFi.mode(WIFI_STA);

  for (int retry = 0; retry < 2; retry++)
  {
    DEBUG.print("Connecting to: ");
    if (retry == 0)
    {
      WiFi.begin(ssid, password);
      DEBUG.print(ssid);
    }
    else
    {
      WiFi.begin(backup_ssid, backup_password);
      DEBUG.print(backup_ssid);
    }
    unsigned long wifiConnectStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
      // Check to see if
      if (WiFi.status() == WL_CONNECT_FAILED) {
        DEBUG.println("connection failed...");
        delay(1000);
        break;
      }
      delay(500);
      DEBUG.println("...");
      // Only try for 'n' seconds.
      if (millis() - wifiConnectStart > 15000) {
        DEBUG.println("connection timed out...");
        break;
      }
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG.println("failed to connect to WiFi");
    return false;
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
void sendNTPpacket(IPAddress & address, WiFiUDP & Udp )
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

int getSessionToken(RestClient & client, char* sessionHeader)
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

int pushData(RestClient & client, char* sessionHeader)
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

//===============================================================
// Notification routines
void flashFailConnect()
{
  // quick flashes
  for (int i = 0; i < 4; i++)
  {
    SetRelayState(RELAY_COMMS_LED, 1);
    delay(250);
    SetRelayState(RELAY_COMMS_LED, 0);
    delay(250);
  }
}

void flashConnect()
{
  // 1 long flash
  for (int i = 0; i < 2; i++)
  {
    SetRelayState(RELAY_COMMS_LED, 1);
    delay(1000);
    SetRelayState(RELAY_COMMS_LED, 0);
    delay(500);
  }
}
