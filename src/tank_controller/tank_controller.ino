#include <TimeLib.h>
#include "EmonLib.h"
#include <SPI.h>
#include <Ethernet.h>
#include <aREST.h>
#include <avr/wdt.h>
#include <NTPClient.h>
#include <EthernetUdp.h>
#include <DS3231.h>
#include <DallasTemperature.h>

#define ETH_CS    10
#define SD_CS  4
#define ONE_WIRE_BUS 12

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
EnergyMonitor emon1;

// Ethernet UDP
EthernetUDP ntpUDP;

// NTP client
NTPClient timeClient(ntpUDP, "192.168.5.1", 0, 60000);

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x12 };

// IP address in case DHCP fails
IPAddress ip(192, 168, 5, 221);

// Gateway adress
IPAddress gateway(192, 168, 5, 1);

// Ethernet server
EthernetServer server(80);

// Create aREST instance
aREST rest = aREST();

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

// Used for detecting time and runnning time based commands.
int currentHour;
int currentMinute;

// Variables to be exposed to the API
float waterTemperature;
float baskingTemperature;
float internalTemperature;
String startupTime;
String currentTime;
String waterLow;
String leak;
int leakValue;
float amps;
float kWh;
float yesterdays_total_kWh;

int loopCount = 0;

// Constants for input/output pins
const int outlet1 = 2;
const int outlet2 = 3;
const int outlet3 = 11;
const int outlet4 = 5;
const int outlet5 = 6;
const int outlet6 = 7;
const int outlet7 = 8;
const int outlet8 = 9;
const int leakPin = A0;
const int outletPins[] = {outlet1, outlet2, outlet3, outlet4, outlet5, outlet6, outlet7, outlet8};
const int outputPinCount = 8;
const int waterLevel = 13;
const int inputPins[] = {waterLevel};
const int inputPinCount = 1;

const float voltage = 117;

bool nightTime = false;
bool dayTime = false;

long start_time_unix;
long new_time;
double last_power;;

void setup(void)
{
  // Start Serial
  Serial.begin(115200);
  Serial.println("Starting up...");

  // Make the Ethernet module actually work.
  pinMode(ETH_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  delay(100);
  digitalWrite(ETH_CS, LOW); // Select the Ethernet Module.
  delay(100);
  digitalWrite(SD_CS, HIGH); // De-Select the internal SD Card

  // Check for a leak
  leakValue = analogRead(leakPin);
  if (leakValue > 100) {
    leak = "true";
  } else {
    leak = "false";
  }

  // Set the pins to input/output mode
  setupPins();

  // Enable the rtc and the ds18b20 temperature sensors
  rtc.begin();
  sensors.begin();

  // Start up current sensor
  emon1.current(1, 57.63);
  amps = emon1.calcIrms(1480);
  start_time_unix = rtc.getUnixTime(rtc.getTime());

  // Set the current hour and minute for use with time based functions
  currentHour = rtc.getTime().hour;
  currentMinute = rtc.getTime().min;

  // If it is after 10pm and before 9am EST
  if ((currentHour >= 3 && currentHour < 14)) {
    if (!nightTime) {
      night("");
    }
  } else {
    if (!dayTime) {
      morning("");
    }
  }

  // Init variables and expose them to REST API
  kWh = 0.0;
  yesterdays_total_kWh = 0.0;
  internalTemperature = sensors.getTempCByIndex(0);
  waterTemperature = sensors.getTempCByIndex(1);
  baskingTemperature = sensors.getTempCByIndex(2);
  currentTime = rtc.getTimeStr();
  startupTime = currentTime;
  if (digitalRead(waterLevel) == 1) {
    waterLow = "true";
  } else {
    waterLow = "false";
  }
  rest.variable("water_temperature", &waterTemperature);
  rest.variable("basking_temperature", &baskingTemperature);
  rest.variable("internal_temperature", &internalTemperature);
  rest.variable("current_time", &currentTime);
  rest.variable("startup_time", &startupTime);
  rest.variable("water_low", &waterLow);
  rest.variable("leaking", &leak);
  rest.variable("amps", &amps);
  rest.variable("kWh", &kWh);

  // Functions to be exposed probably won't need these.
  rest.function("morning", morning);
  rest.function("night", night);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("000001");
  rest.set_name("tank");

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
    // If there is a network connection call out to the NTP server and set the RTC
  } else {
    timeClient.update();
    delay(500);
    timeClient.forceUpdate();
    Serial.println(timeClient.getFormattedTime());

    // Set the RTC
    setTime();
  }

  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  // Start watchdog
  wdt_enable(WDTO_4S);
}

void loop() {

  // Check for a leak every loop
  leakValue = analogRead(leakPin);
  if (leakValue > 100) {
    leak = "true";
    Serial.println("Leak detected. Turning filter off.");
    digitalWrite(outlet7, HIGH);
  } else {
    leak = "false";
  }

  if (loopCount == 30) {
    // Update sensors
    last_power = amps * voltage;
    sensors.requestTemperatures();
    internalTemperature = sensors.getTempCByIndex(0);
    waterTemperature = sensors.getTempCByIndex(1);
    baskingTemperature = sensors.getTempCByIndex(2);

    amps = emon1.calcIrms(1480);

    // Calculate the Kilowatt hours used in the last 24 hours
    // Reset the kWh value to 0 at Midnight ET
    if (currentHour == 5 && currentMinute == 0) {
      yesterdays_total_kWh = kWh;
      kWh = 0;
    }
    new_time = rtc.getUnixTime(rtc.getTime());
    kWh = kWh + ((((amps * voltage) + last_power) / 2) * (new_time - start_time_unix) / 3600000);
    start_time_unix = new_time;

    // Output sensors to serial
    Serial.print("Internal Temp: ");
    Serial.println(internalTemperature);
    Serial.print("Water Temp: ");
    Serial.println(waterTemperature);
    Serial.print("Basking Temp: ");
    Serial.println(baskingTemperature);
    Serial.print("Water low?: ");
    Serial.println(waterLow);
    Serial.print("Amps in use: ");
    Serial.println(amps);
    Serial.print("kWh used: ");
    Serial.println(kWh);

    if (digitalRead(waterLevel) == 1) {
      waterLow = "true";
    } else {
      waterLow = "false";
    }

    currentTime = rtc.getTimeStr();
    Serial.print("Current time: ");
    Serial.println(currentTime);
    currentHour = rtc.getTime().hour;
    currentMinute = rtc.getTime().min;

    // If it is after 10pm and before 9am EST
    if ((currentHour >= 3 && currentHour < 14)) {
      if (!nightTime) {
        night("");
      }
    } else {
      if (!dayTime) {
        morning("");
      }
    }

    // reset loop counter
    loopCount = 0;
  } else {
    delay(1000);
    loopCount = loopCount + 1;
  }

  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);

  wdt_reset();
}

void setupPins() {

  // Setup output pins
  for (int pin = 0; pin < outputPinCount; pin++) {
    pinMode(outletPins[pin], OUTPUT);
    delay(500);
  }
  // Setup input pins
  for (int pin = 0; pin < inputPinCount; pin++) {
    pinMode(inputPins[pin], INPUT);
    delay(500);
  }

  // Turn off all of the optional pins
  digitalWrite(outlet1, HIGH);
  delay(200);
  digitalWrite(outlet2, HIGH);
  delay(200);
  digitalWrite(outlet3, HIGH);
  delay(200);
  //  THE BELOW ARE ALWAYS ON
  //  Heat bulb
  //  digitalWrite(outlet4, HIGH);
  //  Water heaters
  //  digitalWrite(outlet5, HIGH);
  //  digitalWrite(outlet6, HIGH);
  //  Filter
  // Don't turn on the filter if there is a leak at startup
  if (leak == "true") {
    Serial.println("Leak detected. Leaving filter off.");
    digitalWrite(outlet7, HIGH);
  }
  digitalWrite(outlet8, HIGH);

}

// Funtion to call in the morning
int morning(String command) {
  dayTime = true;
  nightTime = false;
  Serial.println("Running day command.");
  digitalWrite(outlet1, LOW);
  delay(500);
  digitalWrite(outlet2, LOW);
  delay(500);
  digitalWrite(outlet3, HIGH);
  return 1;
}

// Function to call at night
int night(String command) {
  nightTime = true;
  dayTime = false;
  Serial.println("Running night command.");
  digitalWrite(outlet3, LOW);
  delay(500);
  digitalWrite(outlet1, HIGH);
  delay(500);
  digitalWrite(outlet2, HIGH);
  return 1;
}

int setTime() {
  timeClient.update();

  Serial.println(timeClient.getFormattedDate());
  int day = timeClient.getDay();
  int hour = timeClient.getHours();
  int minute = timeClient.getMinutes();
  int second = timeClient.getSeconds();
  long dayOfMonth = timeClient.getDate();
  long month = timeClient.getMonth();
  long theYear = timeClient.getYear();
  Serial.println("Time server info.");
  Serial.print("Day of week: ");
  Serial.println(day);
  Serial.print("Hour of day: ");
  Serial.println(hour);
  Serial.print("Minute of hour: ");
  Serial.println(minute);
  Serial.print("Second of minute: ");
  Serial.println(second);
  Serial.print("Day of month: ");
  Serial.println(dayOfMonth + 1);
  Serial.print("Month of year: ");
  Serial.println(month + 1);
  Serial.print("Year: ");
  Serial.println(theYear);

  if (theYear != 2014) {

    if (day == 7) {
      rtc.setDOW(day - 7);
    } else {
      rtc.setDOW(day);
    }
    rtc.setTime(hour, minute, second);
    rtc.setDate(month + 1, dayOfMonth, theYear);
  } else {
    Serial.println("Failed to get time from NTP. Not setting time.");
  }

}
