#include <TimeLib.h>

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

// Ethernet UDP
EthernetUDP ntpUDP;

// NTP client
NTPClient timeClient(ntpUDP, "192.168.5.1", 3600, 60000);

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x12 };

// IP address in case DHCP fails
IPAddress ip(192,168,5,221);

// Gateway adress
IPAddress gateway(192,168,5,1);

// Ethernet server
EthernetServer server(80);

// Create aREST instance
aREST rest = aREST();

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

// Variables to be exposed to the API
float water_temperature;
float basking_temperature;
float internal_temperature;
String start_up_time;
String current_time;
String water_low;
int current_hour;
int current_minute;

int loopCount = 0;

int outlet1 = 2;
int outlet2 = 3;
int outlet3 = 11;
int outlet4 = 5;
int outlet5 = 6;
int outlet6 = 7;
int outlet7 = 8;
int outlet8 = 9;

int outletPins[] = {outlet1, outlet2, outlet3, outlet4, outlet5, outlet6, outlet7, outlet8};
int outputPinCount = 8;

int waterLevel = 13;

int inputPins[] = {waterLevel};
int inputPinCount = 1;

bool nightTime = false;
bool dayTime = false;

void setup(void)
{
  // Start Serial
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(ETH_CS,OUTPUT);
  pinMode(SD_CS,OUTPUT);
  delay(100);
  digitalWrite(ETH_CS,LOW); // Select the Ethernet Module.
  delay(100);
  digitalWrite(SD_CS,HIGH); // De-Select the internal SD Card

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
  } else {
    timeClient.update();
    delay(500);
    timeClient.forceUpdate();
    Serial.print("Formatted date/time: ");
    Serial.println(timeClient.getFormattedTime());
    
    setTime();
  }
  
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  // Start watchdog
  wdt_enable(WDTO_4S);
}

void loop() {

  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);

  // Update sensors
  sensors.requestTemperatures();
  internal_temperature = sensors.getTempCByIndex(0);
  water_temperature = sensors.getTempCByIndex(1);
  basking_temperature = sensors.getTempCByIndex(2);  

  if (loopCount == 30) {
    Serial.print("Internal Temp: ");
    Serial.println(internal_temperature);
    Serial.print("Water Temp: ");
    Serial.println(water_temperature);
    Serial.print("Basking Temp: ");
    Serial.println(basking_temperature);
    Serial.print("Water low?: ");
    Serial.println(water_low);
    

    if (digitalRead(waterLevel) == 1) {
      water_low = "true";
    } else {
      water_low = "false";
    }

    current_time = rtc.getTimeStr();
    Serial.print("Current time: ");
    Serial.println(current_time);
    current_hour = rtc.getTime().hour;
    current_minute = rtc.getTime().min;
    if ((current_hour >= 22 || current_hour < 9)) {
      if (!nightTime) {
        night("");
      }
    } else {
      if (!dayTime) {
        morning("");
      }
    }

    
    loopCount = 0;
  } else {
    loopCount = loopCount + 1;
  }
  
  wdt_reset();
}

void setupPins() {
  for (int pin = 0; pin < outputPinCount; pin++) {
    pinMode(outletPins[pin], OUTPUT);
    delay(500);
  }
  for (int pin = 0; pin < inputPinCount; pin++) {
    pinMode(inputPins[pin], INPUT);
    delay(500);
  }
  digitalWrite(outlet1, HIGH);
  digitalWrite(outlet2, HIGH);
  digitalWrite(outlet3, HIGH);
//  THE BELOW ARE ALWAYS ON
//  Heat bulb
//  digitalWrite(outlet4, HIGH);
//  Water heaters
//  digitalWrite(outlet5, HIGH);
//  digitalWrite(outlet6, HIGH);
//  Filter
//  digitalWrite(outlet7, HIGH);
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
  // Not sure why this is off by 1 hour...
  int hour = timeClient.getHours() - 1;
  int minute = timeClient.getMinutes();
  int second = timeClient.getSeconds();
  long dayOfMonth = timeClient.getDate();
  long month = timeClient.getMonth();
  long the_year = timeClient.getYear();
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
  Serial.println(dayOfMonth);
  Serial.print("Month of year: ");
  Serial.println(month);
  Serial.print("Year: ");
  Serial.println(the_year);

  if (day == 7) {
    rtc.setDOW(day - 7);
  } else {
    rtc.setDOW(day);
  }
  rtc.setTime(hour, minute, second);
  rtc.setDate(month + 1, dayOfMonth, the_year);
  
  // Send date
  Serial.println(rtc.getDateStr());

}
