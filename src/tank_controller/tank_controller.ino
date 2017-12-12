#include <TimeLib.h>

#include <SPI.h>
#include <Ethernet.h>
#include <aREST.h>
#include <avr/wdt.h>
#include <NTPClient.h>
#include <EthernetUdp.h>
#include <DS3231.h>

#define ETH_CS    10
#define SD_CS  4

// Ethernet UDP
EthernetUDP ntpUDP;

// NTP client
NTPClient timeClient(ntpUDP, "192.168.5.1", 3600, 60000);

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x12 };

// IP address in case DHCP fails
IPAddress ip(192,168,2,2);

// Gateway adress
IPAddress gateway(192,168,2,1);

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
float uvb;
String current_time;

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

int inputPins[0];
int inputPinCount = 0;

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

  setupPins();
  rtc.begin();


  // Init variables and expose them to REST API
  water_temperature = 0.0;
  basking_temperature = 0.0;
  internal_temperature = 0.0;
  uvb = 0.0;
  current_time = rtc.getTimeStr();
  
  rest.variable("water_temperature",&water_temperature);
  rest.variable("basking_temperature",&basking_temperature);
  rest.variable("internal_temperature", &internal_temperature);
  rest.variable("uvb",&uvb);
  rest.variable("current_time", &current_time);

  // Functions to be exposed
  rest.function("morning",morning);
  rest.function("night",night);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("000001");
  rest.set_name("tank");

  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    Ethernet.begin(mac, ip);
  }
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  timeClient.update();
  delay(500);
  timeClient.forceUpdate();
  Serial.println(timeClient.getFormattedTime());

  setTime();

  // Start watchdog
  wdt_enable(WDTO_4S);
}

void loop() {

  // listen for incoming clients
  EthernetClient client = server.available();
  rest.handle(client);
  wdt_reset();

}

void setupPins() {
  for (int pin = 0; pin < outputPinCount; pin++) {
    Serial.print("Setting up pin: ");
    Serial.println(pin);
    pinMode(outletPins[pin], OUTPUT);
    delay(100);
    digitalWrite(pin, LOW);
    delay(100);
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
  }
  for (int pin = 0; pin < inputPinCount; pin++) {
    pinMode(inputPins[pin], INPUT);
  }
}

double setWaterTemperature() {
  return 0.0;
}

double setBaskingTemperature() {
  return 0.0;
}

// Funtion to call in the morning
int morning(String command) {
  digitalWrite(outlet1, HIGH);
  delay(100);
  digitalWrite(outlet2, HIGH);
  delay(100);
  digitalWrite(outlet3, LOW);
  return 1;
}

// Function to call at night
int night(String command) {
  digitalWrite(outlet3, HIGH);
  delay(100);
  digitalWrite(outlet1, LOW);
  delay(100);
  digitalWrite(outlet2, LOW);
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
  long the_year = timeClient.getYear();
  Serial.println(day);
  Serial.println(hour);
  Serial.println(minute);
  Serial.println(second);
  Serial.println(dayOfMonth);
  Serial.println(month);
  Serial.println(the_year);

  if (day == 7) {
    rtc.setDOW(day - 7);
  } else {
    rtc.setDOW(day);
  }
  rtc.setTime(hour, minute, second);
  rtc.setDate(month + 1, dayOfMonth, the_year);   // Set the date to January 1st, 2014

  // Send Day-of-Week
  Serial.print(rtc.getDOWStr());
  Serial.print(" ");
  
  // Send date
  Serial.print(rtc.getDateStr());

}
