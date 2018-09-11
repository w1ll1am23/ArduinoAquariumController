#include <ArduinoJson.h>
#include <Timezone.h>
#include <TimeLib.h>
#include "EmonLib.h"
#include <SPI.h>
#include <Ethernet.h>
#include <avr/wdt.h>
#include <NTPClient.h>
#include <EthernetUdp.h>
#include <DS3231.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>

#include "config.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
EnergyMonitor emon1;

// Ethernet UDP
EthernetUDP ntpUDP;

// Enter a MAC address for your controller below.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x12 };

// IP address in case DHCP fails
IPAddress ip(192, 168, 5, 60);

// Gateway adress
IPAddress gateway(192, 168, 5, 1);

NTPClient timeClient(ntpUDP, "192.168.5.1", 0, 60000);

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

// Setup timezones
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
Timezone usEastern(usEDT, usEST);

const char* mqtt_broker = MQTT_BROKER;
const char* mqtt_clientId = MQTT_CLIENTID;
const char* mqtt_username = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;

// MQTT topic setup
String availabilityBase = MQTT_CLIENTID;
String availabilitySuffix = "/availability";
String availabilityTopicStr = availabilityBase + availabilitySuffix;
const char* availabilityTopic = availabilityTopicStr.c_str();
const char* birthMessage = "online";
const char* lwtMessage = "offline";
const char* outlet1_command_topic = MQTT_OUTLET1_COMMAND_TOPIC;
const char* outlet2_command_topic = MQTT_OUTLET2_COMMAND_TOPIC;
const char* outlet3_command_topic = MQTT_OUTLET3_COMMAND_TOPIC;
const char* outlet4_command_topic = MQTT_OUTLET4_COMMAND_TOPIC;
const char* outlet5_command_topic = MQTT_OUTLET5_COMMAND_TOPIC;
const char* outlet6_command_topic = MQTT_OUTLET6_COMMAND_TOPIC;
const char* outlet7_command_topic = MQTT_OUTLET7_COMMAND_TOPIC;
const char* outlet8_command_topic = MQTT_OUTLET8_COMMAND_TOPIC;
const char* all_outlet_command_topics[] = {outlet1_command_topic, outlet2_command_topic, outlet3_command_topic, outlet4_command_topic, outlet5_command_topic, outlet6_command_topic, outlet7_command_topic, outlet8_command_topic};
const char* outlet1_state_topic = MQTT_OUTLET1_STATE_TOPIC;
const char* outlet2_state_topic = MQTT_OUTLET2_STATE_TOPIC;
const char* outlet3_state_topic = MQTT_OUTLET3_STATE_TOPIC;
const char* outlet4_state_topic = MQTT_OUTLET4_STATE_TOPIC;
const char* outlet5_state_topic = MQTT_OUTLET5_STATE_TOPIC;
const char* outlet6_state_topic = MQTT_OUTLET6_STATE_TOPIC;
const char* outlet7_state_topic = MQTT_OUTLET7_STATE_TOPIC;
const char* outlet8_state_topic = MQTT_OUTLET8_STATE_TOPIC;
const char* all_outlet_state_topics[] = {outlet1_state_topic, outlet2_state_topic, outlet3_state_topic, outlet4_state_topic, outlet5_state_topic, outlet6_state_topic, outlet7_state_topic, outlet8_state_topic};
const char* sensors_topic = MQTT_SENSORS_TOPIC;

EthernetClient ethClient;
PubSubClient client(ethClient);


const int OUTLET_PINS[] = {OUTLET1, OUTLET2, OUTLET3, OUTLET4, OUTLET5, OUTLET6, OUTLET7, OUTLET8};

// Used for detecting time and runnning time based commands.
int currentHour;
int currentMinute;

float waterTemperature;
float baskingTemperature;
float internalTemperature;
String startupTime;
String currentTime;
String waterLow;
String leak;
String errorMessage;
int leakValue;
float amps;
float kWh;
float yesterdays_total_kWh;

int loopCount = 0;
float waterTemperatureTemp;
float baskingTemperatureTemp;
float internalTemperatureTemp;

bool nightTime = false;
bool dayTime = false;
bool dst = false;

long start_time_unix;
long new_time;
double last_power;

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
  leakValue = analogRead(LEAK_PIN);
  if (leakValue > 100) {
    leak = "LEAKING";
  } else {
    leak = "NOT LEAKING";
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
  if ((currentHour >= NIGHT_START_HOUR || currentHour < DAY_START_HOUR)) {
    if (!nightTime) {
      night();
    }
  } else {
    if (!dayTime) {
      morning();
    }
  }

  // Init variables
  kWh = 0.0;
  yesterdays_total_kWh = 0.0;
  setTemperatures();
  currentTime = rtc.getTimeStr();
  startupTime = start_time_unix;
  if (digitalRead(WATER_LEVEL) == 1) {
    waterLow = "LOW";
  } else {
    waterLow = "NOT LOW";
  }

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

    if (usEastern.utcIsDST(timeClient.getEpochTime())) {
      Serial.println("It is DST");
      dst = true;
      timeClient.setTimeOffset(SAVINGS_OFFSET);
    } else {
      dst = false;
      timeClient.setTimeOffset(STANDARD_OFFSET);
    }

    // Set the RTC
    setTime();
  }

  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  // Start watchdog
  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);
}

void reconnect() {
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect(mqtt_clientId, mqtt_username, mqtt_password, availabilityTopic, 0, true, lwtMessage)) {
    Serial.println("Connected!");

    // Publish the birth message on connect/reconnect
    publish_birth_message();

    // Subscribe to the action topics to listen for action messages
    for (int pin = 0; pin < OUTPUT_PIN_COUNT; pin++) {
      Serial.print("Subscribing to: ");
      Serial.println(all_outlet_command_topics[pin]);
      client.subscribe(all_outlet_command_topics[pin]);
      delay(100);
    }

    publish_all_statuses();

  }
  else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    delay(5000);
  }
}

void publish_all_statuses() {
  for (int pin = 0; pin < OUTPUT_PIN_COUNT; pin++) {
    if (digitalRead(OUTLET_PINS[pin]) == LOW) {
      client.publish(all_outlet_state_topics[pin], "ON", true);
    } else {
      client.publish(all_outlet_state_topics[pin], "OFF", true);
    }
  }
  publishSensors();
}

void publishSensors() {
  StaticJsonBuffer<512> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();

  JSONencoder["water_level"] = waterLow;
  JSONencoder["filter_leak"] = leak;
  JSONencoder["water_temp"] = (String)waterTemperature;
  JSONencoder["internal_temp"] = (String)internalTemperature;
  JSONencoder["basking_temp"] = (String)baskingTemperature;
  JSONencoder["amps"] = (String)amps;
  JSONencoder["kwh"] = (String)kWh;

  char JSONmessageBuffer[512];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  if (client.publish(sensors_topic, JSONmessageBuffer) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

  client.loop();
}

void publish_birth_message() {
  // Publish the birthMessage
  Serial.print("Publishing birth message \"");
  Serial.print(birthMessage);
  Serial.print("\" to ");
  Serial.print(availabilityTopic);
  Serial.println("...");
  client.publish(availabilityTopic, birthMessage, true);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();

  String topicToProcess = topic;
  payload[length] = '\0';
  String payloadToProcess = (char*)payload;
  triggerAction(topicToProcess, payloadToProcess);
}

void triggerAction(String requestedTopic, String requestedAction) {
  Serial.println("MQTT request received.");
  Serial.print("Topic: ");
  Serial.println(requestedTopic);
  Serial.print("Action: ");
  Serial.println(requestedAction);
  for (int pin = 0; pin < OUTPUT_PIN_COUNT; pin++) {
    if (strcmp(all_outlet_command_topics[pin], requestedTopic.c_str()) == 0) {
      if (requestedAction == "ON") {
        Serial.println("Turning on");
        digitalWrite(OUTLET_PINS[pin], LOW);
        client.publish(all_outlet_state_topics[pin], "ON", true);
      } else if (requestedAction == "OFF") {
        Serial.println("Turning off");
        digitalWrite(OUTLET_PINS[pin], HIGH);
        client.publish(all_outlet_state_topics[pin], "OFF", true);
      } else {
        Serial.println("Invalid action requested.");
      }
    }
  }

}


void loop() {

  if (!client.connected()) {
    reconnect();
  }

  if (loopCount == 30) {
    leakValue = analogRead(LEAK_PIN);
    if (leakValue > 150) {
      leak = "LEAKING";
      if (digitalRead(OUTLET7) == LOW) {
        Serial.println("Leak detected. Turning filter off.");
        digitalWrite(OUTLET7, HIGH);
        publish_all_statuses();
      }
    } else {
      leak = "NOT LEAKING";
    }
    
    // Update sensors
    last_power = amps * VOLTAGE;
    setTemperatures();
    amps = emon1.calcIrms(1480) - 0.10;
    if (amps < 0.3) {
      amps = 0.0;
    }

    // Calculate the Kilowatt hours used in the last 24 hours
    // Reset the kWh value to 0 at Midnight ET
    if (currentHour == 0 && currentMinute == 0) {
      yesterdays_total_kWh = kWh;
      kWh = 0;
    }
    new_time = rtc.getUnixTime(rtc.getTime());
    kWh = kWh + ((((amps * VOLTAGE) + last_power) / 2) * (new_time - start_time_unix) / 3600000);
    // This should fix any strange values that come in at startup
    if (kWh < 0 || kWh > 20) {
      kWh = 0;
    }
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

    if (digitalRead(WATER_LEVEL) == 1) {
      waterLow = "LOW";
    } else {
      waterLow = "NOT LOW";
    }

    currentTime = rtc.getTimeStr();
    Serial.print("Current time: ");
    Serial.println(currentTime);
    currentHour = rtc.getTime().hour;
    currentMinute = rtc.getTime().min;

    if (usEastern.locIsDST(timeClient.getEpochTime())) {
      Serial.println("It is DST");
      timeClient.setTimeOffset(SAVINGS_OFFSET);
      if (dst == false) {
        dst = true;
        setTime();
      }
    } else {
      timeClient.setTimeOffset(STANDARD_OFFSET);
      if (dst == true) {
        dst = false;
        setTime();
      }
    }

    // If it is after 10pm and before 9am EST
    if ((currentHour >= NIGHT_START_HOUR || currentHour < DAY_START_HOUR)) {
      if (!nightTime || digitalRead(OUTLET3) == HIGH) {
        night();
      }
    } else {
      if (!dayTime || digitalRead(OUTLET1) == HIGH) {
        morning();
      }
    }

    // reset loop counter
    loopCount = 0;
    publish_all_statuses();
    client.loop();
  } else {
    delay(1000);
    loopCount = loopCount + 1;
    client.loop();
  }

}

void setTemperatures() {
  sensors.requestTemperatures();
  internalTemperatureTemp = sensors.getTempCByIndex(INTERNAL_TEMP_SENSOR_LOCATION);
  waterTemperatureTemp = sensors.getTempCByIndex(WATER_TEMP_SENSOR_LOCATION);
  baskingTemperatureTemp = sensors.getTempCByIndex(BASKING_TEMP_SENSOR_LOCATION);
  // If the temperatures are way off for some reason
  // don't set the REST variables. Set the error message.
  if (internalTemperatureTemp > 0 && internalTemperatureTemp < 100) {
    internalTemperature = internalTemperatureTemp;
  } else {
    errorMessage = "Internal temperature read error";
  }
  if (waterTemperatureTemp > 0 && waterTemperatureTemp < 100) {
    waterTemperature = waterTemperatureTemp;
  } else {
    errorMessage = "Water temperature read error";
  }
  if (baskingTemperatureTemp > 0 && baskingTemperatureTemp < 100) {
    baskingTemperature = baskingTemperatureTemp;
  } else {
    errorMessage = "Basking temperature read error";
  }
}

void setupPins() {

  // Setup output pins
  for (int pin = 0; pin < OUTPUT_PIN_COUNT; pin++) {
    pinMode(OUTLET_PINS[pin], OUTPUT);
    delay(500);
  }
  // Setup input pins
  for (int pin = 0; pin < INPUT_PIN_COUNT; pin++) {
    pinMode(WATER_LEVEL, INPUT);
    delay(500);
  }

  // Turn off all of the optional pins
  digitalWrite(OUTLET1, HIGH);
  delay(200);
  digitalWrite(OUTLET2, HIGH);
  delay(200);
  digitalWrite(OUTLET3, HIGH);
  delay(200);

  //  THE BELOW ARE ALWAYS ON
  //  Heat bulb
  //  digitalWrite(OUTLET4, HIGH);
  //  Water heaters
  //  digitalWrite(OUTLET5, HIGH);
  //  Bubbler
  //  digitalWrite(OUTLET6, HIGH);
  //  WiFi router
  //  digitalWrite(OUTLET8, HIGH);

  //  Filter
  // Don't turn on the filter if there is a leak at startup
  if (leak == "LEAKING") {
    Serial.println("Leak detected. Leaving filter off.");
    digitalWrite(OUTLET7, HIGH);
  }

}

// Funtion to call in the morning
int morning() {
  dayTime = true;
  nightTime = false;
  Serial.println("Running day command.");
  digitalWrite(OUTLET1, LOW);
  delay(500);
  digitalWrite(OUTLET2, LOW);
  delay(500);
  digitalWrite(OUTLET3, HIGH);
  return 1;
}

// Function to call at night
int night() {
  nightTime = true;
  dayTime = false;
  Serial.println("Running night command.");
  digitalWrite(OUTLET3, LOW);
  delay(500);
  digitalWrite(OUTLET1, HIGH);
  delay(500);
  digitalWrite(OUTLET2, HIGH);
  return 1;
}

int setTime() {
  timeClient.update();
  delay(500);
  timeClient.forceUpdate();

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
