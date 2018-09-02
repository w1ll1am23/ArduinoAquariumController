// MQTT Parameters

#define MQTT_BROKER "MQTT_BROKER_IP"
#define MQTT_CLIENTID "turtleTank"
#define MQTT_USERNAME "MQTT_USERNAME"
#define MQTT_PASSWORD "MQTT_PASSWORD"
#define MQTT_OUTLET1_COMMAND_TOPIC "turtleTank/outlet/1/action"
#define MQTT_OUTLET2_COMMAND_TOPIC "turtleTank/outlet/2/action"
#define MQTT_OUTLET3_COMMAND_TOPIC "turtleTank/outlet/3/action"
#define MQTT_OUTLET4_COMMAND_TOPIC "turtleTank/outlet/4/action"
#define MQTT_OUTLET5_COMMAND_TOPIC "turtleTank/outlet/5/action"
#define MQTT_OUTLET6_COMMAND_TOPIC "turtleTank/outlet/6/action"
#define MQTT_OUTLET7_COMMAND_TOPIC "turtleTank/outlet/7/action"
#define MQTT_OUTLET8_COMMAND_TOPIC "turtleTank/outlet/8/action"
#define MQTT_OUTLET1_STATE_TOPIC "turtleTank/outlet/1/state"
#define MQTT_OUTLET2_STATE_TOPIC "turtleTank/outlet/2/state"
#define MQTT_OUTLET3_STATE_TOPIC "turtleTank/outlet/3/state"
#define MQTT_OUTLET4_STATE_TOPIC "turtleTank/outlet/4/state"
#define MQTT_OUTLET5_STATE_TOPIC "turtleTank/outlet/5/state"
#define MQTT_OUTLET6_STATE_TOPIC "turtleTank/outlet/6/state"
#define MQTT_OUTLET7_STATE_TOPIC "turtleTank/outlet/7/state"
#define MQTT_OUTLET8_STATE_TOPIC "turtleTank/outlet/8/state"
#define MQTT_SENSORS_TOPIC "turtleTank/sensors"

// PIN setup

#define ETH_CS    10
#define SD_CS  4
#define ONE_WIRE_BUS 12
#define OUTLET1 2
#define OUTLET2 3
#define OUTLET3 11
#define OUTLET4 5
#define OUTLET5 6
#define OUTLET6 7
#define OUTLET7 8
#define OUTLET8 9
#define LEAK_PIN A0
#define OUTPUT_PIN_COUNT 8
#define WATER_LEVEL 13
#define OUTPUT_PIN_COUNT 8
#define INPUT_PIN_COUNT 1

// Time settings (Eastern US)

#define SAVINGS "EDT", Second, Sun, Mar, 2, -240  //UTC - 4 hours
#define SAVINGS_OFFSET -14400
#define STANDARD "EST", First, Sun, Nov, 2, -300  //UTC - 5 hours
#define STANDARD_OFFSET -18000

// Random

#define VOLTAGE 119
#define INTERNAL_TEMP_SENSOR_LOCATION 0
#define WATER_TEMP_SENSOR_LOCATION 1
#define BASKING_TEMP_SENSOR_LOCATION 2
#define NIGHT_START_HOUR 22
#define DAY_START_HOUR 9
