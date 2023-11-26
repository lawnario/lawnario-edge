
#include <unity.h>
#include <TelemetryParser.h>
#include <iostream>
#include <string>

#ifdef ARDUINO
#include <Arduino.h>
#endif

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void parse_moisture() {
    //ID010950:REPLY:SOIL:PACKET:6:HUM:47.28:TEMP:27.59:MST:881:BAT:866

    char *payload = "ID010950:REPLY:SOIL:PACKET:15770:HUM:48.35:TEMP:25.81:MST:538:BAT:821";

    Telemetry tel = parseTelemetry(payload);
    std::string payload_result = telemetryToJson(tel);

    std::string az_result ("{ \"msgCount\": 6, \"sensor\": \"ID010950\", \"moist\": 4, \"battery\": 866 }");

    TEST_ASSERT_TRUE(payload_result.compare(az_result) == 0);
}

void check_string_conversion()
{
    Telemetry tel;
    tel.device_name = "Device1";
    tel.battery = 667;
    tel.moisture = 67.32;
    tel.packet = 15666;
    std::string str1 = "Sensor: " + tel.device_name;
    std::string str2 = "Moisture: " + std::to_string((int)std::round(tel.moisture));
    std::string str3 = "Battery: " + std::to_string((int)std::round(tel.battery));
    
}

#ifdef ARDUINO
void setup() {
  delay(2000); // add 2-sec wait for the board w/o software resetting via
               // Serial.DTR/RTS
#else
int main(int argc, char *argv[]) {
#endif
  UNITY_BEGIN();
  RUN_TEST(parse_moisture);
  RUN_TEST(check_string_conversion);
  UNITY_END();

#ifndef ARDUINO
  return 0;
#endif
}

#ifdef ARDUINO
void loop() {}
#endif

