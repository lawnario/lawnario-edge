#include <TelemetryParser.h>

///
//MST - Moisture. Dry - 900. Soaked - 520
const double MIN1 = 527;
const double MAX1 = 894;

Telemetry parseTelemetry(std::string payload)
{
    std::stringstream ss(payload);
    std::string token;
    //ID010950:REPLY:SOIL:PACKET:6:HUM:47.28:TEMP:27.59:MST:881:BAT:866
    int i = 0;

    std::string device_name;
    float moisture;
    float battery;
    int packet;
    while (getline(ss, token, ':')) {
        i++;
        switch(i) {
        case 1:
            device_name = token;
        break;
        case 5:
            packet = std::stoi(token.c_str(), NULL);
        break;
        case 11:
            moisture = strtof(token.c_str(), NULL);
        break;
        case 13:
            battery = strtof(token.c_str(), NULL);
        break;
    }
    }
  
    float perc1 = 100*(MAX1 - moisture)/(MAX1-MIN1);
    Telemetry tel;
    tel.packet = packet;
    tel.device_name = device_name;
    tel.moisture = perc1;
    tel.battery = battery;
    return tel;
}

std::string telemetryToJson(Telemetry telemetry)
{
    std::string payload ("{ \"msgCount\": ");
    payload += std::to_string(telemetry.packet);
    payload += ", \"sensor\": ";
    payload += "\"" + telemetry.device_name + "\"";
    payload += ", \"moist\": ";
    payload += std::to_string((int)std::round(telemetry.moisture));
    payload += ", \"battery\": ";
    payload += std::to_string((int)std::round(telemetry.battery));
    payload += " }";
    return payload;
}