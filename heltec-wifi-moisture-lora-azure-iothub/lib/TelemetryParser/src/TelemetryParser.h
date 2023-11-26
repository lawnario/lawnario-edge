#ifndef TELEMETRYPARSER_H
#define TELEMETRYPARSER_H


#include <az_core.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <cmath>

struct Telemetry {
    int packet;
    std::string device_name;
    float moisture;
    float battery;
};

Telemetry parseTelemetry(std::string payload);

std::string telemetryToJson(Telemetry telemetry);

#endif