#pragma once
#include <Arduino.h>

#define RELAY_SET(pin, state) (digitalWrite(pin, !state))

typedef struct
{
    String name;
    float temperature;
    float humidity;
    float lightIntensity;
    float soilMoisture;
} GreenhouseModeSettings;

typedef struct
{
    float temperature;    // °C
    float humidity;       // %
    float lightIntensity; // lx
    float soilMoisture;   // %
    float pressure;       // hPa
    float waterLevel;     // %
} SensorReadings;
