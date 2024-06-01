#pragma once
#include <Arduino.h>
#include "utils.h"

#define NUM_OF_MODES 4
#define HYSTERESIS_PERCENT 0.05
#define SENSORS_UPDATE_INTERVAL 500UL
#define DIPLAY_UPDATE_INTERVAL 1000UL
#define WATER_LEVEL_0_PCT_DISTANCE 30

#define SOIL_MOISTURE_SENSOR_PIN A0
#define RELAY_HEATER_PIN 2
#define RELAY_WATER_PUMP_PIN 3
#define RELAY_AIR_HUMIDIFIER_PIN 4
#define RELAY_DEHUMIDIFIER_PIN 5
#define RELAY_LIGHT_PIN 6
#define TRIG_PIN 7
#define ECHO_PIN 8
#define UP_BUTTON_PIN 9
#define OK_BUTTON_PIN 10
#define DOWN_BUTTON_PIN 11

const GreenhouseModeSettings greenhouseModeSettings[NUM_OF_MODES] =
{
    {
        .name = "Fruits",
        .temperature = 20.2,
        .humidity = 60.3,
        .lightIntensity = 1235,
        .soilMoisture = 50
    },
    {
        .name = "Vegetables",
        .temperature = 22.2,
        .humidity = 66.3,
        .lightIntensity = 923,
        .soilMoisture = 70
    },
    {
        .name = "Plants",
        .temperature = 23.2,
        .humidity = 55.3,
        .lightIntensity = 543,
        .soilMoisture = 50
    },
    {
        .name = "Cacti",
        .temperature = 29.2,
        .humidity = 70.3,
        .lightIntensity = 876,
        .soilMoisture = 30
    }
};
