void sendSensorsReadings()
{
    Serial.print("{");
    Serial.print("\"temperature\":");
    Serial.print(lastReadings.temperature);
    Serial.print(", \"humidity\":");
    Serial.print(lastReadings.humidity);
    Serial.print(", \"lightIntensity\":");
    Serial.print(lastReadings.lightIntensity);
    Serial.print(", \"soilMoisture\":");
    Serial.print(lastReadings.soilMoisture);
    Serial.print(", \"pressure\":");
    Serial.print(lastReadings.pressure);
    Serial.print(", \"waterLevel\":");
    Serial.print(lastReadings.waterLevel);
    Serial.println("}");
}

void showOnDisplay()
{
    static uint8_t currentSensor = 0;
    static unsigned long lastDisplayUpdate = 0;

    if (millis() - lastDisplayUpdate < DIPLAY_UPDATE_INTERVAL)
    {
        return;
    }

    GreenhouseModeSettings settings = greenhouseModeSettings[currentModeNumber];
    lcd.clear();

    if (currentSensor == 0)
    {
        lcd.print("Temperature [C]");
        lcd.setCursor(0, 1);
        lcd.print(String(lastReadings.temperature, 2) + " (" + String(settings.temperature) + ")");
    }
    else if (currentSensor == 1)
    {
        lcd.print("Humidity [%]");
        lcd.setCursor(0, 1);
        lcd.print(String(lastReadings.humidity, 2) + " (" + String(settings.humidity) + ")");
    }
    else if (currentSensor == 2)
    {
        lcd.print("Light [lx]");
        lcd.setCursor(0, 1);
        lcd.print(String(lastReadings.lightIntensity, 2) + " (" + String(settings.lightIntensity) + ")");
    }
    else if (currentSensor == 3)
    {
        lcd.print("Soil moist. [%]");
        lcd.setCursor(0, 1);
        lcd.print(String(lastReadings.soilMoisture, 2) + " (" + String(settings.soilMoisture) + ")");
    }
    else if (currentSensor == 4)
    {
        lcd.print("Pressure [hPa]");
        lcd.setCursor(0, 1);
        lcd.print(String(lastReadings.pressure, 2));
    }
    else if (currentSensor == 5)
    {
        lcd.print("Water level [%]");
        lcd.setCursor(0, 1);
        lcd.print(String(lastReadings.waterLevel, 0));
    }

    currentSensor = (currentSensor + 1) % 6;
    lastDisplayUpdate = millis();
}

void setMode()
{
    bool updateDisplay = true;

    if (!digitalRead(OK_BUTTON_PIN))
    {
        while (true)
        {
            if (updateDisplay)
            {
                lcd.clear();
                lcd.print("Mode:");
                lcd.setCursor(0, 1);
                lcd.print(greenhouseModeSettings[currentModeNumber].name);
                updateDisplay = false;

                delay(50);
                while (!digitalRead(OK_BUTTON_PIN) || !digitalRead(UP_BUTTON_PIN) || !digitalRead(DOWN_BUTTON_PIN)) {};
                delay(50);
            }

            if (!digitalRead(UP_BUTTON_PIN))
            {
                currentModeNumber = (currentModeNumber + 1) % NUM_OF_MODES;
                updateDisplay = true;
            }
            else if (!digitalRead(DOWN_BUTTON_PIN))
            {
                currentModeNumber = (currentModeNumber == 0 ? (NUM_OF_MODES - 1) : currentModeNumber - 1);
                updateDisplay = true;
            }
            else if (!digitalRead(OK_BUTTON_PIN))
            {
                delay(50);
                while (!digitalRead(OK_BUTTON_PIN) || !digitalRead(UP_BUTTON_PIN) || !digitalRead(DOWN_BUTTON_PIN)) {};
                delay(50);
                return;
            }
        }
    }
}

void setup()
{
    Serial.begin(9600);

    pinMode(SOIL_MOISTURE_SENSOR_PIN, INPUT);
    pinMode(RELAY_HEATER_PIN, OUTPUT);
    pinMode(RELAY_WATER_PUMP_PIN, OUTPUT);
    pinMode(RELAY_AIR_HUMIDIFIER_PIN, OUTPUT);
    pinMode(RELAY_DEHUMIDIFIER_PIN, OUTPUT);
    pinMode(RELAY_LIGHT_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(OK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);

    RELAY_SET(RELAY_HEATER_PIN, LOW);
    RELAY_SET(RELAY_WATER_PUMP_PIN, LOW);
    RELAY_SET(RELAY_AIR_HUMIDIFIER_PIN, LOW);
    RELAY_SET(RELAY_DEHUMIDIFIER_PIN, LOW);
    RELAY_SET(RELAY_LIGHT_PIN, LOW);

    while (!bme280.begin())
    {
        Serial.println("BME280 Connection error");
        delay(1000);
    }

    while (!bh1750.begin(BH1750::CONTINUOUS_LOW_RES_MODE))
    {
        Serial.println("BH1750 Connection error");
        delay(1000);
    }

    lcd.init();
    lcd.backlight();

    handleSensors();
    sendSensorsReadings();
    lastSensorsUpdateTime = millis();
}

void loop()
{
    if (millis() - lastSensorsUpdateTime >= SENSORS_UPDATE_INTERVAL)
    {
        handleSensors();
        sendSensorsReadings();
        lastSensorsUpdateTime = millis();
    }

    setMode();
    showOnDisplay();
}