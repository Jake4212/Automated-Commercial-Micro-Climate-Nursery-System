/**
 * @file main.cpp
 * @brief Scenario 2: Automated Commercial Micro-Climate Nursery System
 * @details Complete ESP32 firmware implementing Autonomous Climate Control, 
 *          Manual Override Mode, and Hardware Safety Fault Recovery routines.
 * @author Embedded Systems Engineer
 * @date 2026
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ============================================================================
// PIN DEFINITIONS & HARDWARE MAPPING
// ============================================================================
#define DHTPIN 15            /**< GPIO pin for DHT22 Data line */
#define DHTTYPE DHT22        /**< DHT Sensor Type */
#define SERVO_PIN 18         /**< GPIO pin for Vent Servo PWM */
#define LDR_PIN 34           /**< ADC1 Channel pin for 2-Pin LDR Voltage Divider */

// Output Pin Assignments (LEDs)
#define LED_RED_1 26         /**< Red LED 1 (Safety Alarm) */
#define LED_RED_2 25         /**< Red LED 2 (Safety Alarm) */
#define LED_RED_3 33         /**< Red LED 3 (Safety Alarm) */
#define LED_GRN_1 32         /**< Green LED 1 (Grow Light Subsystem) */
#define LED_GRN_2 12         /**< Green LED 2 (Grow Light Subsystem) */
#define LED_GRN_3 14         /**< Green LED 3 (Grow Light Subsystem) */

// Input Pin Assignments (Push Buttons)
#define BTN_OVERRIDE_PIN 4   /**< Button 1: Toggle Manual Override */
#define BTN_RESET_PIN 16     /**< Button 2: Fault Reset / Secondary Control */

// OLED Display Configuration
#define SCREEN_WIDTH 128     /**< OLED display width in pixels */
#define SCREEN_HEIGHT 64     /**< OLED display height in pixels */
#define OLED_RESET -1        /**< Reset pin # (or -1 if sharing Arduino reset pin) */
#define SCREEN_ADDRESS 0x3C  /**< I2C address for 128x64 OLED display */

// ============================================================================
// OPERATIONAL CONSTANTS & THRESHOLDS
// ============================================================================
#define TEMP_THRESHOLD 28.0  /**< Temp (°C) threshold to open vent window */
#define LIGHT_THRESHOLD 1500 /**< LDR ADC threshold (0-4095) for Grow Lights */
#define VENT_CLOSED_ANGLE 0  /**< Servo position for closed vent window */
#define VENT_OPEN_ANGLE 90   /**< Servo position for open vent window */

// ============================================================================
// SYSTEM STATES & GLOBAL OBJECTS
// ============================================================================
enum SystemState {
    MODE_AUTONOMOUS,         /**< Normal operational mode driven by sensor feedback */
    MODE_MANUAL_OVERRIDE,    /**< Manual override mode locking vents open */
    MODE_SAFETY_FAULT        /**< Sensor fault or corrupt data fallback mode */
};

SystemState currentState = MODE_AUTONOMOUS;

// Hardware Drivers
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHTPIN, DHTTYPE);
Servo ventServo;

// Environment Variables
float temperature = 0.0;
float humidity = 0.0;
int lightLevel = 0;

// State Flags
bool isVentOpen = false;
bool growLightsOn = false;
bool sensorFault = false;

// Button Debounce Management
unsigned long lastOverridePressTime = 0;
unsigned long lastResetPressTime = 0;
const unsigned long DEBOUNCE_DELAY = 250; /**< Debounce threshold in milliseconds */

// Non-Blocking Execution Timers
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 2000; /**< Read sensors every 2 seconds */

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void readSensors();
void updateControlLogic();
void setGrowLights(bool state);
void setAlarmLEDs(bool state);
void updateDisplay();
void printSerialStatus();
void checkInputs();

/**
 * @brief Reads data from physical sensors (DHT22 and 2-pin LDR ADC).
 * @return void
 */
void readSensors() {
    float tempRead = dht.readTemperature();
    float humRead = dht.readHumidity();
    int ldrRead = analogRead(LDR_PIN);

    // Validate DHT sensor readings (Safety Logic)
    if (isnan(tempRead) || isnan(humRead) || tempRead < -40.0 || tempRead > 80.0) {
        sensorFault = true;
        currentState = MODE_SAFETY_FAULT;
        Serial.println(F("[ALERT] DHT Sensor Read Failure Detected!"));
    } else {
        sensorFault = false;
        temperature = tempRead;
        humidity = humRead;
        lightLevel = ldrRead;
    }
}

/**
 * @brief Drives hardware outputs (Servo, LEDs) based on current operational mode.
 * @return void
 */
void updateControlLogic() {
    switch (currentState) {
        case MODE_MANUAL_OVERRIDE:
            // Override Mode: Vents locked open for maintenance, lights off
            ventServo.write(VENT_OPEN_ANGLE);
            isVentOpen = true;
            setAlarmLEDs(false);
            setGrowLights(false);
            growLightsOn = false;
            break;

        case MODE_SAFETY_FAULT:
            // Safety Posture: Open vent to prevent crop overheating, flash red LEDs
            ventServo.write(VENT_OPEN_ANGLE);
            isVentOpen = true;
            setAlarmLEDs(true);
            setGrowLights(false);
            growLightsOn = false;
            break;

        case MODE_AUTONOMOUS:
        default:
            setAlarmLEDs(false);

            // Sunlight coordination logic (LDR Voltage Divider)
            // Turns ON Green Grow Lights when light level drops below threshold
            if (lightLevel < LIGHT_THRESHOLD) {
                setGrowLights(true);
                growLightsOn = true;
            } else {
                setGrowLights(false);
                growLightsOn = false;
            }

            // Temperature threshold vent actuation
            if (temperature > TEMP_THRESHOLD) {
                ventServo.write(VENT_OPEN_ANGLE);
                isVentOpen = true;
            } else {
                ventServo.write(VENT_CLOSED_ANGLE);
                isVentOpen = false;
            }
            break;
    }
}

/**
 * @brief Controls the status of the Green Grow Light LEDs (GPIOs 32, 12, 14).
 * @param state True to illuminate, False to extinguish.
 * @return void
 */
void setGrowLights(bool state) {
    uint8_t level = state ? HIGH : LOW;
    digitalWrite(LED_GRN_1, level);
    digitalWrite(LED_GRN_2, level);
    digitalWrite(LED_GRN_3, level);
}

/**
 * @brief Controls the status of the Red Alarm/Fault LEDs (GPIOs 26, 25, 33).
 * @param state True to illuminate, False to extinguish.
 * @return void
 */
void setAlarmLEDs(bool state) {
    uint8_t level = state ? HIGH : LOW;
    digitalWrite(LED_RED_1, level);
    digitalWrite(LED_RED_2, level);
    digitalWrite(LED_RED_3, level);
}

/**
 * @brief Checks for non-blocking push-button presses and Serial inputs.
 * @return void
 */
void checkInputs() {
    unsigned long currentMillis = millis();

    // Button 1: Toggle Manual Override
    if (digitalRead(BTN_OVERRIDE_PIN) == LOW) {
        if (currentMillis - lastOverridePressTime > DEBOUNCE_DELAY) {
            lastOverridePressTime = currentMillis;

            if (currentState == MODE_MANUAL_OVERRIDE) {
                currentState = MODE_AUTONOMOUS;
                Serial.println(F("[MODE] Exited Manual Override -> Autonomous"));
            } else {
                currentState = MODE_MANUAL_OVERRIDE;
                Serial.println(F("[MODE] Entered Manual Override Mode"));
            }
        }
    }

    // Button 2: Fault Reset / Mode Return
    if (digitalRead(BTN_RESET_PIN) == LOW) {
        if (currentMillis - lastResetPressTime > DEBOUNCE_DELAY) {
            lastResetPressTime = currentMillis;

            if (currentState == MODE_SAFETY_FAULT) {
                Serial.println(F("[SYSTEM] Manual Reset Triggered. Retrying sensors..."));
                readSensors();
                if (!sensorFault) {
                    currentState = MODE_AUTONOMOUS;
                }
            }
        }
    }

    // Serial Monitor Control Option ('m' key toggle)
    if (Serial.available() > 0) {
        char ch = Serial.read();
        if (ch == 'm' || ch == 'M') {
            if (currentState == MODE_MANUAL_OVERRIDE) {
                currentState = MODE_AUTONOMOUS;
                Serial.println(F("[MODE] Exited Manual Override via Serial"));
            } else {
                currentState = MODE_MANUAL_OVERRIDE;
                Serial.println(F("[MODE] Entered Manual Override via Serial"));
            }
        }
    }
}

/**
 * @brief Renders active context-aware monitoring information onto the OLED display.
 * @return void
 */
void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Display Header
    display.setCursor(0, 0);
    display.print(F("MODE: "));
    if (currentState == MODE_MANUAL_OVERRIDE) {
        display.println(F("MANUAL OVERRIDE"));
    } else if (currentState == MODE_SAFETY_FAULT) {
        display.println(F("SAFETY FAULT"));
    } else {
        display.println(F("AUTONOMOUS"));
    }
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    if (currentState == MODE_SAFETY_FAULT) {
        display.setCursor(0, 18);
        display.println(F("CRITICAL SENSOR ERROR"));
        display.setCursor(0, 32);
        display.println(F("Fault: DHT22 Unresponsive"));
        display.setCursor(0, 48);
        display.println(F("POSTURE: VENT OPEN"));
    } else {
        display.setCursor(0, 15);
        display.print(F("Temp: "));
        display.print(temperature, 1);
        display.println(F(" C"));

        display.setCursor(0, 27);
        display.print(F("Hum:  "));
        display.print(humidity, 1);
        display.println(F(" %"));

        display.setCursor(0, 39);
        display.print(F("Light ADC: "));
        display.println(lightLevel);

        display.setCursor(0, 51);
        display.print(F("Vent: "));
        display.print(isVentOpen ? F("OPEN") : F("CLOSED"));
        display.print(F(" | Light:"));
        display.println(growLightsOn ? F("ON") : F("OFF"));
    }

    display.display();
}

/**
 * @brief Outputs live status strings over the UART Serial Interface.
 * @return void
 */
void printSerialStatus() {
    Serial.print(F("[STATUS] Mode: "));
    Serial.print(currentState == MODE_MANUAL_OVERRIDE ? "MANUAL" : (currentState == MODE_SAFETY_FAULT ? "FAULT" : "AUTO"));
    Serial.print(F(" | Temp: ")); Serial.print(temperature, 1);
    Serial.print(F("C | Hum: ")); Serial.print(humidity, 1);
    Serial.print(F("% | Light ADC: ")); Serial.print(lightLevel);
    Serial.print(F(" | Vent: ")); Serial.print(isVentOpen ? "OPEN" : "CLOSED");
    Serial.print(F(" | GrowLights: ")); Serial.println(growLightsOn ? "ON" : "OFF");
}

/**
 * @brief Main Initialization Setup routine.
 * @return void
 */
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("Initializing Automated Micro-Climate System..."));

    // Configure Pin Modes (LED Outputs)
    pinMode(LED_RED_1, OUTPUT); digitalWrite(LED_RED_1, LOW);
    pinMode(LED_RED_2, OUTPUT); digitalWrite(LED_RED_2, LOW);
    pinMode(LED_RED_3, OUTPUT); digitalWrite(LED_RED_3, LOW);
    pinMode(LED_GRN_1, OUTPUT); digitalWrite(LED_GRN_1, LOW);
    pinMode(LED_GRN_2, OUTPUT); digitalWrite(LED_GRN_2, LOW);
    pinMode(LED_GRN_3, OUTPUT); digitalWrite(LED_GRN_3, LOW);

    // Configure Pin Modes (Button Inputs with Internal Pull-ups)
    pinMode(BTN_OVERRIDE_PIN, INPUT_PULLUP);
    pinMode(BTN_RESET_PIN, INPUT_PULLUP);

    // Initialize Peripherals
    dht.begin();
    ventServo.attach(SERVO_PIN);
    ventServo.write(VENT_CLOSED_ANGLE);

    // Initialize OLED Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 Allocation Failed!"));
        for (;;); // Stop execution
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(F(" System Initialized"));
    display.display();
    delay(1000);
}

/**
 * @brief Main Continuous Loop (Strictly Non-Blocking).
 * @return void
 */
void loop() {
    unsigned long currentMillis = millis();

    // Poll push buttons & Serial input continuously for instant response
    checkInputs();

    // Scheduled sensor evaluation and UI updating
    if (currentMillis - lastSensorReadTime >= SENSOR_INTERVAL) {
        lastSensorReadTime = currentMillis;

        if (currentState != MODE_MANUAL_OVERRIDE) {
            readSensors();
        }

        updateControlLogic();
        updateDisplay();
        printSerialStatus();
    }
}