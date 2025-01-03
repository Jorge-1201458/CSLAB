#include <ESP32Servo.h>
#include <WiFi.h>
#include "DHT.h"
#include <WiFiClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// WiFi credentials
const char *ssid = "NOS-17A6";
const char *password = "WPSN47JG";

// Server details
const char *server_ip = "192.168.1.5";
const int server_port = 8080;

// DHT sensor configuration
#define DHTPIN 18
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// LDR pin
#define LDRPIN 34

// Servo configuration
Servo blindsServo;
#define SERVOPIN 19

// LED pins for heater and lights
#define HEATERPIN 4
#define LIGHTPIN 5

// Task periods (in milliseconds)
#define READ_DATA_PERIOD 20
#define SEND_DATA_PERIOD 30

// Task priorities (higher number = higher priority)
#define READ_DATA_PRIORITY 2
#define SEND_DATA_PRIORITY 1

// Struct for sensor data
struct SensorData {
    float temperature;
    int light_level;
};

// Global variables
WiFiClient client;
SensorData sensorData;
SemaphoreHandle_t sensorMutex;

// Inline assembly function to turn on light
void lightOnAssembly() {
  // ESP32 uses GPIO registers for pin control
  // GPIO5 is the LIGHTPIN in this case
  asm volatile (
    "movi a2, 0x3FF44000\n"   // Load GPIO output register base address
    "movi a3, (1 << 5)\n"     // Create bitmask for GPIO5 (1 << LIGHTPIN)
    "l32i a4, a2, 0x4\n"      // Load current GPIO output register value
    "or a4, a4, a3\n"         // Set the bit for GPIO5 high
    "s32i a4, a2, 0x4\n"      // Store back to GPIO output register
    ::: "a2", "a3", "a4", "memory"
  );
}

// Inline assembly function to turn off light
void lightOffAssembly() {
  asm volatile (
    "movi a2, 0x3FF44000\n"   // Load GPIO output register base address
    "movi a3, ~(1 << 5)\n"    // Create inverted bitmask for GPIO5
    "l32i a4, a2, 0x4\n"      // Load current GPIO output register value
    "and a4, a4, a3\n"        // Clear the bit for GPIO5
    "s32i a4, a2, 0x4\n"      // Store back to GPIO output register
    ::: "a2", "a3", "a4", "memory"
  );
}


// Function to handle commands from server
void handleCommand(String command) {
    if (command == "HEATER_ON\n") {
        digitalWrite(HEATERPIN, HIGH);
    } else if (command == "HEATER_OFF\n") {
        digitalWrite(HEATERPIN, LOW);
    } else if (command == "LIGHT_ON\n") {
        //digitalWrite(LIGHTPIN, HIGH);
        lightOnAssembly();
    } else if (command == "LIGHT_OFF\n") {
        //digitalWrite(LIGHTPIN, LOW);
        lightOffAssembly();
    } else if (command == "BLINDS_OPEN\n") {
        for (int pos = 0; pos <= 180; pos++) {
            blindsServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    } else if (command == "BLINDS_CLOSE\n") {
        for (int pos = 180; pos >= 0; pos--) {
            blindsServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
}

// Read Data Sensors Task (Periodic, Priority 2)
void readDataTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(READ_DATA_PERIOD);

    while (1) {
        // Record start time
        unsigned long startTime = micros();

        // Read sensors
        float temp = dht.readTemperature();
        int light = analogRead(LDRPIN);

        if (!isnan(temp)) {
            // Update shared data with mutex protection
            xSemaphoreTake(sensorMutex, portMAX_DELAY);
            sensorData.temperature = temp;
            sensorData.light_level = light;
            xSemaphoreGive(sensorMutex);
            
            Serial.printf("Read - Temperature: %.1f°C, Light: %d\n", temp, light);
        }

        // Calculate execution time
        unsigned long executionTime = micros() - startTime;
        Serial.printf("Read Task Execution Time: %lu microseconds\n", executionTime);

        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// Send Data Sensors Task (Periodic, Priority 1)
void sendDataTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(SEND_DATA_PERIOD);

    while (1) {
        // Record start time
        unsigned long startTime = micros();

        if (client.connected()) {
            // Get sensor data with mutex protection
            xSemaphoreTake(sensorMutex, portMAX_DELAY);
            float temp = sensorData.temperature;
            int light = sensorData.light_level;
            xSemaphoreGive(sensorMutex);

            // Send data to server
            String data = String(temp) + "," + String(light) + "\n";
            client.print(data);
            Serial.printf("Sent: %s", data.c_str());

            // Check for and handle incoming commands
            while (client.available()) {
                String command = client.readStringUntil('\n') + "\n";
                handleCommand(command);
            }
        } else {
            // Attempt to reconnect
            Serial.println("Reconnecting to server...");
            client.connect(server_ip, server_port);
        }

        // Calculate execution time
        unsigned long executionTime = micros() - startTime;
        Serial.printf("Send Task Execution Time: %lu microseconds\n", executionTime);

        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void setup() {
    Serial.begin(9600);

    // Initialize mutex
    sensorMutex = xSemaphoreCreateMutex();

    // Initialize hardware
    analogSetAttenuation(ADC_11db);
    dht.begin();
    blindsServo.attach(SERVOPIN);
    pinMode(HEATERPIN, OUTPUT);
    pinMode(LIGHTPIN, OUTPUT);

    // Set default states
    digitalWrite(HEATERPIN, LOW);
    digitalWrite(LIGHTPIN, LOW);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Connect to server
    while (!client.connect(server_ip, server_port)) {
        Serial.println("Connection to server failed. Retrying...");
        delay(5000);
    }
    Serial.println("Connected to server");

    // Create tasks
    xTaskCreatePinnedToCore(
        readDataTask,                // Task function
        "ReadDataTask",             // Name
        4096,                       // Stack size
        NULL,                       // Parameters
        READ_DATA_PRIORITY,         // Priority
        NULL,                       // Task handle
        1                          // Core ID (1 = secondary core)
    );

    xTaskCreatePinnedToCore(
        sendDataTask,               // Task function
        "SendDataTask",            // Name
        4096,                      // Stack size
        NULL,                      // Parameters
        SEND_DATA_PRIORITY,        // Priority
        NULL,                      // Task handle
        1                         // Core ID (1 = secondary core)
    );
}

void loop() {
    // Empty loop - tasks handle everything
    vTaskDelete(NULL);
}