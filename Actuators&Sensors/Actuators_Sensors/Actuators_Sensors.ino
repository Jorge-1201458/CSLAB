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

//const char *ssid = "MEO-CASA";
//const char *password = "229823683";

//const char *ssid = "IoTDEI2";
//const char *password = "#8tud3nt2024";

// Server details
const char *server_ip = "192.168.1.8";
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
#define SEND_DATA_PERIOD 3000

// Tasks priorities (Since both tasks run on different cores the priority is the same)
#define PRIORITY 1

//Register for Lights
#define GPIO_OUT_REG 0x3FF44004
#define GPIO_OUT_W1TS_REG 0x3FF44008  // Register to set GPIO pins high
#define GPIO_OUT_W1TC_REG 0x3FF4400C  // Register to set GPIO pins low


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

// Optimized assembly function for GPIO control
// Parameter: state (1 for on, 0 for off)
void lightControlAssembly(bool state) {
    asm volatile (
        // Load appropriate register address based on state
        "movi    a2, %[set_reg]\n"      // Load GPIO SET register address
        "movi    a3, %[clear_reg]\n"    // Load GPIO CLEAR register address
        "mov     a4, %[state]\n"        // Load state parameter
        
        // Prepare GPIO mask (1 << 5 for GPIO5)
        "movi    a5, 1\n"               // Load 1
        "slli    a5, a5, 5\n"           // Shift to create mask for GPIO5
        
        // Choose between set and clear based on state
        "beqz    a4, clear\n"           // If state is 0, jump to clear
        
        // Set GPIO high
        "s32i    a5, a2, 0\n"           // Write to SET register
        "j       done\n"
        
        // Set GPIO low
        "clear:\n"
        "s32i    a5, a3, 0\n"           // Write to CLEAR register
        
        "done:\n"
        
        : // no outputs
        : [state] "r" (state),
          [set_reg] "i" (GPIO_OUT_W1TS_REG),    // GPIO SET register
          [clear_reg] "i" (GPIO_OUT_W1TC_REG)    // GPIO CLEAR register
        : "a2", "a3", "a4", "a5", "memory"
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
        //lightOnAssembly();
        lightControlAssembly(1);
    } else if (command == "LIGHT_OFF\n") {
        //digitalWrite(LIGHTPIN, LOW);
        //lightOffAssembly();
        lightControlAssembly(0);
    } else if (command == "BLINDS_OPEN\n") {
        for (int pos = 180; pos <= 360; pos++) {
            blindsServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    } else if (command == "BLINDS_HALF_OPEN\n") {
        for (int pos = 0; pos <= 180; pos++) {
            blindsServo.write(pos);
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    } else if (command == "BLINDS_HALF_CLOSE\n") {
        for (int pos = 360; pos >= 180; pos--) {
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
        //unsigned long executionTime = micros() - startTime;
        //Serial.printf("Read Task Execution Time: %lu microseconds\n", executionTime);

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
       // unsigned long executionTime = micros() - startTime;
        //Serial.printf("Send Task Execution Time: %lu microseconds\n", executionTime);

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
        PRIORITY,
        NULL,                       // Task handle
        1                          // Core ID (1 = secondary core)
    );

    xTaskCreatePinnedToCore(
        sendDataTask,               // Task function
        "SendDataTask",            // Name
        4096,                      // Stack size
        NULL,                      // Parameters
        PRIORITY,
        NULL,                      // Task handle
        0                         // Core ID (0 = primary core)
    );
}

void loop() {
    // Empty loop - tasks handle everything
    vTaskDelete(NULL);
}