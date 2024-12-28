#include <ESP32Servo.h>
#include <WiFi.h>
#include "DHT.h"
#include <WiFiClient.h>

// WiFi credentials
const char *ssid = "NOS-17A6";
const char *password = "WPSN47JG";
//const char *ssid = "IoTDEI2";
//const char *password = "#8tud3nt2024";

// Server details
const char *server_ip = "192.168.1.2"; // Replace with your server's IP
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
int pos = 0;

// LED pins for heater and lights
#define HEATERPIN 4
#define LIGHTPIN 5

WiFiClient client;

unsigned long sensorReadTime = 0;
unsigned long dataSendTime = 0;

void setup() {
  Serial.begin(9600);

  analogSetAttenuation(ADC_11db);

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize sensors and actuators
  dht.begin();
  blindsServo.attach(SERVOPIN);
  pinMode(HEATERPIN, OUTPUT);
  pinMode(LIGHTPIN, OUTPUT);

  // Default states
  digitalWrite(HEATERPIN, LOW);
  digitalWrite(LIGHTPIN, LOW);
}

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

void loop() {
  // Connect to server
  if (!client.connected()) {
    Serial.println("Connecting to server...");
    if (client.connect(server_ip, server_port)) {
      Serial.println("Connected to server");
    } else {
      Serial.println("Failed to connect to server");
      delay(5000);
      return;
    }
  }

  // Read sensors & Measure sensor reading time
  unsigned long startSensorRead = micros();
  float temperature = dht.readTemperature();
  int lightLevel = analogRead(LDRPIN);
  unsigned long sensorReadDuration = micros() - startSensorRead;

  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor");
    delay(2000);
    return;
  }

  // 2. Measure data sending time
  unsigned long startDataSend = micros();
  String data = String(temperature) + "," + String(lightLevel) + "\n";
  client.print(data);
  unsigned long dataSendDuration = micros() - startDataSend;

  // Print timing information
  Serial.print("Sensor Read Time: ");
  Serial.print(sensorReadDuration);
  Serial.println(" microseconds");
  
  Serial.print("Data Send Time: ");
  Serial.print(dataSendDuration);
  Serial.println(" microseconds\n\n");

  // Wait for server's response
  unsigned long startCommandReceive = micros();
  if (client.available()) {
    String commands = client.readString();
    unsigned long commandReceiveDuration = micros() - startCommandReceive;
    
    Serial.print("Command Receive Time: ");
    Serial.print(commandReceiveDuration);
    Serial.println(" microseconds\n\n");

    // Parse and execute commands
    int index = 0;
    while ((index = commands.indexOf('\n')) != -1) {
      unsigned long startCommandExecution = micros();
      
      String command = commands.substring(0, index);
      command.trim();
      
      if (command == "HEATER_ON") {
        digitalWrite(HEATERPIN, HIGH);
      } else if (command == "HEATER_OFF") {
        digitalWrite(HEATERPIN, LOW);
      } else if (command == "LIGHT_ON") {
        lightOnAssembly();
      } else if (command == "LIGHT_OFF") {
        lightOffAssembly();
      } else if (command == "BLINDS_OPEN") {
        for (pos = 0; pos <= 180; pos++) {
          blindsServo.write(pos);
          delay(15);
        }
      } else if (command == "BLINDS_CLOSE") {
        for (pos = 180; pos >= 0; pos--) {
          blindsServo.write(pos);
          delay(15);
        }
      }

      unsigned long commandExecutionDuration = micros() - startCommandExecution;
      Serial.print("Command Execution Time for ");
      Serial.print(command);
      Serial.print(": ");
      Serial.print(commandExecutionDuration);
      Serial.println(" microseconds\n\n");

      // Remove the processed command
      commands = commands.substring(index + 1);
    }
  }

  delay(5000); // Wait before the next sensor reading
}