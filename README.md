#include <Wire.h>                     // Library for I2C communication
#include <Adafruit_MPU6050.h>         // Library for MPU6050 sensor
#include <Adafruit_Sensor.h>          // General sensor library for Adafruit sensors
#include <Adafruit_GPS.h>             // Library for GPS communication
#include <WiFi.h>                     // Wi-Fi library for ESP32
#include <HTTPClient.h>               // HTTP library for sending data to the server

// Wi-Fi credentials (Replace with your Wi-Fi SSID and Password)
const char* ssid = "YourSSID";
const char* password = "YourPassword";

// Server API endpoint
const char* serverUrl = "http://your-server.com/api";

// GPS setup
HardwareSerial gpsSerial(1);          // Use UART1 for GPS communication
Adafruit_GPS GPS(&gpsSerial);         // GPS object

// MPU6050 setup
Adafruit_MPU6050 mpu;                 // MPU6050 object

// Pins for buzzer and LED
const int buzzerPin = 15;             // Pin for buzzer
const int ledPin = 2;                 // Pin for LED

// Thresholds for violations
float accidentThreshold = 2.5;        // Threshold for detecting sudden acceleration (m/s^2)
float speedThreshold = 30.0;          // Speed limit threshold (km/h)

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");

  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // UART pins: RX=16, TX=17
  GPS.begin(9600);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1); // Stop execution if MPU6050 fails to initialize
  }

  // Initialize buzzer and LED
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Read GPS data
  GPS.read();
  if (GPS.newNMEA()) {
    float latitude = GPS.latitudeDegrees;    // Latitude from GPS
    float longitude = GPS.longitudeDegrees;  // Longitude from GPS
    float speed = GPS.speed * 1.852;         // Convert speed from knots to km/h

    // Debugging: Print GPS data to Serial
    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude);
    Serial.print("Speed: ");
    Serial.println(speed);

    // Check for speed violations
    if (speed > speedThreshold) {
      Serial.println("Speed violation detected!");
      triggerAlarm(); // Trigger alarm for speed violation
    }

    // Read accelerometer data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Check for sudden acceleration (accident detection)
    if (abs(a.acceleration.x) > accidentThreshold ||
        abs(a.acceleration.y) > accidentThreshold ||
        abs(a.acceleration.z) > accidentThreshold) {
      Serial.println("Accident detected!");
      triggerAlarm(); // Trigger alarm for accident detection
      sendDataToServer(latitude, longitude, true, "accident"); // Send data with accident flag
    } else {
      // Send normal data to server
      sendDataToServer(latitude, longitude, false, "normal");
    }
  }

  delay(1000); // Delay for stability (1 second)
}

// Function to trigger alarm (buzzer and LED)
void triggerAlarm() {
  digitalWrite(buzzerPin, HIGH); // Activate buzzer
  digitalWrite(ledPin, HIGH);   // Turn on LED
  delay(1000);                  // Alarm duration: 1 second
  digitalWrite(buzzerPin, LOW); // Deactivate buzzer
  digitalWrite(ledPin, LOW);    // Turn off LED
}

// Function to send data to the server
void sendDataToServer(float latitude, float longitude, bool accident, String violation) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl); // Set server URL

    // Create JSON payload
    String payload = "{\"device_id\":\"BUS001\",";
    payload += "\"latitude\":" + String(latitude, 6) + ",";
    payload += "\"longitude\":" + String(longitude, 6) + ",";
    payload += "\"accident\":" + String(accident) + ",";
    payload += "\"violation\":\"" + violation + "\"}";

    // Set HTTP headers and send POST request
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(payload);

    // Debugging: Print server response
    if (httpResponseCode > 0) {
      Serial.print("Server response: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error sending data: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // Close HTTP connection
  } else {
    Serial.println("Wi-Fi not connected!"); // Debugging message if Wi-Fi is not available
  }
}

