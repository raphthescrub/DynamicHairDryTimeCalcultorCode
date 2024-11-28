#include <OneWire.h>
#include <DallasTemperature.h>
#include <SimpleKalmanFilter.h>
#include <WiFi.h>
#include "DHT.h"
#include "ThingSpeak.h"

// WiFi Configuration
const char *ssid = "SSID";   // Your WiFi SSID
const char *password = "PASSWORD";    // Your WiFi password

// ThingSpeak Configuration
WiFiClient client;
unsigned long myChannelNumber = 000000;//Insert Your Channel Number
const char *myWriteAPIKey = "APIKey";//InsertAPIKey

// DS18B20 Sensor Pin
#define ONE_WIRE_BUS 33
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// DHT Sensor Configuration
const int DHTPIN = 4;
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Thermistor Configuration
const int thermistorPin = 27;
const float R1 = 1100;   // Bias resistor value
const float Vin = 3.3;   // Input voltage

// Moisture Sensor Configuration
const int MoisturePin = 35;

// Kalman Filter
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Global Variables
float TemperatureHair, HairMoisture, EstimatedTime;
float ThermistorResistance, Temp, kconstant;
int MoistPrevious = 0;
unsigned long StartTime = 0;

// Function Declarations
float calculateThermistorResistance(float Vout, float Vin);
float LinearTemperature(float Resistance);
float celsiusToFahrenheit(float celsius);
float ConvertADC(int VoltageReading);
float kconstantCalc(float Temperature);
float TimeCalculation(float kconstant, int Mcurrent, int Mfinal, int Minitial);
float KalmanFilterFunc(float input);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("Hair Moisture Sensor System Initialized");

  // Initialize DS18B20 Sensor
  sensors.begin();

  // Initialize DHT Sensor
  dht.begin();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected!");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Initialize Moisture Sensor
  MoistPrevious = analogRead(MoisturePin);
}

void loop() {
  // Read Thermistor and Calculate Temperature
  int ThermistorReading = analogRead(thermistorPin);
  float SensorVout = ConvertADC(ThermistorReading);
  ThermistorResistance = calculateThermistorResistance(SensorVout, Vin);
  Temp = LinearTemperature(ThermistorResistance);
  float estimatedTemp = KalmanFilterFunc(Temp);

  // Read DS18B20 Hair Temperature
  sensors.requestTemperatures();
  TemperatureHair = sensors.getTempFByIndex(0);

  // Read Moisture Sensor
  int MoistureDigital = analogRead(MoisturePin);
  MoistureDigital = KalmanFilterFunc(MoistureDigital);

  // Read Humidity
  float humidity = dht.readHumidity();

  // Calculate Drying Time
  kconstant = kconstantCalc(TemperatureHair);
  EstimatedTime = TimeCalculation(kconstant, MoistureDigital, 2000, 1100);

  // Log Sensor Data
  Serial.print("Estimated Time: ");
  Serial.println(EstimatedTime);

  // Update ThingSpeak
  ThingSpeak.writeField(myChannelNumber, 1, MoistureDigital, myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber, 2, EstimatedTime, myWriteAPIKey);

  // Handle Slope-based Time Estimation
  int slope = MoistureDigital - MoistPrevious;
  MoistPrevious = MoistureDigital;

  // Reset Start Time if slope is zero
  if (slope == 0) {
    StartTime = millis();
  }

  delay(1000);  // Adjust delay as needed
}

// Function Definitions
float calculateThermistorResistance(float Vout, float Vin) {
  return (R1 * (Vout + (5 * Vin) - 1)) / ((5 * Vin) - Vout + 1);
}

float LinearTemperature(float Resistance) {
  return (-0.0185 * Resistance) + 44.244;
}

float ConvertADC(int VoltageReading) {
  return ((VoltageReading + 280) / 1333.893) - 0.1;
}

float kconstantCalc(float Temperature) {
  float TempFar = Temperature; // Assuming Temperature is in Fahrenheit
  float kconst = 0.002 * exp(0.0393 * TempFar);
  Serial.println(kconst);
  return kconst;
}

float TimeCalculation(float kconstant, int Mcurrent, int Mfinal, int Minitial) {
  if (Mfinal <= Mcurrent) return 0; // Avoid invalid calculations
  return -1 * ((1 / kconstant) * log((float)(Mfinal - Mcurrent) / (Mfinal - Minitial)));
}

float KalmanFilterFunc(float input) {
  return simpleKalmanFilter.updateEstimate(input);
}
