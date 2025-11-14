#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "GravityTDS.h"

extern const char *AIO_NAME;
extern const char *AIO_PASS;
extern const char *ssid;
extern const char *pass;

// WiFi credentials
#define WLAN_SSID ssid
#define WLAN_PASS pass

// Adafruit IO credentials
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883

// Sensor pins - ESP32 specific
#define DHTPIN 15        // GPIO15 for DHT11
#define DHTTYPE DHT11
#define TDS_PIN 34       // ADC1_CH6 (GPIO34)
#define PH_PIN 35        // ADC1_CH7 (GPIO35)
#define TURBIDITY_PIN 33 // GPIO33 (changed from 32)

// Turbidity calibration for 1000 NTU sensor
#define TURBIDITY_SAMPLES 20
#define VOLTAGE_CLEAR_WATER 4.2    // Ideal voltage in clear water (0 NTU)
#define VOLTAGE_MAX_TURBIDITY 2.5  // Voltage at 1000 NTU
#define MAX_NTU 1000.0             // Your sensor's maximum range

// LCD configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// TDS sensor object
GravityTDS gravityTds;

// WiFi and MQTT clients
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_NAME, AIO_PASS);

// Build feed path strings
String phFeedPath;
String tdsFeedPath;
String temperatureFeedPath;
String turbidityFeedPath;

// Declare feed objects
Adafruit_MQTT_Publish *phFeed;
Adafruit_MQTT_Publish *tdsFeed;
Adafruit_MQTT_Publish *tempFeed;
Adafruit_MQTT_Publish *turbidityFeed;

// Store calibrated clear water voltage
float calibratedClearVoltage = 3.3; // Will be measured during setup

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Water Quality Monitor v2.0 ===");
  Serial.println("=== Turbidity Sensor: 0-1000 NTU ===\n");

  // Initialize EEPROM
  EEPROM.begin(32);

  // Configure TDS sensor
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Build feed paths
  phFeedPath = String(AIO_NAME) + "/feeds/ph";
  tdsFeedPath = String(AIO_NAME) + "/feeds/tds";
  temperatureFeedPath = String(AIO_NAME) + "/feeds/temperature";
  turbidityFeedPath = String(AIO_NAME) + "/feeds/turbidity";

  // Initialize feed objects
  phFeed = new Adafruit_MQTT_Publish(&mqtt, phFeedPath.c_str());
  tdsFeed = new Adafruit_MQTT_Publish(&mqtt, tdsFeedPath.c_str());
  tempFeed = new Adafruit_MQTT_Publish(&mqtt, temperatureFeedPath.c_str());
  turbidityFeed = new Adafruit_MQTT_Publish(&mqtt, turbidityFeedPath.c_str());

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Water Quality");
  lcd.setCursor(0, 1);
  lcd.print("Monitor v2.0");
  delay(2000);

  // Initialize DHT sensor
  dht.begin();

  // Calibrate turbidity sensor
  Serial.println("\n--- Turbidity Sensor Calibration ---");
  Serial.println("IMPORTANT: Place sensor in CLEAN water for calibration!");
  Serial.println("Reading in 3 seconds...");
  delay(3000);
  
  // Take calibration reading
  long sumCal = 0;
  for (int i = 0; i < 20; i++)
  {
    sumCal += analogRead(TURBIDITY_PIN);
    delay(50);
  }
  float avgADC = sumCal / 20.0;
  calibratedClearVoltage = avgADC * (3.3 / 4095.0);
  
  Serial.print("Calibration Complete!");
  Serial.print(" Clear Water Voltage: ");
  Serial.print(calibratedClearVoltage, 3);
  Serial.println("V");
  
  if (calibratedClearVoltage < 2.0 || calibratedClearVoltage > 3.3)
  {
    Serial.println("WARNING: Unusual voltage reading!");
    Serial.println("Check sensor connection and power supply.");
    Serial.println("Expected range: 2.5-3.3V in clear water");
  }
  
  Serial.println("Calibration values:");
  Serial.print("  Clear water (0 NTU): ");
  Serial.print(calibratedClearVoltage, 2);
  Serial.println("V = 100% clarity");
  Serial.print("  Max turbidity (1000 NTU): ");
  Serial.print(VOLTAGE_MAX_TURBIDITY, 2);
  Serial.println("V = 0% clarity");
  Serial.println("--------------------------------\n");

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  lcd.clear();
  lcd.print("Connecting WiFi");
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.print("WiFi Connected!");
  delay(1000);

  // Connect to Adafruit IO
  connectMQTT();
}

void loop()
{
  // Maintain MQTT connection
  if (!mqtt.connected())
  {
    connectMQTT();
  }
  mqtt.processPackets(10);

  if (!mqtt.ping())
  {
    mqtt.disconnect();
  }

  // Read all sensors
  float temperature = readTemperature();
  float tdsValue = readTDS(temperature);
  float phValue = readPH();
  float turbidityPercent = readTurbidity();

  // Display on LCD
  displayOnLCD(temperature, tdsValue, phValue, turbidityPercent);

  // Publish to Adafruit IO
  publishToAdafruitIO(temperature, tdsValue, phValue, turbidityPercent);

  delay(10000);
}

float readTemperature()
{
  float temp = dht.readTemperature();
  if (isnan(temp))
  {
    Serial.println("Failed to read from DHT sensor!");
    return 25.0;
  }
  return temp;
}

float readTDS(float temperature)
{
  gravityTds.setTemperature(temperature);
  gravityTds.update();
  float tdsValue = gravityTds.getTdsValue();
  return tdsValue;
}

float readPH()
{
  int measurings = 0;
  for (int i = 0; i < 10; i++)
  {
    measurings += analogRead(PH_PIN);
    delay(10);
  }

  float voltage = 3.3 / 4095.0 * measurings / 10;
  float phValue = 7.0 + ((2.5 - voltage) / 0.18);
  phValue = constrain(phValue, 0.0, 14.0);

  return phValue;
}

float readTurbidity()
{
  long sum = 0;

  for (int i = 0; i < TURBIDITY_SAMPLES; i++)
  {
    sum += analogRead(TURBIDITY_PIN);
    delay(10);
  }

  float adc = sum / (float)TURBIDITY_SAMPLES;
  float voltage = adc * (3.3 / 4095.0);

  // --- Correct turbidity calibration range ---
  float clearWaterVoltage = calibratedClearVoltage;  // ~2.8 - 3.3V
  float veryDirtyVoltage  = 1.2;  // Typical voltage in 1000 NTU

  // Clamp voltage into valid range
  if (voltage > clearWaterVoltage) voltage = clearWaterVoltage;
  if (voltage < veryDirtyVoltage) voltage = veryDirtyVoltage;

  // Map to 0–100% clarity
  float clarity = (voltage - veryDirtyVoltage) * 100.0 /
                  (clearWaterVoltage - veryDirtyVoltage);

  clarity = constrain(clarity, 0, 100);

  Serial.print("Turbidity | ADC:");
  Serial.print(adc);
  Serial.print("  Voltage:");
  Serial.print(voltage, 3);
  Serial.print(" V  Clarity:");
  Serial.print(clarity, 1);
  Serial.println("%");

  return clarity;
}


void displayOnLCD(float temp, float tds, float ph, float turbidity)
{
  static unsigned long lastSwitch = 0;
  static int displayMode = 0;

  if (millis() - lastSwitch > 3000)
  {
    lastSwitch = millis();
    displayMode = (displayMode + 1) % 2;

    lcd.clear();
    if (displayMode == 0)
    {
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(temp, 1);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("pH:");
      lcd.print(ph, 2);
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("TDS:");
      lcd.print(tds, 0);
      lcd.print("ppm");

      lcd.setCursor(0, 1);
      lcd.print("Clarity:");
      lcd.print(turbidity, 0);
      lcd.print("%");
    }
  }
}

void publishToAdafruitIO(float temp, float tds, float ph, float turbidity)
{
  Serial.println("\n--- Publishing to Adafruit IO ---");
  Serial.print("Temp: ");
  Serial.print(temp, 1);
  Serial.print("°C | TDS: ");
  Serial.print(tds, 0);
  Serial.print("ppm | pH: ");
  Serial.print(ph, 2);
  Serial.print(" | Clarity: ");
  Serial.print(turbidity, 1);
  Serial.println("%");

  if (!tempFeed->publish(temp))
  {
    Serial.println("✗ Failed to publish temperature");
  }
  else
  {
    Serial.println("✓ Temperature published");
  }
  delay(100);

  if (!tdsFeed->publish(tds))
  {
    Serial.println("✗ Failed to publish TDS");
  }
  else
  {
    Serial.println("✓ TDS published");
  }
  delay(100);

  if (!phFeed->publish(ph))
  {
    Serial.println("✗ Failed to publish pH");
  }
  else
  {
    Serial.println("✓ pH published");
  }
  delay(100);

  if (!turbidityFeed->publish(turbidity))
  {
    Serial.println("✗ Failed to publish turbidity");
  }
  else
  {
    Serial.println("✓ Turbidity published");
  }

  Serial.println("--- Publish Complete ---\n");
}

void connectMQTT()
{
  Serial.print("Connecting to Adafruit IO... ");
  lcd.clear();
  lcd.print("Connecting AIO");

  int8_t ret;
  uint8_t retries = 0;

  while ((ret = mqtt.connect()) != 0)
  {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying in 5 seconds...");
    mqtt.disconnect();
    delay(5000);

    retries++;
    if (retries > 5)
    {
      Serial.println("Failed to connect to MQTT after 5 attempts. Restarting...");
      ESP.restart();
    }
  }

  Serial.println("Adafruit IO Connected!");
  lcd.clear();
  lcd.print("AIO Connected!");
  delay(1000);
}
