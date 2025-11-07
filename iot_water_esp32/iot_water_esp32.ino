#include <WiFi.h> // ESP32 WiFi library
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "GravityTDS.h" // Add TDS library

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
#define DHTPIN 15 // GPIO15 for DHT11
#define DHTTYPE DHT11

// ESP32 has multiple ADC pins - use different pins for each sensor
#define TDS_PIN 34       // ADC1_CH6 (GPIO34)
#define PH_PIN 35        // ADC1_CH7 (GPIO35)
#define TURBIDITY_PIN 39 // ADC1_CH4 (GPIO32)

// LCD configuration (I2C pins: SDA=21, SCL=22)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// TDS sensor object
GravityTDS gravityTds;

// WiFi and MQTT clients
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_NAME, AIO_PASS);

// Build feed path strings - MUST be kept in global scope
String phFeedPath;
String tdsFeedPath;
String temperatureFeedPath;
String turbidityFeedPath;

// Declare feed objects (initialized in setup)
Adafruit_MQTT_Publish *phFeed;
Adafruit_MQTT_Publish *tdsFeed;
Adafruit_MQTT_Publish *tempFeed;
Adafruit_MQTT_Publish *turbidityFeed;

void setup()
{
  Serial.begin(115200);

  // Initialize EEPROM for TDS sensor calibration
  EEPROM.begin(32);

  // Configure TDS sensor
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);      // ESP32 reference voltage
  gravityTds.setAdcRange(4096); // ESP32 12-bit ADC
  gravityTds.begin();           // Initialize TDS sensor

  // Configure ADC
  analogReadResolution(12);

  // Set ADC attenuation for full 0-3.3V range
  analogSetAttenuation(ADC_11db);

  // Build feed paths using String concatenation
  phFeedPath = String(AIO_NAME) + "/feeds/ph";
  tdsFeedPath = String(AIO_NAME) + "/feeds/tds";
  temperatureFeedPath = String(AIO_NAME) + "/feeds/temperature";
  turbidityFeedPath = String(AIO_NAME) + "/feeds/turbidity";

  // Initialize feed objects using c_str() - safe because Strings persist
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

  // Ping the server to keep the MQTT connection alive
  if (!mqtt.ping())
  {
    mqtt.disconnect();
  }

  // Read all sensors
  float temperature = readTemperature();
  float tdsValue = readTDS(temperature);
  float phValue = readPH();
  float turbidity = readTurbidity();

  // Display on LCD
  displayOnLCD(temperature, tdsValue, phValue, turbidity);

  // Publish to Adafruit IO
  publishToAdafruitIO(temperature, tdsValue, phValue, turbidity);

  delay(10000); // Send data every 10 seconds
}

float readTemperature()
{
  float temp = dht.readTemperature();
  if (isnan(temp))
  {
    Serial.println("Failed to read from DHT sensor!");
    return 25.0; // Default temperature
  }
  return temp;
}

float readTDS(float temperature)
{
  // Set temperature for automatic compensation
  gravityTds.setTemperature(temperature);

  // Update sensor reading
  gravityTds.update();

  // Get TDS value in ppm
  float tdsValue = gravityTds.getTdsValue();

  return tdsValue;
}

float readPH()
{
  // Read pH sensor from GPIO35
  int measurings = 0;
  for (int i = 0; i < 10; i++)
  {
    measurings += analogRead(PH_PIN);
    delay(10);
  }

  // ESP32 ADC is 12-bit: 0-4095
  float voltage = 3.3 / 4095.0 * measurings / 10;
  float phValue = 7.0 + ((2.5 - voltage) / 0.18);

  // Constrain pH to valid range
  phValue = constrain(phValue, 0.0, 14.0);

  return phValue;
}

float readTurbidity()
{
  // Read turbidity sensor from GPIO32
  int sensorValue = analogRead(TURBIDITY_PIN);
  float voltage = sensorValue * (3.3 / 4095.0);

  // Convert voltage to NTU (calibration required)
  float turbidityNTU = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;

  // Ensure non-negative turbidity
  if (turbidityNTU < 0)
    turbidityNTU = 0;

  return (turbidityNTU / 4095.0) * 100;
}

void displayOnLCD(float temp, float tds, float ph, float turbidity)
{
  // Display alternating sensor data on LCD
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
      lcd.print("Turb:");
      lcd.print(turbidity, 1);
      lcd.print("%");
    }
  }
}

void publishToAdafruitIO(float temp, float tds, float ph, float turbidity)
{
  Serial.print("Publishing - Temp: ");
  Serial.print(temp);
  Serial.print(" | TDS: ");
  Serial.print(tds);
  Serial.print(" | pH: ");
  Serial.print(ph);
  Serial.print(" | Turbidity: ");
  Serial.println(turbidity);

  // Use arrow operator -> since feeds are pointers
  if (!tempFeed->publish(temp))
  {
    Serial.println("Failed to publish temperature");
  }
  delay(100);

  if (!tdsFeed->publish(tds))
  {
    Serial.println("Failed to publish TDS");
  }
  delay(100);

  if (!phFeed->publish(ph))
  {
    Serial.println("Failed to publish pH");
  }
  delay(100);

  if (!turbidityFeed->publish(turbidity))
  {
    Serial.println("Failed to publish turbidity");
  }
}

void connectMQTT()
{
  Serial.print("Connecting to Adafruit IO... ");
  lcd.clear();
  lcd.print("Connecting AIO");

  int8_t ret;
  while ((ret = mqtt.connect()) != 0)
  {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }

  Serial.println("Adafruit IO Connected!");
  lcd.clear();
  lcd.print("AIO Connected!");
  delay(1000);
}
