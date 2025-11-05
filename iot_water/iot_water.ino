#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

extern const char *AIO_NAME;
extern const char *AIO_PASS;

extern const char *ssid;  // Enter your WiFi Name
extern const char *pass;  // Enter your WiFi Password

// WiFi credentials
#define WLAN_SSID ssid
#define WLAN_PASS pass

// Adafruit IO credentials
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
// #define AIO_USERNAME AIO_NAME
// #define AIO_KEY AIO_PASS

// Sensor pins
#define DHTPIN 2  // was D4
#define DHTTYPE DHT11
#define TDS_PIN A0
#define PH_PIN A0
#define TURBIDITY_PIN A0

// Multiplexer control pins
#define MUX_TDS 14        // was D5
#define MUX_PH 12         // was D6
#define MUX_TURBIDITY 13  // was D7

// LCD configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change 0x27 to 0x3F if not working

// DHT sensor
DHT dht(DHTPIN, DHTTYPE);

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

// TDS variables
#define VREF 3.3
#define SCOUNT 30
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;

void setup() {
  Serial.begin(115200);

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
  lcd.print("Monitor v1.0");
  delay(2000);

  // Initialize DHT sensor
  dht.begin();

  // Initialize multiplexer pins
  pinMode(MUX_TDS, OUTPUT);
  pinMode(MUX_PH, OUTPUT);
  pinMode(MUX_TURBIDITY, OUTPUT);
  digitalWrite(MUX_TDS, LOW);
  digitalWrite(MUX_PH, LOW);
  digitalWrite(MUX_TURBIDITY, LOW);

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  lcd.clear();
  lcd.print("Connecting WiFi");
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  lcd.clear();
  lcd.print("WiFi Connected!");
  delay(1000);

  // Connect to Adafruit IO
  connectMQTT();
}

void loop() {
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    connectMQTT();
  }
  mqtt.processPackets(10);
  mqtt.ping();

  // Read all sensors
  float temperature = readTemperature();
  float tdsValue = readTDS(temperature);
  float phValue = readPH();
  float turbidity = readTurbidity();

  // Display on LCD
  displayOnLCD(temperature, tdsValue, phValue, turbidity);

  // Publish to Adafruit IO
  publishToAdafruitIO(temperature, tdsValue, phValue, turbidity);

  delay(10000);  // Send data every 10 seconds
}

float readTemperature() {
  float temp = dht.readTemperature();
  if (isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return 25.0;  // Default temperature
  }
  return temp;
}

float readTDS(float temperature) {
  // Enable TDS sensor
  digitalWrite(MUX_TDS, HIGH);
  digitalWrite(MUX_PH, LOW);
  digitalWrite(MUX_TURBIDITY, LOW);
  delay(100);

  // Read TDS sensor
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  // Calculate average voltage
  float averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (VREF / 1024.0);

  // Temperature compensation
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;

  // Convert to TDS value
  float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
                    - 255.86 * compensationVoltage * compensationVoltage
                    + 857.39 * compensationVoltage)
                   * 0.5;

  digitalWrite(MUX_TDS, LOW);
  return tdsValue;
}

float readPH() {
  // Enable pH sensor
  digitalWrite(MUX_PH, HIGH);
  digitalWrite(MUX_TDS, LOW);
  digitalWrite(MUX_TURBIDITY, LOW);
  delay(100);

  int measurings = 0;
  for (int i = 0; i < 10; i++) {
    measurings += analogRead(PH_PIN);
    delay(10);
  }

  float voltage = 3.3 / 1024.0 * measurings / 10;
  float phValue = 7.0 + ((2.5 - voltage) / 0.18);

  digitalWrite(MUX_PH, LOW);
  return phValue;
}

float readTurbidity() {
  // Enable turbidity sensor
  digitalWrite(MUX_TURBIDITY, HIGH);
  digitalWrite(MUX_TDS, LOW);
  digitalWrite(MUX_PH, LOW);
  delay(100);

  int sensorValue = analogRead(TURBIDITY_PIN);
  float voltage = sensorValue * (3.3 / 1024.0);

  // Convert voltage to NTU (calibration required)
  float turbidityNTU = -1120.4 * voltage * voltage + 5742.3 * voltage - 4352.9;

  digitalWrite(MUX_TURBIDITY, LOW);
  return turbidityNTU;
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void displayOnLCD(float temp, float tds, float ph, float turbidity) {
  // Display alternating sensor data on LCD
  static unsigned long lastSwitch = 0;
  static int displayMode = 0;

  if (millis() - lastSwitch > 3000) {
    lastSwitch = millis();
    displayMode = (displayMode + 1) % 2;

    lcd.clear();
    if (displayMode == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(temp, 1);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("pH:");
      lcd.print(ph, 2);
    } else {
      lcd.setCursor(0, 0);
      lcd.print("TDS:");
      lcd.print(tds, 0);
      lcd.print("ppm");

      lcd.setCursor(0, 1);
      lcd.print("Turb:");
      lcd.print(turbidity, 1);
      lcd.print("NTU");
    }
  }
}

void publishToAdafruitIO(float temp, float tds, float ph, float turbidity) {
  Serial.print("Publishing - Temp: ");
  Serial.print(temp);
  Serial.print(" | TDS: ");
  Serial.print(tds);
  Serial.print(" | pH: ");
  Serial.print(ph);
  Serial.print(" | Turbidity: ");
  Serial.println(turbidity);

  // Use arrow operator -> since feeds are pointers
  if (!tempFeed->publish(temp)) {
    Serial.println("Failed to publish temperature");
  }
  delay(100);

  if (!tdsFeed->publish(tds)) {
    Serial.println("Failed to publish TDS");
  }
  delay(100);

  if (!phFeed->publish(ph)) {
    Serial.println("Failed to publish pH");
  }
  delay(100);

  if (!turbidityFeed->publish(turbidity)) {
    Serial.println("Failed to publish turbidity");
  }
}

void connectMQTT() {
  Serial.print("Connecting to Adafruit IO... ");
  lcd.clear();
  lcd.print("Connecting AIO");

  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
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
