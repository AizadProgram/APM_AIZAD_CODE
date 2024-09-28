#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include "MQ7.h"
#include <MQ135.h>
#include <HardwareSerial.h>
void setup_wifi();

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  

//-------WiFI-------------------------------------
const char* ssid = "usernanme wifi";     // put wi-fi id
const char* password = "password wifi";  // wi-fi password

//-------MQTT-------------------------------------
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;    //mqtt
PubSubClient client(espClient); //MQTT

#define BUZZER_PIN 2

#define SDA_PIN 13
#define SCL_PIN 14

#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 9600 // Kecepatan baudrate sensor PMS7003

HardwareSerial pmsSerial(1); // Gunakan UART 1 pada ESP32

//-------------------------------------Constant Data ----------------------------------------------------//
#define A_PIN 36
#define VOLTAGE 12
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

const int mq135_analog_pin = A0;
unsigned long buzzerStartTime = 0; // Variable to store the start time of buzzer activation
const unsigned long buzzerDuration = 10000; // Duration for which the buzzer will sound in milliseconds (e.g., 10 seconds)
unsigned long delayTime;

//-------------------------------------Variable ----------------------------------------------------//
int pm2_5 = 0;
int co_sensor = 33;
char varPotString[8];
unsigned long lastMsg = 0, last = 0, UpDate = 0;
char msg[50];
int value = 0;
int sensor_value = 0;


// init MQ7 device
MQ7 mq7(A_PIN, VOLTAGE);



void setup()
{
  {

    Serial.begin(115200);
    Wire.begin(13, 14); //sda
    
    pinMode(co_sensor, INPUT);
    lcd.init();
    lcd.backlight();

    
    lcd.setCursor(0,0);
    lcd.print(" WELCOME TO SYSTEM ");
    lcd.setCursor(5,2);
    lcd.print("AIR QUALITY");
    lcd.setCursor(0,3);
    lcd.print("  MONITORING SYSTEM");
    delay(4000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connecting To WiFi..");
    lcd.setCursor(0,1);
    lcd.print("Status : DISCONNECT");
    pmsSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    setup_wifi();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connecting To WiFi..");
    lcd.setCursor(0,1);
    lcd.print("Status : CONNECT");
    delay(4000);
    client.setServer(mqtt_server, 1883); //mqtt
    client.setCallback(callback); //mqtt

    while (!Serial);   // time to get serial running
    Serial.println(F("BME280 test"));
    unsigned status;

    // default settings
    status = bme.begin(0x76);


    if (!status)
    {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
      Serial.print("\t\tID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("\t\tID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("\t\tID of 0x60 represents a BME 280.\n");
      Serial.print("\t\tID of 0x61 represents a BME 680.\n");
      while (1) delay(10);

    }
    Serial.println("-- Default Test --");
    delayTime = 1000;
    Serial.println();

    Serial.println("");   // blank new line

    Serial.println("Calibrating MQ7");
    mq7.calibrate();    // calculates R0
    Serial.println("Calibration done!");
  }
}


void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();

  if (now - lastMsg > 2000)
  {
    dtostrf(sensor_value, 1, 1, varPotString);
    client.publish("sensor3/ppmmq135", varPotString);

    dtostrf(mq7.readPpm(), 1, 1, varPotString);
    client.publish("sensor2/ppmmq7", varPotString);

    dtostrf(pm2_5, 1, 1, varPotString);
    client.publish("sensor4/pms7003", varPotString);

    dtostrf(bme.readTemperature(), 1, 1, varPotString);
    client.publish("sensor1/BME280", varPotString);

    lastMsg = now;
  }
  if (now - last > 1000) {
    buzzer();
    BME280SENSOR();
    MQ7_S();
    MQ135_S();
    readPMSData();
    last = now;
  }
  if(now - UpDate > 2000){
    UpDate = now;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("DIOXIDE  : ");
    lcd.setCursor(11,0);
    lcd.print(sensor_value);
    
    lcd.setCursor(0,1);
    lcd.print("MONOXIDE :");
    lcd.setCursor(11,1);
    lcd.print(mq7.readPpm());

    lcd.setCursor(0,2);
    lcd.print("HUMIDITY :");
    lcd.setCursor(12,2);
    lcd.print(bme.readTemperature());
  }
}

void buzzer() {
  if (mq7.readPpm() > 51 || mq135_analog_pin > 200 || pm2_5 > 10000 || bme.readTemperature() > 37.5) {
    sirenEffect(); // Trigger the buzzer
    buzzerStartTime = millis(); // Record the start time of the buzzer activation
  }
  else{
    noTone(BUZZER_PIN);
  }

  // Check if the buzzer has been active for the specified duration
//  if (millis() - buzzerStartTime >= buzzerDuration) {
//    noTone(BUZZER_PIN); // Turn off the buzzer if the duration has elapsed
//  }

  //delay(1000); // Adjust delay according to your requirement
}

void sirenEffect() {
  // Creating siren effect
  for (int freq = 1000; freq <= 5000; freq += 100) {
    tone(BUZZER_PIN, freq);
    delay(10); // Delay for each frequency
  }

  for (int freq = 5000; freq >= 1000; freq -= 100) {
    tone(BUZZER_PIN, freq);
    delay(10); // Delay for each frequency
  }
}

void MQ135_S() {

  // Read analog value from MQ135 sensor
  sensor_value = analogRead(mq135_analog_pin);

  // Publish sensor value to MQTT topic
//  char varPotString[8]; // Ensure varPotString is properly declared
//  dtostrf(sensor_value, 1, 1, varPotString);
//  client.publish("sensor3/ppmmq135", varPotString);

  // Print the sensor value to Serial monitor (optional)
  Serial.print("MQ135 Sensor Value: ");
  Serial.println(sensor_value);
  Serial.println("");   // blank new line

  //delay(1000); // Adjust delay as needed
}

void MQ7_S() {
  Serial.print("PPM = ");
  Serial.println(mq7.readPpm());
  Serial.println("");   // blank new line
  //delay(1000);
  //--------------publish nilai ppm mq7 ke mqtt

//  varPotString[8];
//  dtostrf(mq7.readPpm(), 1, 1, varPotString);
//  client.publish("sensor2/ppmmq7", varPotString);
}

void readPMSData() {
  unsigned char buffer[32];
  pmsSerial.readBytes(buffer, 32);

  // Extract data from the buffer
  int pm1_0 = makeWord(buffer[4], buffer[5]);
  int pm2_5 = makeWord(buffer[6], buffer[7]);
  int pm10 = makeWord(buffer[8], buffer[9]);

  // Print the particle concentration values
  Serial.print("PM1.0: ");
  Serial.println(pm1_0);
  Serial.print("PM2.5: ");
  Serial.println(pm2_5);
  Serial.print("PM10: ");
  Serial.println(pm10);

//  varPotString[8]; // Ensure varPotString is properly declared
//  dtostrf(pm2_5, 1, 1, varPotString);
//  client.publish("sensor4/pms7003", varPotString);
  // Additional data can be extracted from the buffer if needed
}

//void BME280SENSOR() {
//  BME280();
//  //delay(delayTime);
//}

void BME280SENSOR() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println();


  //--------------publish mesej dari bme280 ke mqtt

//  varPotString[8];
//  dtostrf(bme.readTemperature(), 1, 1, varPotString);
//  client.publish("sensor1/BME280", varPotString);
}

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(3000);
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  if (String(topic) == "home/Relay02") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      //digitalWrite(Relay02,HIGH);
      client.publish("home/relay02/status", "RELAY 1 ON");

    }
    else if (messageTemp == "off") {
      Serial.println("off");
      //digitalWrite(Relay02,LOW);
      client.publish("home/relay02/status", "RELAY 1 OFF");
    }
  }

  if (String(topic) == "home/suis2") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      //digitalWrite(Relay02,HIGH);
      client.publish("led/status", "LED 1 ON");
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      //digitalWrite(Relay02,LOW);
      client.publish("led/status", "LED 1 OFF");
    }
  }

}

//----client subscribe----------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("airmonitor123")) {    //client mesti unik dan berbeza----------
      Serial.println("connected");
      // Subscribe
      //client.subscribe("home/Relay02");
      //client.subscribe("home/suis2");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
