#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Servo.h> // Mengimpor pustaka Servo
#include "DHT.h"

// Define pins for TRIG and ECHO
#define TRIG_PIN 4
#define ECHO_PIN 5
#define FAN 6
#define PUMP 7
#define GETARAN 8
#define RED_LED 10
#define GREEN_LED 11

//Include the servo motor library
#include <Servo.h>
//Define the LDR sensor pins
#define RAIN A0
#define LDR1 A1
#define LDR2 A2
//Define the error value. You can change it as you like
#define error 10
//Starting point of the servo motor
int Spoint =  90;
//Create an object for the servo motor
Servo servo;

Adafruit_INA219 ina219;

int angleX = 0;
int angleY = 0;

// Konfigurasi DHT22
#define DHTPIN 9// Pin data DHT22 terhubung ke GPIO 4
#define DHTTYPE DHT22  // Jenis sensor DHT
DHT dht(DHTPIN, DHTTYPE);

float humidity = 0.0;
float temperature = 0.0;
int valueGetaran = 0;
int rainValue = 0;
int intensitasHujan = 0;
int lightIntensity1 = 0;
int lightIntensity2 = 0;

// Variabel timer 
unsigned long previousMillis = 0;
const long interval = 2000;  // Interval pembacaan data sensor (ms)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  dht.begin();          // Inisialisasi sensor DHT22

  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FAN, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(GETARAN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Optional: Test message
  Serial.println("HC-SR04 Distance Measurement");

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  ina219.setCalibration_32V_1A(); 
  // Or to use a lower 16V, 40`0mA range (higher precision on volts and amps):
  // ina219.setCalibration_16V_400mA();
  Serial.println("Measuring voltage and current with INA219 ...");

  servo.attach(3);
}

void loop() {
  // read temperature
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Membaca data dari sensor DHT22
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    // Validasi pembacaan data
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
  }

  Serial.print("Temperature:   "); Serial.print(temperature); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" g/kg");

  // Wait before next measurement
  delay(100);

  // Read rain sensor and control pump
  rainValue = analogRead(RAIN);
  intensitasHujan = map(rainValue, 0, 1023, 100, 0);
  Serial.print("Intensitas Hujan: ");
  Serial.println(intensitasHujan);
  if (rainValue < 500) {
      digitalWrite(PUMP, HIGH);
      Serial.println("Hujan terdeteksi, pompa dimatikan.");
  } else {
      digitalWrite(PUMP, LOW);
      Serial.println("Tidak hujan, pompa dinyalakan.");
  }

  // read current, load voltage
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");

  if(temperature > 30)
  {
    digitalWrite(FAN, HIGH);
  }else
  {
    digitalWrite(FAN, LOW);
  }

  //Get the LDR sensor value
  int ldr1 = analogRead(LDR1);
  //Get the LDR sensor value
  int ldr2 = analogRead(LDR2);
  Serial.print("Light Intensity 1: ");
  Serial.println(ldr1);
  Serial.print("Light Intensity 2: ");
  Serial.println(ldr2);
  lightIntensity1 = map(ldr1, 0, 1023, 0, 100);
  lightIntensity2 = map(ldr2, 0, 1023, 0, 100);

  //Get the difference of these values
  int value1 = abs(ldr1 - ldr2);
  int value2 = abs(ldr2 - ldr1);
  Serial.print("Panel Degree: ");
  Serial.println(Spoint);
//Check these values using a IF condition
  if ((value1 <= error) || (value2 <= error)) {

  } else {
    if(Spoint < 130 && Spoint > 30 )
    {
      servo.write(Spoint);
    }

    if (Spoint <= 130 && ldr1 > ldr2) {
      Spoint = Spoint + 12;  
    }
    if (Spoint >= 30 && ldr1 < ldr2) {
      Spoint = Spoint - 12;
    }
  }

  // Serial.print("IOTSENSOR");
  // Serial.print("#");
  // Serial.print(random(0, 10));
  // Serial.print("#");
  // Serial.print(random(10, 20));
  // Serial.print("#");
  // Serial.print(random(20, 30));
  // Serial.print("#");
  // Serial.print(random(30, 40));
  // Serial.print("#");
  // Serial.print(random(40, 50));
  // Serial.print("#");
  // Serial.print(random(50, 60));
  // Serial.print("#");
  // Serial.println(random(60, 70));

  
  Serial.print("IOTDEA");
  Serial.print("#");
  Serial.print(temperature);
  Serial.print("#");
  Serial.print(lightIntensity1);
  Serial.print("#");
  Serial.print(lightIntensity2);
  Serial.print("#");
  Serial.print(Spoint);
  Serial.print("#");
  Serial.print(intensitasHujan);
  Serial.print("#");
  Serial.print(current_mA);
  Serial.print("#");
  Serial.println(loadvoltage);
  delay(1000);
  Serial.println("");
}
