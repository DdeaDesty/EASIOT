#include <WiFi.h>
#include <ThingsBoard.h>
#include <Server_Side_RPC.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino_MQTT_Client.h>

constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "setFanValue";
constexpr const char RPC_SWITCH_KEY[] = "fanValue";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;
constexpr uint16_t MAX_MESSAGE_SIZE = 256U;

const char* ssid = "vivo Y21s";
const char* pass = "00000000";

const char* mqtt_server = "demo.thingsboard.io";
const char* token = "LHNJm3lGrGuRFvrsI7vN";

WiFiClient espClient;
PubSubClient client(espClient);
Arduino_MQTT_Client mqttClient(espClient);
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_SUBSCRIPTIONS> rpc;
const std::array<IAPI_Implementation*, 1U> apis = { &rpc };

bool subscribed = false;

ThingsBoard tb(mqttClient);
int led_state = 0;  // Status awal LED
// Timer variables
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval for reading data (e.g., every 10 seconds)

void processGetJson(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the json RPC method");
}

void processButtonChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set switch method");

  // Process data
  led_state = data;

  Serial.print("Switch state change: ");
  Serial.println(led_state);

  // Mengatur warna LED sesuai dengan led_state
  if (led_state == 1) {
//    setLEDColor(0, 255, 0);  // Hijau
    Serial.println("LED Color: Green");
  } else {
//    setLEDColor(0, 0, 0);  // Merah
    Serial.println("LED Color: Red");
  }
  
  response.set(22.02);
}

void processSwitchChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set switch method");

  // Process data
  led_state = data["fanValue"];

  Serial.print("Switch state change: ");
  Serial.println(led_state);
  
  // Mengatur warna LED sesuai dengan led_state
  if (led_state == 1) {
//    setLEDColor(0, 255, 0);  // Hijau
    Serial.println("LED Color: Green");
  } else {
//    setLEDColor(255, 0, 0);  // Merah
    Serial.println("LED Color: Red");
  }
  
  response.set(22.02);
}

void processAllSensorChange(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set temperature RPC method");

  // Process data
//  const double example_temperature = data[RPC_TEMPERATURE_KEY];
  // temperature = data["tempValue"]; 
  // humidity = data["humidityValue"]; 
  // distance = data["distanceValue"]; 
  // ph = data["phValue"]; 
  // turbidity = data["turbidityValue"]; 
  
  // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC template parameter accordingly to ensure that the value can be deserialized.RPC_Callback.
  // See https://arduinojson.org/v6/api/jsondocument/add/ for more information on which variables cause a copy to be created
  response["string"] = "exampleResponseString";
  response["int"] = 5;
  response["float"] = 5.0f;
  response["double"] = 10.0;
  response["bool"] = true;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", token, "")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(1000);
    }
  }
}

String VAR1, VAR2, VAR3, VAR4, VAR5, VAR6, VAR7;

void setup() {
  Serial.begin(115200); // Starts the serial communication

  // Hubungkan ke Wi-Fi  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  client.setServer(mqtt_server, 1883);
//  strip.begin();       // Inisialisasi library NeoPixel
//  strip.show();        // Pastikan LED dimatikan awalnya
  
  // Hubungkan ke ThingsBoard
  tb.connect(mqtt_server, token);
  }
  
void loop() {

  if(Serial.available() > 0){

    String data = Serial.readStringUntil('\n');

    if (data.startsWith("IOTDEA#")) {
        Serial.println(data);
        // Hapus prefix "MCMAX#"
        data = data.substring(10);
    
        // Pisahkan data berdasarkan tanda '#'
        int firstHash = data.indexOf('#');
        int secondHash = data.indexOf('#', firstHash + 1);
        int thirdHash = data.indexOf('#', secondHash + 1);
        int fourHash = data.indexOf('#', thirdHash + 1);
        int fiveHash = data.indexOf('#', fourHash + 1);
        int sixHash = data.indexOf('#', fiveHash + 1);
        
        String var1Str = data.substring(0, firstHash);
        String var2Str = data.substring(firstHash + 1, secondHash);
        String var3Str = data.substring(secondHash + 1, thirdHash);
        String var4Str = data.substring(thirdHash + 1, fourHash);
        String var5Str = data.substring(fourHash + 1, fiveHash);
        String var6Str = data.substring(fiveHash + 1, sixHash);
        String var7Str = data.substring(sixHash + 1);
        
        // Mengisi variabel VAR1, VAR2, VAR3
        VAR1 = var1Str;
        VAR2 = var2Str;
        VAR3 = var3Str;
        VAR4 = var4Str;
        VAR5 = var5Str;
        VAR6 = var6Str;
        VAR7 = var7Str;
        

        // Untuk verifikasi, cetak nilai VAR1, VAR2, VAR3
        Serial.print("Temperature: ");
        Serial.println(VAR1);
        Serial.print("Light Intensity1: ");
        Serial.println(VAR2);
        Serial.print("Light Intensity2: ");
        Serial.println(VAR3);
        Serial.print("PanelDegree: ");
        Serial.println(VAR4);
        Serial.print("Rain Intensity: ");
        Serial.println(VAR5);
        Serial.print("Current: ");
        Serial.println(VAR6);
        Serial.print("Voltage: ");
        Serial.println(VAR7);
      }
  }else{
    Serial.println("Data Sensor tidak diterima");
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Create payload for ThingsBoard
  String payload = "{";
  payload += "\"temperatureValue\":" + String(VAR1) + ",";
  payload += "\"lightIntensity1Value\":" + String(VAR2) + ",";
  payload += "\"lightIntensity2Value\":" + String(VAR3) + ",";
  payload += "\"panelDegreeValue\":" + String(VAR4) + ",";
  payload += "\"rainIntensityValue\":" + String(VAR5) + ",";
  payload += "\"currentValue\":" + String(VAR6) + ",";
  payload += "\"voltageValue\":" + String(VAR7);
  payload += "}";

  // payload += "\"temperatureValue\":" + String(random(0, 100)) + ",";
  // payload += "\"humidityValue\":" + String(random(0, 100)) + ",";
  // payload += "\"distanceValue\":" + String(random(0, 100)) + ",";
  // payload += "\"angleXValue\":" + String(random(0, 100)) + ",";
  // payload += "\"angleYValue\":" + String(random(0, 100)) + ",";
  // payload += "\"currentValue\":" + String(random(0, 100)) + ",";
  // payload += "\"voltageValue\":" + String(random(0, 100));
  // payload += "}";

  // Publish data to ThingsBoard
  client.publish("v1/devices/me/telemetry", payload.c_str());
  delay(1000);

  // Pastikan tetap terhubung ke ThingsBoard
  if (!tb.connected()) {
    // Reconnect to the ThingsBoard server,
    // if a connection was disrupted or has not yet been established
    Serial.printf("Connecting to: (%s) with token (%s)\n", mqtt_server, token);
    if (!tb.connect(mqtt_server, token, 1883U)) {
      Serial.println("Failed to connect");
      return;
    }
  }


  // Subscribe ke RPC ThingsBoard
  if (!subscribed) {
    Serial.println("Subscribing for RPC...");
    const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
       // Internal size can be 0, because if we use the JsonDocument as a JsonVariant and then set the value we do not require additional memory
      RPC_Callback{"setLedButtonValue",         processButtonChange },
      RPC_Callback{"setFanValue",               processSwitchChange },
      RPC_Callback{"setAllSensor",              processAllSensorChange }
    };
    // Perform a subscription. All consequent data processing will happen in
    // processTemperatureChange() and processSwitchChange() functions,
    // as denoted by callbacks array.
    if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
    
    Serial.println("Subscribe done");
    subscribed = true;
  }
  
  tb.loop();
  delay(1000);
}
