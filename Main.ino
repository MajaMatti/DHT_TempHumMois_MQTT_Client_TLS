/* MQTT Client ESP8266 with a DHT Temperature and Humidity Sensor
 *  clientID = ESP_1
 *  Author = maja
 *  
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Hash.h>
#include <DHT.h>
#include <SimpleDHT.h>
//es fehlen vielleicht noch libraries

///////////////////////////////////////////////////////////////////////
///                      Definitions                                ///
///////////////////////////////////////////////////////////////////////
#define LED_BUILTIN 2

/* Sensor Pin */
int pinDHT22 = 16;
int pinRelay = 5;

/*MoistureSensor*/
const float AirValue = 730;   
const float WaterValue = 270;
float rangePercentage = (AirValue - WaterValue)/100;
int soilMoistureValue = 0;
float soilMoisturePercentage;
float actualMoisturePercentage;

/* WiFi */
const char* cfg_wifi_ssid = "WiFi-Name";
const char* cfg_wifi_pwd = "WiFi-Password";
/* MQTT */
const char* mqtt_server = "www.majamaja.de";
const unsigned int mqtt_port = 8883;
const char* mqtt_user =   "username";    //broker username
const char* mqtt_pass =   "password";    //broker password
const char* clientID = "ESP_1"; //evtl. nicht gebraucht

//echo | openssl s_client -connect 192.168.178.26:8883 | openssl x509 -fingerprint -noout
//send on raspberry pi bash
//enter the result hash here:
const char* mqtt_fprint = "4F:BD:64:C3:52:FF:4B:F9:61:69:44:A3:93:84:3C:XX:XX:XX:XX:XX";

WiFiClientSecure espClient;
PubSubClient client(espClient);
SimpleDHT22 dht22(pinDHT22);

// temp & hum
float t = 0.0;
float h = 0.0;

// last updeate time
unsigned long previousMillis = 0;    

// Updates DHT readings
const long interval = 1000;  

// Schalter /foo
bool Schalter = false;

// WiFi Reconnect
int wifi_retry = 0;

///////////////////////////////////////////////////////////////////////
///                      void setup                                 ///
///////////////////////////////////////////////////////////////////////

void setup() {


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinRelay, OUTPUT);
  digitalWrite(pinRelay, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

   
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("TestMQTT");

  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg_wifi_ssid, cfg_wifi_pwd);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  espClient.setFingerprint(mqtt_fprint);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    verifyFingerprint();
    if (client.connect(WiFi.macAddress().c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe("/foo");
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

///////////////////////////////////////////////////////////////////////
///                      void callback                              ///
///////////////////////////////////////////////////////////////////////

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
    }
    message[length] = '\0';
    Serial.println(message);


if (strcmp(message,"1")==0){
     digitalWrite(LED_BUILTIN, LOW);
     digitalWrite(pinRelay, LOW); 
    }
else if (strcmp(message,"0")==0){
    digitalWrite(LED_BUILTIN, HIGH); 
    digitalWrite(pinRelay, HIGH);
    }
}

///////////////////////////////////////////////////////////////////////
///                      void verifyFingerprint                     ///
///////////////////////////////////////////////////////////////////////

void verifyFingerprint() {
  if(client.connected() || espClient.connected()) return; //Already connected
  
  Serial.print("Checking TLS @ ");
  Serial.print(mqtt_server);
  Serial.print("...");
  
  if (!espClient.connect(mqtt_server, mqtt_port)) {
    Serial.println("Connection failed. Rebooting.");
    Serial.flush();
    ESP.restart();
  }
  if (espClient.verify(mqtt_fprint, mqtt_server)) {
    Serial.print("Connection secure -> .");
  } else {
    Serial.println("Connection insecure! Rebooting.");
    Serial.flush();
    ESP.restart();
  }

  espClient.stop();

  delay(100);
}

unsigned int counter = 0;


///////////////////////////////////////////////////////////////////////
///                      void loop                                  ///
///////////////////////////////////////////////////////////////////////

void loop() {


// WIFI Reconnect

    while(WiFi.status() != WL_CONNECTED && wifi_retry < 5 ) {
        wifi_retry++;
        Serial.println("WiFi not connected. Try to reconnect");
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg_wifi_ssid, cfg_wifi_pwd);
        delay(100);
    }
    if(wifi_retry >= 5) {
        Serial.println("\nReboot");
        ESP.restart();
    }

  
  soilMoistureValue = analogRead(A0);
  actualMoisturePercentage = (AirValue - soilMoistureValue)/rangePercentage;
  
  client.loop();
  delay(1000);

// Teil 2 neu

  float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;

  if((err=dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess)
  {
  Serial.print("Read DHT22 failed, err=");
  Serial.println(err);
  delay(2000);
  return;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;
    float newT = (float)temperature;

    if (isnan(newT)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      t = newT;
      Serial.println(t);
    }

    float newH = float(humidity);

    if (isnan(newH)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      h = newH;
      Serial.println(h);
    }
  }

    char buffer[10];
  
    dtostrf(t,4, 2, buffer);
 if (client.publish("Temperatur", buffer)){
      Serial.println("temp sent!");
    }

   dtostrf(h,4, 2, buffer);
 if (client.publish("Humidity", buffer)){
      Serial.println("humidity sent!");
    }

   dtostrf(actualMoisturePercentage,4, 2, buffer);
 if (client.publish("Moisture", buffer)){
      Serial.println("moisture sent!");
    }


}
