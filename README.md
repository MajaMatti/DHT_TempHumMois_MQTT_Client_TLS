# DHT_TempHumMois_MQTT_Client_TLS

A project combining a Raspberry Pi with a MQTT Broker running, a ESP8622 where this code runs, a DHT Temperature and Humidity Sensor, a well as a Moisture Sensor.
The MQTT connection is set up between the broker (on the RasPi) and the client (the ESP8622 mcu). You can connect a smartphone app to the broker via Internet (if your raspi is online) and send the corresponding control command messages to the broker. The broker will then hand it to the ESP board or you sign for the relevant topics you wanna listen to.
