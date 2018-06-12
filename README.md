# g-8266-gast

IOT house automation for **ESP8266 and OpenHAB**

This project uses the **Sonoff DEV board** as HW to control a guest house. 
This is a very simple ESP8266 HW with 5 solid connectors. I recommend to order the connectors wiith the board.  
The ESP8266 communicates with the MQTT server over a distance of 40 m (~ 86dB) !!

* Sonoff DEV board 
* MQTT server with Imroy library
* MQTT messages for OpenHAB
* MySQL logging via nodejs POST 
* Sensors: IR (commercial from Alarms), DHT 21
* Output: Relays for light 
* Light presence simulation
* OTA via MQTT message
* ISR Ticker for Alarm Siren, Light simolator and watchdog.

Developed in PlatformIO. Here the ini

[env:esp01_1m]
platform = espressif8266
board = esp01_1m
framework = arduino
monitor_speed = 115200




