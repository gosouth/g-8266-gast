/*----------------------------------------------------------------------

Quilaco Gast - Home automation for quilaco
Implemented with Atom IDE 1.24

pubsubclient : https://github.com/Imroy/pubsubclient
ntpclient : https://github.com/arduino-libraries/NTPClient

[env:nodemcuv2]
env_default = nodemcuv2
platform = https://github.com/platformio/platform-espressif8266.git#feature/stage
board = nodemcuv2
framework = arduino

mosquitto_pub -h 192.168.10.124 -t '/pucon/gast/ota' -r -f firmware.bin

~/Documents/PlatformIO/Projects/q-8266 gast/.pioenvs/esp01_1m

sftp> put firmware.bin          // 292384 ?
sftp {user}@{host}:{remote_dir} <<< $'put {local_file_path}'

-------------------------------------------------------------------------

Ricardo Timmermann

2 Mai 18    : Sonoff DEV board
12 Apr 18   : Log 2 DB
16 May 18   : mqtt OTA and a lot of refactoring

-------------------------------------------------------------------------*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>           // https://github.com/Imroy/pubsubclient
#include <Ticker.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// #define _DEBUG                      // comment to use TX / RX as io pins

const char *ssid =  "Quilaco";      // cannot be longer than 32 characters!
const char *pass =  "";             // WiFi password

WiFiClient wifi;

#define OFF 0
#define ON 1

// == MQTT server ======================================

String mqttServerName = "192.168.10.124";
int    mqttport = 1883;
String mqttuser =  "";              // from CloudMQTT account data
String mqttpass =  "";              // from CloudMQTT account data

PubSubClient mqttClient( wifi, mqttServerName, mqttport );
void callback( const MQTT::Publish& sub );

void autoLicht( void );             // light precense emulation
String LogStatus( String );
void log2DB( String );

// == NTP time ====================================================

#define NTP_OFFSET   -4 * 60 * 60       // In seconds for Chile time
#define NTP_INTERVAL 60 * 1000          // In miliseconds
#define NTP_ADDRESS  "cl.pool.ntp.org"  // "europe.pool.ntp.org"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// == devices ============================================

String sDeviceID =      "GAST";
String sFeed =          "/pucon/gast/";

#define S_OTA           "ota"                   // OTA update
#define S_STATE         "/state"

#define T_DHTGAST       "DHT-gast/sensor"       // DB log hum > 70
#define T_IRZIMMER      "IR-room/sensor"
#define T_IRBODEGA      "IR-bodega/sensor"      // DB log
#define T_TIMBRE        "timbre/sensor"         // DB log
#define T_SIGNAL        "signal/sensor"         // tmp DB log

#define T_PATIOLICHT    "patiolicht"
#define T_RELAY         "relay"
#define T_HUPE          "hupe"

#define T_LSIMULATOR    "LightSimulator"

// == HW GPIO defines =====================================================

#define LED     13

#define IO_TIMBRE       4
#define IO_IRZIMMER     12
#define IO_RELAY        14
#define IO_HUPE         LED
#define IO_DHT          5

#define IO_PATIOLICHT   1       // TX
#define IO_IRBODEGA     3       // RX

// == extended only to simplify DEUG calls =========================

void turnPatioLicht( bool mode )
{
#ifndef _DEBUG
    digitalWrite( IO_PATIOLICHT, mode );
#endif
}

// == DHT 21 Sonoff Sensor plug insane pins =================================
//
//  GRN : GND
//  BLK : 3.3V
//  WHI : SIG

#define DHTTYPE         DHT21         // DHT 21 (AM2301)
#define DHT_PIN         IO_DHT

DHT_Unified dht(DHT_PIN, DHTTYPE);

float temperature = 0;
float humidity = 0, prevHumidity = 0;

/*--------------------------------------------------------------------------
;
;   watchDog settings, uses Ticker.h
;
;---------------------------------------------------------------------------*/

#define TM_ISR 5

Ticker secondTick;
static int watchdogCount = 0;

void ISRwatchdog()
{
    ++watchdogCount;
    if( watchdogCount == 10 ) {
        Serial.printf("\n\n== Watchdog timeout ==\n\n");
        ESP.restart();
    }
}

/*--------------------------------------------------------------------------
;
;   hupe alarm, uses Ticker ISR
;
;---------------------------------------------------------------------------*/

#define TM_HUPE 1

Ticker hupeTick;
static int hupeCount = 0;
static int hupeMode = OFF;

void hupeAlarm()
{
    ++hupeCount;

    switch( hupeMode ) {

        case 0:
            hupeCount=0;
            digitalWrite( IO_HUPE, LOW );
            break;

        case 1:
            if( hupeCount < 4 ) digitalWrite( IO_HUPE, HIGH );
            else {
                digitalWrite( IO_HUPE, LOW );
                hupeCount=0;
                hupeMode = OFF;
                }
            break;

        default:
            if( hupeCount % hupeMode ) digitalWrite( IO_HUPE, HIGH );
            else digitalWrite( IO_HUPE, LOW );
            break;
    }
}

/*--------------------------------------------------------------------------------
;
;   light precense emulation, internet independ function, uses Ticker.h
;
;--------------------------------------------------------------------------------*/

#define TM_LSIMUL  (10*60)        // 10*60

Ticker minuteTick;
static unsigned long tm_MinuteCnt = 0;
static bool simulatorMode = 0;

// == array to tell the relay ON / OFF in lapses of 10 minutes.

uint8_t lightState[] = {

                2,3,1,1,0,1,        // 0    20:00 PM
                1,3,2,1,1,1,        //      21:00
                3,1,1,1,0,1,        // 12   22:00
                1,0,1,1,0,0,        //      23:00
                0,0,0,0,0,0,        // 24   24:00
                0,0,0,1,0,0,        //      1:00 AM
                0,0,0,0,0,0,        // 36   2:00
                0,0,1,0,0,0,        //      3:00
                0,0,0,0,0,0,        // 48   4:00
                0,0,0,0,0,0,        //      5:00
                1,0,0,3,1,0 };      // 60   6:00 AM

void LightSimulator( void )
{
    int lumi;
    lumi = analogRead( A0 );

    // -- day mode --------------
    if( lumi>40 ) {
        tm_MinuteCnt=0;
        digitalWrite( IO_RELAY, LOW );
        turnPatioLicht( LOW );
        return;
        }

    // == night mode ==============
    if( simulatorMode == ON ) {
        digitalWrite( IO_RELAY, lightState[tm_MinuteCnt] & 1 ? HIGH : LOW );
        turnPatioLicht( lightState[tm_MinuteCnt] & 2 ? HIGH : LOW );
    }

    if( tm_MinuteCnt < sizeof(lightState)-1 )
        ++tm_MinuteCnt;
}

/*--------------------------------------------------------------------------------
;
;   time stamp string for logs
;
;--------------------------------------------------------------------------------*/

String LogStatus( String log )
{

    timeClient.update();

    String formattedTime = timeClient.getFormattedDate();
    formattedTime += " ";
    formattedTime += timeClient.getFormattedTime();
    formattedTime += " : ";

//    Serial.println( formattedTime + log );

    return formattedTime + log;

}
/*--------------------------------------------------------------------------------
;
;   Log to mySql DB
;
;--------------------------------------------------------------------------------*/

WiFiClient client;

void log2DB( String sLog )
{

//    message=+It+is+working&devID=GAST

    String sMsg = "devID=" + sDeviceID + "&message=";
    sMsg += sLog;

//    Serial.println( sMsg);

    if( client.connect( "192.168.10.124", 8000 ) ) {

//        Serial.println("--> Connected to server");

        client.println("POST /api/records HTTP/1.1");
        client.println("Host: 192.168.10.124:8000");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.println("Cache-Control: no-cache");

        client.println("Content-Length: " + String(sMsg.length()));

        client.println();
        client.println( sMsg );
        client.stop();
    }

    else
        Serial.println("--> connection failed");
}

/*--------------------------------------------------------------------------------
;
;   publish message
;
;--------------------------------------------------------------------------------*/

void pubMessage( String sTopic, String sPayload )
{
    Serial.println(" p>> " + sFeed + sTopic + "  :  " + sPayload );
    mqttClient.publish( sFeed + sTopic, sPayload );
}

/*--------------------------------------------------------------------------------
;
;   Pub message in JSon format
;
;--------------------------------------------------------------------------------*/

#define MAXBUF 80
char sJson[MAXBUF];

String pubJsonMessage( String sTopic, String sArg1, String sVal1, String sArg2, String sVal2 )
{
    snprintf( sJson, MAXBUF, "{ \"%s\":%s, \"%s\":%s }", sArg1.c_str(), sVal1.c_str(), sArg2.c_str(), sVal2.c_str() );
    pubMessage( sTopic, sJson );
    return String( sJson );
}

String pubJsonMessage( String sTopic, String sArg1, int nVal1, String sArg2, int nVal2 )
{
    return pubJsonMessage( sTopic, sArg1, String(nVal1), sArg2, String(nVal2) );
}

/*--------------------------------------------------------------------------------
;
;   connect to wifi if not connected
;
;--------------------------------------------------------------------------------*/

void setup_WiFi( String sLog )
{
    if( WiFi.status() != WL_CONNECTED ) {
        Serial.print("Connecting WiFi : ");
        Serial.print(ssid);
        Serial.print("...");

        WiFi.begin(ssid, pass);

        if (WiFi.waitForConnectResult() != WL_CONNECTED) return;

        Serial.println( "Success" );
        Serial.print( "IP address: " );
        Serial.println( WiFi.localIP() );
        Serial.println("");

        log2DB( sLog );
        }
}

/*--------------------------------------------------------------------------------
;
;   reconnect to MQTT broker
;
;--------------------------------------------------------------------------------*/

void reconnect_Broker( void )
{

    Serial.print( "Connecting to MQTT server ... " );
    bool success;

    String clientId = sDeviceID + "-"+ WiFi.macAddress();

    if( mqttuser.length() > 0 )
        success = mqttClient.connect( MQTT::Connect( clientId ).set_auth(mqttuser, mqttpass) );
    else success = mqttClient.connect( clientId );

    // -- subscribe for what ever ... ----------------------------

    if (success) {
        Serial.println( "OKI" );

        mqttClient.set_callback( callback );
        mqttClient.subscribe( sFeed + "+/control" );
        mqttClient.subscribe( sFeed + S_OTA );

        log2DB( "MQTT connected" );
        }

    else {
        Serial.println( "MQTT connection FAIL" );
        delay(1000);
        }
}

/*--------------------------------------------------------------------------------
;
;   publish current gpio-out state, this for openHAB feedback
;
;--------------------------------------------------------------------------------*/

#define TM_PUBSTATE (1000*30)            // 30 sec min : State of publish
static unsigned long lastStatusTime;

void publishPayloadState( void )
{
    if ( (millis() - lastStatusTime) > TM_PUBSTATE ) {

        // Serial.printf( "Hupe = %d, Relay=%d, Patio=%d\n", digitalRead(IO_HUPE), digitalRead(IO_RELAY), digitalRead(IO_PATIOLICHT) );

        pubMessage( T_HUPE S_STATE, digitalRead(IO_HUPE) ? "ON" : "OFF" );
        pubMessage( T_RELAY S_STATE, digitalRead(IO_RELAY) ? "ON" : "OFF" );
        pubMessage( T_PATIOLICHT S_STATE, digitalRead(IO_PATIOLICHT) ? "ON" : "OFF" );
        pubMessage( T_LSIMULATOR S_STATE, simulatorMode ? "ON" : "OFF" );

        lastStatusTime = millis();
        }
}

/*--------------------------------------------------------------------------------
;
;   ESP8266 Setup
;
;--------------------------------------------------------------------------------*/

void setup()
{

    Serial.begin(115200);                       // for debugging
    while (!Serial);                            // wait for serial port to connect

    // == wifi mode

    WiFi.mode( WIFI_STA );
    WiFi.disconnect();

    dht.begin();

    Serial.println("");
    Serial.println("==== Q-ESP8266 Gast ==== ");
    Serial.println( __DATE__ " " __TIME__ );

    simulatorMode = ON;
    hupeMode = OFF;

    // -- setup gpio's

    #ifndef _DEBUG
        Serial.println("===> Swap mode");
        delay(100);
        Serial.swap();          // turn serial off, we need TX & RX as gpio

        delay(500);
        pinMode(IO_PATIOLICHT, OUTPUT);
        pinMode(IO_IRBODEGA, INPUT_PULLUP);
    #endif

    pinMode(IO_HUPE, OUTPUT);
    pinMode(IO_RELAY, OUTPUT);

    digitalWrite(IO_HUPE,LOW);
    digitalWrite(IO_RELAY,LOW);

    pinMode(IO_TIMBRE, INPUT_PULLUP);
    pinMode(IO_IRZIMMER, INPUT_PULLUP);

    // attach time interrupt services
    secondTick.attach( TM_ISR, ISRwatchdog );
    minuteTick.attach( TM_LSIMUL, LightSimulator );
    hupeTick.attach( TM_HUPE, hupeAlarm );

    Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
    Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());

    setup_WiFi( ">>> Q-8266 Gast Started <<<" );
    log2DB( "Boot ver : " __DATE__ " " __TIME__ );
}

/*--------------------------------------------------------------------------------
;
;   main() loop
;
;--------------------------------------------------------------------------------*/

#define TM_LOOP (1000*60*1)                 // 1  min : loop
#define TM_IR (1000*60*15)                  // 15 min : IR
#define TM_PRESENCE (1000*60*60)            // 60 min : presence

static unsigned long tm_loop;

static unsigned long tm_timbre = 0;
static unsigned long tm_IRzimmer= 0;
static unsigned long tm_IRbodega= 0;

long lastDebounceTime = 0;  // the last time the output pin was toggled
bool TimbreState = OFF;

void loop()
{
    watchdogCount = 0;      // reset watchdog to signal keepalife

    setup_WiFi( "Wifi Connected" );

    // == connect to MQTT broker if not connected =====================================

    if ( !mqttClient.connected() ) {
        reconnect_Broker();
        return;
        }

    mqttClient.loop();
    yield();

    publishPayloadState();                  // for feedback in openHUP

    // == read Presence key and debounce ========================================

    int sw = digitalRead( IO_TIMBRE );

    // filter out any noise by setting a time buffer
    if ( (millis() - lastDebounceTime) > 200 ) {

        if( sw == LOW ) {
            //Serial.printf( "Timbre !!!" );
            lastDebounceTime = millis(); //set the current time
            TimbreState = OFF;
            }

        else {
            lastDebounceTime = millis(); //set the current time
            TimbreState = ON;
            }
        }

    // -- timbre - food data logger ----------------------------------

    if( millis() > tm_timbre ) {

        if( TimbreState==OFF ) {
            tm_timbre = millis() + TM_PRESENCE;
            pubMessage( T_TIMBRE, LogStatus("Precense tag") );
            log2DB( "Precense tag" );
            }
        }

    // == IR Zimmer =========================================================

    int IRstate;

    if( millis() > tm_IRzimmer ) {

        IRstate = digitalRead( IO_IRZIMMER );

        if( IRstate==1 ) {
            tm_IRzimmer = millis() + TM_IR;
            pubMessage( T_IRZIMMER, LogStatus("IR Zimmer") );
            //log2DB( "IR Zimmer" );
            }
        }

    // == IR Bodega =========================================================

    #ifndef _DEBUG

    if( millis() > tm_IRbodega ) {

        IRstate = digitalRead( IO_IRBODEGA );

        if( IRstate==1 ) {
            tm_IRbodega = millis() + TM_IR;
            pubMessage( T_IRBODEGA, LogStatus("IR Bodega") );
            log2DB( "IR Bodega" );
            }
        }

    #endif

    // == continue only each TM_LOOPs =========================================

    if ( millis() - tm_loop < TM_LOOP) return;
    tm_loop = millis();

    int lumi;
    lumi = analogRead( 0 );         // lumi

    // == RSSI signal =======================================================

    int32_t rssi = WiFi.RSSI();
//    pubJsonMessage( T_SIGNAL, "Lumi", lumi, "Signal dBm", rssi );

    snprintf( sJson, MAXBUF, "{ \"Lumi\":%d, \"Scnt\":%d, \"Signal dBm\":%d }", lumi, tm_MinuteCnt, rssi );
    pubMessage( T_SIGNAL, sJson );

    // == DHT Gast =========================================================

    sensors_event_t event;

    dht.temperature().getEvent(&event);
    temperature = event.temperature;

    dht.humidity().getEvent(&event);
    humidity = event.relative_humidity;

    snprintf( sJson, MAXBUF, "{ \"Temp\":%.1f, \"Hum\":%.1f }", temperature, humidity );
    pubMessage( T_DHTGAST, sJson );

    // log only if gap > 3%
    if( humidity>78 && (humidity-prevHumidity) > 2 ) {
        prevHumidity = humidity;
        log2DB( sJson );
    }
}

/*--------------------------------------------------------------------------------
;
;   subscription: get messages from OpenHAB
;
;   /pucon/gast/relay/control 0/1
;
;--------------------------------------------------------------------------------*/

void callback( const MQTT::Publish& sub )
{

    // -- OTA Update -----------------------------------

    if( strstr( sub.topic().c_str(), S_OTA )>0 ) {

        Serial.println( sub.topic().c_str() );

        uint32_t startTime = millis();
        uint32_t size = sub.payload_len();
        if (size == 0) return;

        snprintf( sJson, MAXBUF, "Receiving OTA of %d bytes", size );
        log2DB( sJson );

        Serial.println( sJson );

        //Serial.setDebugOutput( true );

        if (ESP.updateSketch( *sub.payload_stream(), size, true, false )) {

            mqttClient.unset_callback();
            mqttClient.unsubscribe( sFeed + S_OTA );

            Serial.println( "Clearing retained message" );
            mqttClient.publish(MQTT::Publish(sub.topic(), "").set_retain());
            delay(2000);
            mqttClient.publish(MQTT::Publish(sub.topic(), "").set_retain());
            delay(1000);
            pubMessage( S_OTA, "" );
            delay(1000);
            pubMessage( S_OTA, "" );

//            mqttClient.disconnect();

            snprintf( sJson, MAXBUF, "OTA Firmware updated: %d bytes", size );
            log2DB( sJson );

            delay( 2000 );

            Serial.printf("Update Success: %u\nRebooting...\n", millis() - startTime);

            ESP.restart();
            delay( 10000 );
            }

        log2DB( "OTA Update FAIL" );

//        Update.printError(Serial);
//        Serial.setDebugOutput(false);

        return;
        }

    // -- subscribe from widgets ---------------

    Serial.print( "s<< ");
    Serial.print( sub.topic() );
    Serial.print(" <= ");
    Serial.println( sub.payload_string() ) ;

    // -- Light Simulator mode -----------------------------

    if( strstr( sub.topic().c_str(), T_LSIMULATOR )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_LSIMULATOR S_STATE, "OFF" );
            simulatorMode = OFF;
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_LSIMULATOR S_STATE, "ON" );
            simulatorMode = ON;
            }
        }

    // -- Patio Licht ----------------------------------

    if( strstr( sub.topic().c_str(), T_PATIOLICHT )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_PATIOLICHT S_STATE, "OFF" );
            turnPatioLicht( LOW );
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_PATIOLICHT S_STATE, "ON" );
            turnPatioLicht( HIGH );
            }
        }

     // -- Hupe ----------------------------------

     if( strstr( sub.topic().c_str(), T_HUPE )>0 ) {

         String sMode = sub.payload_string();
         hupeMode = sMode.toInt();
         }

     // -- Relay enchufe zimmer ---------------------

     if( strstr( sub.topic().c_str(), T_RELAY )>0 ) {

         if (sub.payload_string() == "0") {
             pubMessage( T_RELAY S_STATE, "OFF" );
             digitalWrite(IO_RELAY,LOW);
             }
         else if (sub.payload_string() == "1") {
             pubMessage( T_RELAY S_STATE, "ON" );
             digitalWrite(IO_RELAY,HIGH);
             }
         }
}
