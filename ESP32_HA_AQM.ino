/**************************************************************************************************************************************************
* File name     : ESP32_HA_AQM.c
* Compiler      : 
* Autor         : VIGASAN   
* Created       : 05/02/2023
* Modified      : 
* Last modified :
*
*
* Description   : 
*
* Other info    : Air Quality Monitor for Home Assistant
**************************************************************************************************************************************************/

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Include Files----------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DFRobot_OxygenSensor.h>
#include <DFRobot_OzoneSensor.h>

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Constants--------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
#define COLLECT_NUMBER          20    // Collection Range for Oxygen and Ozone sensors
#define LED                     4


/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Configuration --------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
const char*         g_ssid = "your Wifi Name";                              // Wifi Name
const char*         g_password = "Wifi Password";                           // Wifi Password
const char*         g_mqtt_server = "192.168.1.25";                         // MQTT Server IP, same of Home Assistant
const char*         g_mqttUser = "mqttUser";                                // MQTT Server User Name
const char*         g_mqttPsw = "password";                                 // MQTT Server password
int                 g_mqttPort = 1883;                                      // MQTT Server Port

// Variable used for MQTT Discovery
const char*         g_deviceModel = "AQM Vigasan";                            // Hardware Model
const char*         g_swVersion = "1.0";                                      // Firmware Version
const char*         g_manufacturer = "Vigasan";                               // Manufacturer Name
String              g_deviceName = "AQM_Room";                                // Device Name
String              g_mqttStatusTopic = "esp32iotsensor/" + g_deviceName;     // MQTT Topic

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------Public variables-------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
WiFiClient          g_WiFiClient;
PubSubClient        g_mqttPubSub(g_WiFiClient);
DFRobot_OxygenSensor g_OxygenSensor;
DFRobot_OzoneSensor g_OzoneSensor;
unsigned char       co2dataReq[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
unsigned int        PMSa = 0,FMHDSa = 0,TPSa = 0,HDSa = 0,PMSb = 0,FMHDSb = 0,TPSb = 0,HDSb = 0;
unsigned int        PMS = 0,FMHDS = 0,TPS = 0,HDS = 0,CR1 = 0,CR2 = 0;
unsigned char       buffer_RTT[40]={};   //Serial buffer; Received Data
unsigned long       g_ElapsedTimeI2C = 0;
unsigned long       g_ElapsedTimeCO2 = 0;
unsigned long       g_ElapsedTimeSendData = 0;
float               g_Temperature = 0.0;
float               g_Humidity = 0.0;
float               g_O2 = 0;
int16_t             g_O3 = 0;
long                g_CO2 = 0;
String              g_UniqueId;
bool                g_InitSystem = true;
int                 g_mqttCounterConn = 0;
char col;

void setup() 
{

    Serial.begin(9600);  // PM2.5, Formaldeyde, Temperature and Humidity Sensor
    Serial2.begin(9600); // CO2 Sensor
    delay(500);

    pinMode(LED, OUTPUT);

    Serial.println("");
    Serial.println("");
    Serial.println("----------------------------------------------");
    Serial.print("MODEL: ");
    Serial.println(g_deviceModel);
    Serial.print("DEVICE: ");
    Serial.println(g_deviceName);
    Serial.print("SW Rev: ");
    Serial.println(g_swVersion);
    Serial.println("----------------------------------------------");

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Wifi Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    setup_wifi();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    g_mqttPubSub.setServer(g_mqtt_server, g_mqttPort);
    g_mqttPubSub.setCallback(MqttReceiverCallback);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Oxygen Sensor Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int count = 0;
    while(!g_OxygenSensor.begin(ADDRESS_2) && count++ < 3) 
    {
        Serial.println("Oxygen Connection Error !");
        delay(1000);
    }
    Serial.println("Oxygen Sensor Connected !!");


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Ozone Sensor Init
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    count = 0;
    while(!g_OzoneSensor.begin(ADDRESS_3) && count++ < 3) 
    {
        Serial.println("Ozone Connection Error !");
        delay(1000);
    }  
    Serial.println("Ozone Sensor Connected !");
    g_OzoneSensor.SetModes(MEASURE_MODE_AUTOMATIC);
    delay(500);

    digitalWrite(LED, LOW);
}

void loop() 
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MQTT Connection
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(WiFi.status() == WL_CONNECTED)
    {
        if(!g_mqttPubSub.connected())
            MqttReconnect();
        else
            g_mqttPubSub.loop();
    }

    if(g_InitSystem)
    {
        delay(100);
        g_InitSystem = false;
        Serial.println("INIT SYSTEM...");
        MqttHomeAssistantDiscovery();     // Send Discovery Data
    }
    
    if(millis() - g_ElapsedTimeI2C > 3000)
    {
        g_ElapsedTimeI2C = millis();

        ////////////////////////////////////////////////////////////////
        // Oxygen Sensor
        ////////////////////////////////////////////////////////////////
        g_O2 = g_OxygenSensor.getOxygenData(COLLECT_NUMBER);

        ////////////////////////////////////////////////////////////////
        // Ozone Sensor
        ////////////////////////////////////////////////////////////////
        g_O3 = g_OzoneSensor.ReadOzoneData(COLLECT_NUMBER);
    }


    ////////////////////////////////////////////////////////////////
    // CO2 Sensor
    ////////////////////////////////////////////////////////////////
    if(millis() > g_ElapsedTimeCO2 + 1000)
    {
        g_ElapsedTimeCO2 = millis();
        Serial2.flush();
        Serial2.write(co2dataReq, 9);
    }

    if(millis() > g_ElapsedTimeCO2 + 300)
    {
        for(int i = 0, j = 0; i < 9; i++)
        {
            if (Serial2.available() > 0)
            {
                long hi, lo;
                int ch = Serial2.read();
                
                if(i == 2)     
                    hi = ch;   // High byte concentration
                
                if(i == 3)
                    lo = ch;   // Low byte concentration
    
                if(i == 8) 
                {
                    g_CO2 = (hi * 256) + lo;  //CO2 concentration
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    // Temperature Humidity Formaldeyde PM2.5 Sensor
    ////////////////////////////////////////////////////////////////
    while(Serial.available() > 0)   //Data check: weather there is any Data in Serial
    {
        for(int i = 0; i < 40; i++)
        {
            col = Serial.read();
            buffer_RTT[i] = (char)col;
            delay(2);
        }

        Serial.flush();

        CR1 =(buffer_RTT[38] << 8) + buffer_RTT[39];
        CR2 = 0;
        for(int i = 0; i < 38; i++)
            CR2 += buffer_RTT[i];
            
        if(CR1 == CR2)                  // Check
        {
            PMSa=buffer_RTT[12];        // Read PM2.5 High 8-bit
            PMSb=buffer_RTT[13];        // Read PM2.5 Low 8-bit
            PMS=(PMSa<<8)+PMSb;         // PM2.5 value
            FMHDSa=buffer_RTT[28];      // Read Formaldehyde High 8-bit
            FMHDSb=buffer_RTT[29];      // Read Formaldehyde Low 8-bit
            FMHDS=(FMHDSa<<8)+FMHDSb;   // Formaldehyde value
            TPSa=buffer_RTT[30];        // Read Temperature High 8-bit
            TPSb=buffer_RTT[31];        // Read Temperature Low 8-bit
            TPS=(TPSa<<8)+TPSb;         // Temperature value
            HDSa=buffer_RTT[32];        // Read Humidity High 8-bit
            HDSb=buffer_RTT[33];        // Read Humidity Low 8-bit
            HDS=(HDSa<<8)+HDSb;         // Humidity value
        } else
        {
            PMS = 0;
            FMHDS = 0;
            TPS = 0;
            HDS = 0;
        }
    }

   ////////////////////////////////////////////////////////////////
    // Print and Save Measured Data
    ////////////////////////////////////////////////////////////////
    if(millis() - g_ElapsedTimeSendData > 60000)
    {
        g_ElapsedTimeSendData = millis();
/*
        if(g_deviceConnected) 
        {
            byte dataToSend = 0;
            g_OutputBuffer[dataToSend++] = (TPS & 0xFF00) >> 8;
            g_OutputBuffer[dataToSend++] = (TPS & 0x00FF);
            g_OutputBuffer[dataToSend++] = (HDS & 0xFF00) >> 8;
            g_OutputBuffer[dataToSend++] = (HDS & 0x00FF);
            g_OutputBuffer[dataToSend++] = (PMS & 0xFF00) >> 8;
            g_OutputBuffer[dataToSend++] = (PMS & 0x00FF);
            g_OutputBuffer[dataToSend++] = (FMHDS & 0xFF00) >> 8;
            g_OutputBuffer[dataToSend++] = (FMHDS & 0x00FF);
            g_OutputBuffer[dataToSend++] = (g_O3 & 0xFF00) >> 8;
            g_OutputBuffer[dataToSend++] = (g_O3 & 0x00FF);
            g_OutputBuffer[dataToSend++] = (g_CO2 & 0xFF000000) >> 24;
            g_OutputBuffer[dataToSend++] = (g_CO2 & 0x00FF0000) >> 16;
            g_OutputBuffer[dataToSend++] = (g_CO2 & 0x0000FF00) >> 8;
            g_OutputBuffer[dataToSend++] = (g_CO2 & 0x000000FF);

            g_pTxCharacteristic->setValue(g_OutputBuffer, dataToSend);
            g_pTxCharacteristic->notify();
        }
*/
        g_Temperature = (float)(TPS) / 10.0;
        g_Humidity = (float)(HDS) / 10.0;

        Serial.println("----------- Data Measured -----------");

        Serial.print("Oxygen Concentration: ");
        Serial.print(g_O2);
        Serial.println(" %");
        
        Serial.print("Ozone Concentration: ");
        Serial.print(g_O3);
        Serial.println(" ppb");
        
        Serial.print("Temperature: ");
        Serial.print(g_Temperature);
        Serial.println(" °C");

        Serial.print("Humidity: ");
        Serial.print(g_Humidity);
        Serial.println(" %");

        Serial.print("PM2.5: ");
        Serial.print(PMS);
        Serial.println(" ug/m3");

        Serial.print("Formaldehyde: ");
        Serial.print(FMHDS);
        Serial.println(" ug/m3");

        Serial.print("CO2 concentration: ");
        Serial.print(g_CO2);
        Serial.println(" ppm");

        Serial.println("  ");

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SEND MQTT DATA
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

        StaticJsonDocument<300> payload;  
        payload["temp"] = g_Temperature;
        payload["hum"] = g_Humidity;
        payload["oxy"] = Round2(g_O2);
        payload["ozn"] = g_O3;
        payload["pm25"] = PMS;
        payload["frm"] = FMHDS;
        payload["co2"] = g_CO2;
        

        String strPayload;
        serializeJson(payload, strPayload);

        if(g_mqttPubSub.connected())
        {
            digitalWrite(LED, HIGH);
            g_mqttPubSub.publish(g_mqttStatusTopic.c_str(), strPayload.c_str()); 
            Serial.println("MQTT: Send Data!!!");
            Serial.println(" ");
            Serial.println(" ");
            delay(500);
            digitalWrite(LED, LOW);
        }

        

    }
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------ Public Functions -----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/
void setup_wifi() 
{
    int counter = 0;
    byte mac[6];
    delay(10);
    // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(g_ssid);

    WiFi.begin(g_ssid, g_password);

    WiFi.macAddress(mac);
    g_UniqueId =  String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);

    Serial.print("Unique ID: ");
    Serial.println(g_UniqueId);    
   
    while(WiFi.status() != WL_CONNECTED && counter++ < 8) 
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");

    if(WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else
    {
        Serial.println("WiFi NOT connected!!!");
    }
}

void MqttReconnect() 
{
    // Loop until we're reconnected
    while (!g_mqttPubSub.connected()  && (g_mqttCounterConn++ < 4))
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (g_mqttPubSub.connect(g_deviceName.c_str(), g_mqttUser, g_mqttPsw)) 
        {
            Serial.println("connected");
            // Subscribe
            g_mqttPubSub.subscribe("homeassistant/status");
            delay(100);
        } else 
        {
            Serial.print("failed, rc=");
            Serial.print(g_mqttPubSub.state());
            Serial.println(" try again in 1 seconds");
            delay(1000);
        }
    }  
    g_mqttCounterConn = 0;
}

void MqttHomeAssistantDiscovery()
{
    String discoveryTopic;
    String payload;
    String strPayload;
    if(g_mqttPubSub.connected())
    {
        Serial.println("SEND HOME ASSISTANT DISCOVERY!!!");
        StaticJsonDocument<600> payload;
        JsonObject device;
        JsonArray identifiers;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Temperature
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_temp" + "/config";
        
        payload["name"] = g_deviceName + ".temp";
        payload["uniq_id"] = g_UniqueId + "_temp";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "temperature";
        payload["val_tpl"] = "{{ value_json.temp | is_defined }}";
        payload["unit_of_meas"] = "°C";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Humidity
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_hum" + "/config";
        
        payload["name"] = g_deviceName + ".hum";
        payload["uniq_id"] = g_UniqueId + "_hum";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "humidity";
        payload["val_tpl"] = "{{ value_json.hum | is_defined }}";
        payload["unit_of_meas"] = "%";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Oxygen
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_oxy" + "/config";
        
        payload["name"] = g_deviceName + ".oxy";
        payload["uniq_id"] = g_UniqueId + "_oxy";
        payload["stat_t"] = g_mqttStatusTopic;
        //payload["dev_cla"] = "none";
        payload["val_tpl"] = "{{ value_json.oxy | is_defined }}";
        payload["unit_of_meas"] = "%";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Ozone
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_ozn" + "/config";
        
        payload["name"] = g_deviceName + ".ozn";
        payload["uniq_id"] = g_UniqueId + "_ozn";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "ozone";
        payload["val_tpl"] = "{{ value_json.ozn | is_defined }}";
        payload["unit_of_meas"] = "ppb";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // PM2.5
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_pm25" + "/config";
        
        payload["name"] = g_deviceName + ".pm25";
        payload["uniq_id"] = g_UniqueId + "_pm25";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "pm25";
        payload["val_tpl"] = "{{ value_json.pm25 | is_defined }}";
        payload["unit_of_meas"] = "ug/m3";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Formaldehyde
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_frm" + "/config";
        
        payload["name"] = g_deviceName + ".frm";
        payload["uniq_id"] = g_UniqueId + "_frm";
        payload["stat_t"] = g_mqttStatusTopic;
        //payload["dev_cla"] = "formaldehyde";
        payload["val_tpl"] = "{{ value_json.frm | is_defined }}";
        payload["unit_of_meas"] = "ug/m3";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // CO2
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        payload.clear();
        device.clear();
        identifiers.clear();
        strPayload.clear();

        discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_co2" + "/config";
        
        payload["name"] = g_deviceName + ".co2";
        payload["uniq_id"] = g_UniqueId + "_co2";
        payload["stat_t"] = g_mqttStatusTopic;
        payload["dev_cla"] = "carbon_dioxide";
        payload["val_tpl"] = "{{ value_json.co2 | is_defined }}";
        payload["unit_of_meas"] = "ppm";
        device = payload.createNestedObject("device");
        device["name"] = g_deviceName;
        device["model"] = g_deviceModel;
        device["sw_version"] = g_swVersion;
        device["manufacturer"] = g_manufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(g_UniqueId);

        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, strPayload);

        g_mqttPubSub.publish(discoveryTopic.c_str(), strPayload.c_str());


    }
}

void MqttReceiverCallback(char* topic, byte* inFrame, unsigned int length) 
{
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    byte state = 0;
    String messageTemp;
    
    for (int i = 0; i < length; i++) 
    {
        Serial.print((char)inFrame[i]);
        messageTemp += (char)inFrame[i];
    }
    Serial.println();
  
    if(String(topic) == String("homeassistant/status")) 
    {
        if(messageTemp == "online")
            MqttHomeAssistantDiscovery();
    }
}

float Round2(float value) 
{
   return (int)(value * 100 + 0.5) / 100.0;
}
