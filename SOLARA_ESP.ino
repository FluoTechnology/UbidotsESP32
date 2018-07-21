#include <WiFi.h>
#include <PubSubClient.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define WIFISSID "MobileWiFi-e47f" // Put your WifiSSID here
#define PASSKEY "adry87ctpp2" // Put your wifi password here

#define TOKEN "BBFF-sUGTKCuh64z6vopS59jURWKR0visQp" // Put your Ubidots' TOKEN

#define MQTT_CLIENT_NAME "testtest87HJU" // MQTT client Name, you can set                                       
#define VARIABLE_LABEL_PUBLISH "sensor" // Assing the variable label
#define DEVICE_LABEL "fluowifi" // Assig the device label, you can set

// MQTT
char mqttBroker[]  = "things.ubidots.com";

//TIME
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

/****************************************
 * Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);

void callback(char* topic, byte* payload, unsigned int length) 
{
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);
    Serial.write(payload, length);
    Serial.println(topic);
}

void reconnect() 
{
 
}

// WIFI
byte LinkStatus;
void WiFiEvent(WiFiEvent_t event);

enum{
    LINK_STA,
    LINK_WAN,
    LINK_NO,
};

// SETUP
void setup() 
{
    static int TmpBlinkLink;

    HardwareInit();
  
    // WiFi Setting
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);

    WiFi.onEvent(WiFiEvent); // start

    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin(WIFISSID, PASSKEY);   // Assign the pins as INPUT/OUTPUT 

    Serial.println();
    Serial.print("Wait for WiFi...");
  
    while (LinkStatus != LINK_WAN) 
    {
        Serial.print(".");
        digitalWrite(LED_LINK, (TmpBlinkLink) ? HIGH : LOW); // ESP32 Alive
        TmpBlinkLink = !TmpBlinkLink;
  
        delay(500);
    }

    //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime(); 
  
    client.setServer(mqttBroker, 1883);
    client.setCallback(callback);

    delay(2000);

}

//LOOP
void loop() 
{
    Scheduler();
}


void printLocalTime()
{
    struct tm timeinfo;
    
    if(!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        delay(1000);
        return;
    }
    
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    delay(1000);
}

unsigned long prevMillisTaskSendData;
unsigned long prevMillisTaskGetTime;
unsigned long prevMillisTaskUpgradeData;
unsigned long prevMillisTaskService;

byte TaskSendDataSem;
byte TaskGetTimeSem;
byte TaskUpgradeDataSem;
byte TaskServiceSem;

void Scheduler()
{

    if ( hasExp(prevMillisTaskUpgradeData, 100) ) // 500 ms interval
    {
        if (!TaskUpgradeDataSem)
            TaskUpgradeData();
    }
  
    if ( hasExp(prevMillisTaskSendData, 5000) ) // 5000 + 500 ms second interval
    {
        if (!TaskSendDataSem)
            TaskSendData();
    }
  
    if ( hasExp(prevMillisTaskService, 1000) ) // 1 sec second interval
    {
        if (!TaskServiceSem)
            TaskService();
    }
  
    if ( hasExp(prevMillisTaskGetTime, 120000) ) // 120 sec second interval
    {
        if (!TaskGetTimeSem)
            TaskGetTime();
    }
  
}

void TaskUpgradeData()
{ 
    //SENSOR
  
    AddSensor(0, "sensor_00", random(1, 255), "Celsius", "Temp Combustion Camera", "false");
    AddSensorLimit(0, 200, 250, 50, 25);

    AddSensor(1, "Sensor_01", random(1, 255), "Celsius", "Temperature Torcia", "false");
    AddSensorLimit(1, 200, 250, 50, 25);

    AddSensor(2, "Sensor_02", random(1, 10), "Bar", "Pressione camera iperbarica", "false");
    AddSensorLimit(2, 9, 7.7, 2, 1.2);

    AddSensor(3, "Sensor_03", random(273, 523), "Kelvin", "Temperature Condotto", "false");
    AddSensorLimit(3, 500, 420, 300, 280);

    AddSensor(4, "Sensor_04", random(273, 523), "Kelvin", "Temperature Condotto", "false");
    AddSensorLimit(4, 500, 420, 300, 280);
    

    //VAR

    AddVariable(0, "Pressure", random(150000, 300000), "Pa", "Pressione Principale");
    
    AddVariable(1, "Fan_1", random(0, 1000), "rpm", "Ventola Principale");
        
    AddVariable(2, "Fan_2", random(0, 1000), "rpm", "Ventola Secondaria");
            
    AddVariable(3, "MotorPower", random(0, 11), "kW", "Potenza Richiesta Motore");

    AddVariable(4, "RealPower", random(0, 11), "kW", "Potenza Effettiva Motore");

    AddVariable(5, "Temp_1", random(0, 100), "Celsius", "Temperatura Impianto");
    
    AddVariable(6, "Temp_2", random(0, 40), "Celsius", "Temperatura Ambiente");
    

    // ALARM                    

    AddAlarm(0, "Alarm_1", random(0, 2), "Alarm General 1", "Spegnere Impianto");

    AddAlarm(1, "Alarm_2", random(0, 2), "Alarm General 2", "Cambiare Filtro");

    AddAlarm(2, "Alarm_3", random(0, 2), "Alarm General 3", "Chiudere Valvola 1");

    AddAlarm(3, "Alarm_4", random(0, 2), "Alarm General 4", "Aprire Valvola 3");

    AddAlarm(4, "AUTO_MANUAL", random(0, 2), "Automazione Impianto", "N/D"); // Special case
    

    // UPGRADE STRUCT 
     SensorsData();
     AlarmData();
     VariableData();

}

void TaskGetTime()
{
    printLocalTime();
}

//VARIABLE

#define VARIABLES_MAX 10 

// private structure IO
struct VARIABLES_t
{
    char name[16];
    float value;
    
    char metric[8];
    char description[64];

    byte busy;
};
struct VARIABLES_t variable[VARIABLES_MAX];

void AddVariable(byte count, const char *name, float value, const char *metric, const char *description)
{
    strncpy(variable[count].name, name, 16); // copy name

    variable[count].value = value;
    
    strncpy(variable[count].metric, metric, 8); // copy metric
    
    strncpy(variable[count].description, description, 64); // copy description

    variable[count].busy = 1;
}

//VARIABLE

#define ALARMS_MAX 10 

// private structure IO
struct ALARMS_t
{
    char name[16];
    float value;
    
    char description[64];
    char action[64];

    byte busy;
};
struct ALARMS_t alarmLL[ALARMS_MAX];


void AddAlarm(byte count, const char *name, float value, const char *description, const char *action)
{
    strncpy(alarmLL[count].name, name, 16); // copy name

    alarmLL[count].value = value;
    
    strncpy(alarmLL[count].description, description, 64); // copy description

    strncpy(alarmLL[count].action, action, 64); // copy description

    alarmLL[count].busy = 1;
}


//SENSORS

#define SENSORS_MAX 10 // 30 sensori max

// private structure IO
struct SENSORS_t
{
    char name[16];
    float value;
    char metric[8];
    char description[64];
    char alarm[5];

    float prelimUP;
    float limUP;
    float prelimDOWN;
    float limDOWN;

    byte busy;
};
struct SENSORS_t sensor[SENSORS_MAX];

void AddSensor(byte count, const char *name, float value, const char *metric, const char *description, const char *alarm)
{
    strncpy(sensor[count].name, name, 16); // copy name

    sensor[count].value = value;
    
    strncpy(sensor[count].metric, metric, 8); // copy metric
    
    strncpy(sensor[count].description, description, 64); // copy description

    strncpy(sensor[count].alarm, alarm, 5); // copy alarm

    sensor[count].busy = 1;
}

void AddSensorLimit(byte count, float prelimUP, float limUP, float prelimDOWN, float limDOWN)
{
    sensor[count].prelimUP = prelimUP;
    sensor[count].limUP = limUP;
    sensor[count].prelimDOWN = prelimDOWN;
    sensor[count].limDOWN = limDOWN;
}

// PKT

#define SPKT_MAX 10 // 10 PKT max
#define VPKT_MAX 10 // 10 PKT max
#define APKT_MAX 10 // 10 PKT max

struct SPKT_t
{
    char payload[200];
    char topic[50];
    byte empty;
};
struct SPKT_t Spkt[SPKT_MAX];

struct VPKT_t
{
    char payload[200];
    char topic[50];
    byte empty;
};
struct VPKT_t Vpkt[VPKT_MAX];

struct APKT_t
{
    char payload[200];
    char topic[50];
    byte empty;
};
struct APKT_t Apkt[APKT_MAX];


void SensorsData()
{
    static int i; 
     
    // Space to store values to send
    char str_value[10];
    char str_prelimUP[10];
    char str_prelimDOWN[10];
    char str_limUP[10];
    char str_limDOWN[10];

    // access SENSORS struct pointer element 0
    struct SENSORS_t *st; // new pointer
    st = &sensor[i];

    if(st->busy == 0)
    {
        i = 0;
        return;
    }

    // access PKT struct pointer element 0
    struct SPKT_t *pt; // new pointer
    pt = &Spkt[i];

    //TOPIC
    
    sprintf(pt->topic, "%s", ""); // Cleans the topic content
    sprintf(pt->topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

    // PAYLOAD
    
    sprintf(pt->payload, "%s", ""); // Cleans the payload
    sprintf(pt->payload, "{\"%s\":", st->name); // Adds the variable label
    

    /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
    dtostrf(st->value, 4, 2, str_value);
        
    dtostrf(st->prelimUP, 4, 2, str_prelimUP);
    dtostrf(st->prelimDOWN, 4, 2, str_prelimDOWN);
    dtostrf(st->limUP, 4, 2, str_limUP);
    dtostrf(st->limDOWN, 4, 2, str_limDOWN);
  
    sprintf(pt->payload, "%s {\"value\": %s", pt->payload, str_value); // Adds the value

    sprintf(pt->payload, "%s, \"context\":{\"metric\": \"%s\", \"descrip\": \"%s\", \"alarm\": \"%s\", \"limUP\": %s, \"plimUP\": %s, \"limDOWN\": %s, \"plimDOWN\": %s}", pt->payload, st->metric, st->description, st->alarm, str_limUP, str_prelimUP, str_limDOWN, str_prelimDOWN); // Adds context !
    
    sprintf(pt->payload, "%s } }", pt->payload); // Closes the dictionary brackets

    pt->empty = 1;

    i++;
}

void VariableData()
{
    static int i; 
     
    // Space to store values to send
    char str_value[10];

    // access SENSORS struct pointer element 0
    struct VARIABLES_t *var; // new pointer
    var = &variable[i];

    if(var->busy == 0)
    {   
        i = 0;
        return;
    }

    // access PKT struct pointer element 0
    struct VPKT_t *pt; // new pointer
    pt = &Vpkt[i];

    //TOPIC
    
    sprintf(pt->topic, "%s", ""); // Cleans the topic content
    sprintf(pt->topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

    // PAYLOAD
    
    sprintf(pt->payload, "%s", ""); // Cleans the payload
    sprintf(pt->payload, "{\"%s\":", var->name); // Adds the variable label
    
    /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
    dtostrf(var->value, 4, 2, str_value);
  
    sprintf(pt->payload, "%s {\"value\": %s", pt->payload, str_value); // Adds the value

    sprintf(pt->payload, "%s, \"context\":{\"metric\": \"%s\", \"descrip\": \"%s\"}", pt->payload, var->metric, var->description); // Adds context !
    
    sprintf(pt->payload, "%s } }", pt->payload); // Closes the dictionary brackets

    pt->empty = 1;

    i++;
}

void AlarmData()
{
    static int i; 
     
    // Space to store values to send
    char str_value[2];

    // access SENSORS struct pointer element 0
    struct ALARMS_t *al; // new pointer
    al = &alarmLL[i];

    if(al->busy == 0)
    {
        i = 0;
        return;
    }

    // access PKT struct pointer element 0
    struct APKT_t *pt; // new pointer
    pt = &Apkt[i];

    //TOPIC
    
    sprintf(pt->topic, "%s", ""); // Cleans the topic content
    sprintf(pt->topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

    // PAYLOAD
    
    sprintf(pt->payload, "%s", ""); // Cleans the payload
    sprintf(pt->payload, "{\"%s\":", al->name); // Adds the variable label

    itoa(al->value, str_value, 2);

    sprintf(pt->payload, "%s {\"value\": %s", pt->payload, str_value); // Adds the value

    sprintf(pt->payload, "%s, \"context\":{\"descrip\": \"%s\", \"action\": \"%s\"}", pt->payload, al->description, al->action); // Adds context !
    
    sprintf(pt->payload, "%s } }", pt->payload); // Closes the dictionary brackets

    pt->empty = 1;

    i++;
}


void TaskSendData()
{
     static char choice= 'S' ;

     // Loop until we're reconnected
     while (!client.connected()) 
     {
         Serial.println("Attempting MQTT connection...");
            
         // Attemp to connect
         if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) 
         {
             Serial.println("Connected");
         } 
         else 
         {
             Serial.print("Failed, rc=");
             Serial.print(client.state());
             Serial.println(" try again in 2 seconds");
     
             delay(2000); // Wait 2 seconds before retrying
             return; // check if MQTT link is up
         }
     }
     
     digitalWrite(LED_BLE, HIGH);

     Serial.println("Publishing data to Ubidots Cloud");

        switch(choice) 
        {

            case 'S':
            
                    for(int i=0; i <= SPKT_MAX; i++)
                    {
                          if (Spkt[i].empty == 0)
                              break;
                        
                          client.publish(Spkt[i].topic, Spkt[i].payload); // publish
                          
                          Serial.println(Spkt[i].topic);
                          Serial.println(Spkt[i].payload);
              
                          delay(250);
                     }
                     
                     choice = 'V';
            break; 

            case 'V':
            
                    for(int i=0; i <= VPKT_MAX; i++)
                    {
                          if (Vpkt[i].empty == 0)
                              break;
                        
                          client.publish(Vpkt[i].topic, Vpkt[i].payload); // publish
                          
                          Serial.println(Vpkt[i].topic);
                          Serial.println(Vpkt[i].payload);
              
                          delay(250);
                     }
                     
                     choice = 'A';
            break; 

            case 'A':
            
                    for(int i=0; i <= APKT_MAX; i++)
                    {
                          if (Spkt[i].empty == 0)
                              break;
                        
                          client.publish(Apkt[i].topic, Apkt[i].payload); // publish
                          
                          Serial.println(Apkt[i].topic);
                          Serial.println(Apkt[i].payload);
              
                          delay(250);
                     }
                     
                     choice = 'S'; // restart
            break; 
  

        }

        client.loop();
        
        digitalWrite(LED_BLE, LOW);
        
}

void TaskService()
{

    digitalWrite(LED_STATUS, HIGH); // ESP32 Alive
  
    if (LinkStatus == LINK_STA)
    {
        digitalWrite(LED_LINK, HIGH);
        digitalWrite(LED_CLOUD, LOW);
    }
  
      if (LinkStatus == LINK_WAN)
    {
        digitalWrite(LED_LINK, HIGH);
        digitalWrite(LED_CLOUD, HIGH);
    }
  
    if (LinkStatus == LINK_NO)
    {
        digitalWrite(LED_LINK, LOW);
        digitalWrite(LED_CLOUD, LOW);
    }
  
}

void HardwareInit()
{
  
    pinMode(LED_STATUS, OUTPUT); //FluoWiFi only
    pinMode(LED_LINK, OUTPUT);   //FluoWiFi only
    pinMode(LED_CLOUD, OUTPUT);  //FluoWiFi only
    pinMode(LED_BLE, OUTPUT);    //FluoWiFi only
    pinMode(SPI_EN, OUTPUT);     //FluoWiFi only
    pinMode(SDCD, INPUT);        //FluoWiFi only
    pinMode(RESET_644P, OUTPUT); //FluoWiFi only
    
    pinMode(INTERRUPT_644P, OUTPUT);  //FluoWiFi only
    
    pinMode(BUTTON_CHECK, OUTPUT);       //FluoWiFi only
    pinMode(BUTTON_APRST, INPUT_PULLUP); //FluoWiFi only

    digitalWrite(LED_STATUS, 1);  // force on  
    digitalWrite(LED_CLOUD, 0);
    digitalWrite(LED_LINK, 0);
    digitalWrite(LED_BLE, 0);
    digitalWrite(SPI_EN, 1);      // 0 enable - 1 disable

    Serial.begin(115200);
    //SerialInternal.begin(57600);
}

bool hasExp(unsigned long &prevTime, unsigned long interval)
{
    if (  millis() - prevTime > interval )
    {
        prevTime = millis();
        return true;
    }
  
    else
        return false;
}


// Check Internet connection
void CheckInternet()
{
    WiFiClient clientPing;
  
    // se il ping e' ok return 1
    if( clientPing.connect("www.google.com", 80) == 1 )
        LinkStatus = LINK_WAN;
  
    clientPing.stop();
}


//--------------------------------------------------------
// WiFiEvent_t event

void WiFiEvent(WiFiEvent_t event)
{
    switch(event)
    {

        case SYSTEM_EVENT_STA_START:
        {
            Serial.println("[WiFiEvent] STA_START ");
            break;
        }
  
        case SYSTEM_EVENT_STA_STOP:
        {
            Serial.println("[WiFiEvent] STA_STOP ");
            break;
        }
  
        case SYSTEM_EVENT_STA_CONNECTED:
        {
            LinkStatus = LINK_STA;
            Serial.println("[WiFiEvent] ESP_STA connected with a HotSpot ");
            break;
        }
  
        case SYSTEM_EVENT_STA_GOT_IP:
        {
            Serial.println("[WiFiEvent] ESP_STA connected and GOT IP");
            Serial.print("[WiFiEvent] ESP_STA -> IP address: ");
            Serial.println(WiFi.localIP());
    
            CheckInternet();
    
            break;
        }
  
        case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            Serial.println("[WiFiEventNew] ESP_STA HotSpot Signal lost ");
    
            WiFi.begin();
            WiFi.reconnect(); // Try reconnect
    
            LinkStatus = LINK_NO;
            break;
        }

    }
    
}
