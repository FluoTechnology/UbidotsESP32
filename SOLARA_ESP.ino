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
#define DEVICE_LABEL "FluoWiFi" // Assig the device label, you can set

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
unsigned long prevMillisTaskCheckReconn;
unsigned long prevMillisTaskService;

byte TaskSendDataSem;
byte TaskGetTimeSem = 1;
byte TaskCheckReconnSem;
byte TaskServiceSem;

void Scheduler()
{
  
    if ( hasExp(prevMillisTaskSendData, 3500) ) // 3500 + 500 ms second interval
    {
        if (!TaskSendDataSem)
            TaskSendData();
    }
  
    if ( hasExp(prevMillisTaskService, 1000) ) // 1 sec second interval
    {
        if (!TaskServiceSem)
            TaskService();
    }
  
    if ( hasExp(prevMillisTaskGetTime, 60000) ) // 60 sec second interval
    {
        if (!TaskGetTimeSem)
            TaskGetTime();
    }
  
    if ( hasExp(prevMillisTaskCheckReconn, 2000) ) // 1 second interval
    {
        if (!TaskCheckReconnSem)
            TaskCheckReconn();
    }
  
}

void TaskCheckReconn()
{ 
  //
}

void TaskGetTime()
{
    printLocalTime();
}

#define SENSORS_MAX 30 // 30 sensori max
#define PKT_MAX 30 // 30 PKT max

// private structure IO
struct SENSORS_t
{
    uint8_t mode; 
    uint8_t busy; 
    uint8_t type; 
    int16_t value; 
};
struct SENSORS_t sensor[SENSORS_MAX]; 

struct PKT_t
{
    char payload[100];
    char topic[150];
};
struct PKT_t pkt[PKT_MAX];


void SensorsData()
{
     
  // Space to store values to send
    static char str_sensor[10];
    static char str_lat[6];
    static char str_lng[6];

    // access PKT struct pointer element 0
    struct PKT_t *pt; // new pointer
    pt = &pkt[0];  

    //TOPIC
    
    sprintf(pt->topic, "%s", ""); // Cleans the topic content
    sprintf(pt->topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

    // PAYLOAD
    
    sprintf(pt->payload, "%s", ""); // Cleans the payload content
    sprintf(pt->payload, "%s", ""); // Cleans the payload
    sprintf(pt->payload, "{\"%s\":", VARIABLE_LABEL_PUBLISH); // Adds the variable label
    
    float sensor = 23.7; 
    float lat = 6.101;
    float lng = -1.293;

    /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
    dtostrf(sensor, 4, 2, str_sensor);
    dtostrf(lat, 4, 2, str_lat);
    dtostrf(lng, 4, 2, str_lng);  
  
    sprintf(pt->payload, "%s {\"value\": %s", pt->payload, str_sensor); // Adds the value
    sprintf(pt->payload, "%s, \"context\":{\"lat\": %s, \"lng\": %s}", pt->payload, str_lat, str_lng); // Adds coordinates
    sprintf(pt->payload, "%s } }", pt->payload); // Closes the dictionary brackets
  
}



void TaskSendData()
{

     SensorsData();

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
         }
     }
        digitalWrite(LED_BLE, HIGH);
        
        Serial.println("Publishing data to Ubidots Cloud");
        Serial.println(pkt[0].topic);
        Serial.println(pkt[0].payload);
        delay(2000);
        
        client.publish(pkt[0].topic, pkt[0].payload); // publish
        client.loop();
        
        delay(500);
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
  
    pinMode(LED_STATUS, OUTPUT);
    pinMode(LED_LINK, OUTPUT);
    pinMode(LED_CLOUD, OUTPUT);
    pinMode(LED_BLE, OUTPUT);
    pinMode(SPI_EN, OUTPUT);
    pinMode(SDCD, INPUT);
    pinMode(RESET_644P, OUTPUT);

    digitalWrite(LED_STATUS, 1);  // force on  
    digitalWrite(LED_CLOUD, 0);
    digitalWrite(LED_LINK, 0);
    digitalWrite(LED_BLE, 0);
    digitalWrite(SPI_EN, 1);      // 0 enable - 1 disable

    pinMode(INTERRUPT_644P, OUTPUT);   //interrupt 644p

    pinMode(BUTTON_CHECK, OUTPUT);
    pinMode(BUTTON_APRST, INPUT_PULLUP);

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
