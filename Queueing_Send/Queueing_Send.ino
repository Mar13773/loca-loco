#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
//#include "DHT.h"
//#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);
#include <SPI.h>
#include <SD.h>

#define ME 0X20
#define NEXT 0X21
#define SIZE 150
#define GREENLED D5 
#define REDLED D6  
#define WAIT_TIME 400

#define DHTPIN D2
#define DHTTYPE DHT22
//DHT dht(DHTPIN, DHTTYPE);

#define countof(a) (sizeof(a) / sizeof(a[0]))

const int chipSelect = 10; // Data logging SD shields and modules use pin 10

//where we put data from sensors
//float = 4 bytes, uint16 = 2 bytes
typedef struct SensorReadings {
  //float soilMoisture;
  float t;
  float h;
  //uint16_t id;
} SensorReadings;

//data struct for transmission
typedef struct Packet {
  uint8_t id = ME;
  SensorReadings packet[30];
  char datetime[];
} Packet;

Packet incData;
Packet theData;

Packet items[SIZE]; 
int front = -1, rear = -1;

unsigned long previousMillis = 0;

uint8_t myAddress[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, ME};
uint8_t nextAddress[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, NEXT} ;

void Enqueue(Packet QueueData);
void Dequeue();
void loadPacket();
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
void printDateTime(const RtcDateTime& dt);

void setup() {
  //initialize baud rate and configure as station
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //change mac address to something more memorable
  wifi_set_macaddr(STATION_IF, myAddress);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //dht.begin();
  //pinMode(GREENLED, OUTPUT);
  //pinMode(REDLED, OUTPUT);

  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

   if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) {
            // There's a communications error
            Serial.print("RTC communications error = ");
            Serial.println(Rtc.LastError());
        }
        else {
            // Set the RTC to the date & time this sketch was compiled
            Rtc.SetDateTime(compiled);
        }
    }

    if (!Rtc.GetIsRunning()) {
      // RTC is not actively running, and thus start now
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) {
        // RTC is older than compile time; hence update DateTime
        Rtc.SetDateTime(compiled);
    }
  
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 

  // see if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }

  //sets the esp as both master and slave
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  //callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  //add device to paired device list, max of 20
  esp_now_add_peer(nextAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);

  Serial.println();
  delay(5000);
  Serial.println("main>5s delay passed");
  Serial.println("Magandang Umaga!");

  Serial.println("MY MAC:" + WiFi.macAddress());
  delay(50);
  Serial.print("Next device: ");
  Serial.println(NEXT);
  Serial.println();
}

void loop() {
  if (millis() > previousMillis) {
    previousMillis = previousMillis + WAIT_TIME;
    loadPacket();
    RtcDateTime now = Rtc.GetDateTime();
    printDateTime(now);

    //actually sends data
    esp_now_send(nextAddress, (uint8_t *) &theData, sizeof(theData));
  }
}

void loadPacket() {
  //read sensors
  //float t = dht.readTemperature();
  //float h = dht.readHumidity();
  SensorReadings Temp;
  Temp.t = 1.2;
  Temp.h = 2.3;

  //store reading into current packet
  theData.packet[0] = Temp;
  Serial.println();
  String sensorstring = "SENSOR READINGS -> Temp: " + (String)Temp.t + " Hum: " + (String)Temp.h;
  Serial.println(sensorstring);

  // Open the SD card and create or open the text file
  File dataFile = SD.open("carpe.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(sensorstring);
      dataFile.close();
    }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
    //digitalWrite(GREENLED, HIGH);
    //digitalWrite(REDLED, LOW);
    //if queue is not empty, dequeue 
    if (front != -1){
    Dequeue();
  }
  }
  else {
    Serial.println("Delivery fail");
    //digitalWrite(REDLED, HIGH);
    //digitalWrite(GREENLED, LOW);
    //add to queue
    Enqueue(theData);
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  //copy the incoming data to a local variable, NOTE:the size must be exactly the same or it won't work
  memcpy(&incData, incomingData, sizeof(incData)); //dest,source,size
  Serial.println("Incoming Packet:");
  //retrieves the data
  // for (byte i = 0; i < incData.l; i++) {
  //   Serial.println("DATA: " + String(incData.l) + " " + String(*mac) + " " + String(incData.packet[i].t) + " " + String(incData.packet[i].h));
  //   if (theData.l >= 30) {
  //     Serial.println("ERROR: Data overflow");
  //     break;
  //   }

  // }
}

void Enqueue(Packet QueueData){
  //check if queue is full, else save the current data to queue
  if (rear == SIZE - 1){
    Serial.println("Queue is full");
  }
  else {
    //if first element, move the pointer to 0
    if (front == -1){
      front = 0;
    }
    rear++;
    items[rear] = QueueData;
    Serial.println("ADDING TO QUEUE -> [" + String(rear) + "] TEMP: " + String(items[rear].packet->t) + " HUM: " + String(items[rear].packet->h));
  }
  //Serial.println();
}

void Dequeue() {
  while(front != -1){
    esp_now_send(nextAddress, (uint8_t *) &items[front], sizeof(items[front]));
    front++;
    if (front > rear){
      front = rear = -1;
    }
  }
}

void printDateTime(const RtcDateTime& dt) {
    char datestring[21];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%04u-%02u-%02uT%02u:%02u:%02uZ"),
            dt.Year(),
            dt.Day(),
            dt.Month(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    strcpy(theData.datetime, datestring);
    Serial.println(theData.datetime);
    Serial.println();

    // Open the SD card and create or open the text file
    File dataFile = SD.open("carpe.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(datestring);
      dataFile.close();
    }
}
