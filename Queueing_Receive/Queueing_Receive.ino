#include <ESP8266WiFi.h>
#include <espnow.h>
//#include "DHT.h"
//#include <Adafruit_Sensor.h>

#define ME 0X21
#define NEXT 0X22
#define SIZE 150
#define WAIT_TIME 400

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
uint8_t nextAddress[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, NEXT};

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
void Enqueue(Packet QueueData);
void Dequeue();
void loadPacket();

void setup() {
  //initialize baud rate and configure as station
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println();
  delay(5000);
  Serial.println("main>5s delay passed");
  Serial.println("Magandang Umaga, Supremo!");

  //change mac address to something more memorable
  wifi_set_macaddr(STATION_IF, myAddress);

  if (esp_now_init() != 0) {
  Serial.println("Error initializing ESP-NOW");
  return;
  }

  //pinMode(16, OUTPUT);

  //sets the esp as both master and slave
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  //callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  //add device to paired device list, max of 20
  esp_now_add_peer(nextAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);

  Serial.println("MY MAC:" + WiFi.macAddress());
  delay(50);
  Serial.print("Next device: ");
  Serial.println(NEXT);
}

void loop() {
    if (millis() > previousMillis) {
    previousMillis = previousMillis + WAIT_TIME;
    loadPacket();
  }
}

void loadPacket() {
  //read sensors
  if(theData.packet->t != 0 && theData.packet->h !=0){
    esp_now_send(nextAddress, (uint8_t *) &theData, sizeof(theData));
    return;
  }
  //store reading into current packet
  //actually sends data
  //esp_now_send(nextAddress, (uint8_t *) &theData, sizeof(theData));
}


void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
    //if queue is not empty, dequeue 
    if (front != -1){
    Dequeue();
  }
  }
  else {
    Serial.println("Delivery fail");
    //add to queue
    Enqueue(theData);
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

  char macStr[18];
  Serial.print("Packet received from: ");

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(macStr);
  
  //copy the incoming data to a local variable, NOTE:the size must be exactly the same or it won't work
  memcpy(&incData, incomingData, sizeof(incData)); //dest,source,size
  Serial.println("Incoming Packet:");
  //retrieves the data
  Serial.println("DATA: " + String(incData.packet[0].t) + " " + String(incData.packet[0].h));
  Serial.println(incData.datetime);
  Serial.println();
  //Save the incoming sensor readings to the next packet
  theData = incData;
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
  Serial.println();
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
