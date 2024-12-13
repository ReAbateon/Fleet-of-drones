#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>  // Per esp_read_mac()
//Librerie per giroscopio:
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//Libreria per radar:
#include <ESP32Servo.h>

#define MAC_SIZE 6
#define MAX_CONNESSI 3
#define WIFI_CHANNEL 11

#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 1
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 1

#define DRONE_WITH_RADAR

// MAC Address del Server (sostituiscilo con l'indirizzo reale del tuo Server)
uint8_t serverAddress[] = {0x30, 0x83, 0x98, 0x7b, 0x8f, 0x30};
uint8_t zeroMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t myMAC[6];

int ping_drone[MAX_CONNESSI-1] = {0}; //se ho pingato un drone
int ack_drone[MAX_CONNESSI-1] = {0}; //se un drone mi ha risposto col vivo
int ack = 0; //indica se ho ricevuto un ack del server
int n = 0; //numero di messaggi da inviare prima di interrompere (a scopo di test)
int ping_server = 0; //indica se ho già inviato un ping
int tower = 0; //indica se è connesso alla torre
int missionStatus = 0; //indica la missione a che punto è
uint8_t droni_connessi[MAX_CONNESSI-1][6];
int drone_table[MAX_CONNESSI-1][2];
int d = 0;  //numero di droni connessi a questo drone

unsigned long lastMessageTime = 0;
const unsigned long Timeout = 11000;
unsigned long lastMessageDroneTime[MAX_CONNESSI-1] = {0};
const unsigned long Timeout_Drone = 5000;

// Struttura dati da inviare (vuota, usata solo come heartbeat)
typedef struct struct_message {
  char message[32];
  uint8_t MAC_RICEVUTI[MAX_CONNESSI][6];
} struct_message;

typedef struct info_drone{
  char message[32];
  int mStatus;
  int droni_c[MAX_CONNESSI-1];
  int slope;
  int temperature;
  int obstacle;
} struct_info;

typedef struct mess_drone{
  char message[32];
}struct_drone;

struct_info dataToSend;
struct_message receivedMessage;
struct_drone droneToSend;
struct_drone receivedDrone;

//Giroscopio
Adafruit_MPU6050 mpu;
const float acc_g = 9.81;

//Radar
#define TRIG_PIN 23 // Pin TRIG del sensore ad ultrasuoni
#define ECHO_PIN 19 // Pin ECHO del sensore ad ultrasuoni
#define SERVO_PIN 26 // Pin del servomotore
#define LED_PIN 18 // Pin del led
#define DISTANCE_THRESHOLD 50.0 // Soglia di distanza per attivare il buzzer
float duration_us, distance_cm;
int obstacleToSend = 0;
Servo myServo;

//Thread per drone
void thDrone(void *pv){
  while (true) {
   
    //Gyroscope's data:
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    int slope_angle = acos(0.707 * (a.acceleration.x + a.acceleration.y) / acc_g) * (180.0 / M_PI);

    // Invia ogni 2 secondi
    if(tower == 0 && ack == 0){
      strcpy(dataToSend.message, "CONNECTION");
      esp_now_send(serverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
      Serial.println("Connection request sent");
    }else if(ack == 1 && tower == 1){
      if(n != 10){
        strcpy(dataToSend.message, "INFO");
        dataToSend.mStatus = missionStatus;
        dataToSend.slope = (slope_angle-88)*2; //Le cose semplici sono le migliori ;)
        dataToSend.temperature = temp.temperature;
        dataToSend.obstacle = obstacleToSend;
        for(int z=0; z<MAX_CONNESSI-1; z++){
          dataToSend.droni_c[z]=drone_table[z][0];
        }
        esp_now_send(serverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
        Serial.println("INFO sent to tower");
        ack = 0;
        n++;
      }
    }else if(ack == 0 && millis() - lastMessageTime > Timeout){
      if(ping_server == 1){
        tower = 0;
        strcpy(dataToSend.message, "CONNECTION");
        esp_now_send(serverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
        Serial.println("Connection request sent");
        lastMessageTime = millis();
      }else{
        Serial.println("PING to tower sent");
        strcpy(dataToSend.message, "PING");
        esp_now_send(serverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
        lastMessageTime = millis();
        ping_server = 1;
      }
    }

    for(int i=0; i<MAX_CONNESSI-1; i++){
      if(drone_table[i][0] == 0){
        lastMessageDroneTime[i]=millis();
      }
      if(ping_drone[i] == 1 && millis()-lastMessageDroneTime[i] > Timeout_Drone){
        lastMessageDroneTime[i] = millis();
        drone_table[i][0] = 0;
        drone_table[i][1] = 0;
        d--;
        if (esp_now_del_peer(droni_connessi[i]) == ESP_OK) {
                  Serial.println("Peer removed");
        }else{
                  Serial.println("Errore durante la rimozione del peer");
        }
        memset(droni_connessi[i], 0, sizeof(droni_connessi[i]));
        if(tower==0){
          missionStatus=0;
        }
      }else if(drone_table[i][1] != 0 && ack_drone[i]==1){
        strcpy(droneToSend.message, "VIVO");
        lastMessageDroneTime[i]=millis();
        esp_now_send(droni_connessi[i], (uint8_t *)&droneToSend, sizeof(droneToSend));
        Serial.printf("ALIVE message to drone %d sent\n", abs(drone_table[i][0]) % 100000);
        ack_drone[i]=0;
      }else if(drone_table[i][1] != 0 && millis()-lastMessageDroneTime[i] > Timeout_Drone && ping_drone[i] == 0){
        strcpy(droneToSend.message, "VIVO");
        esp_now_send(droni_connessi[i], (uint8_t *)&droneToSend, sizeof(droneToSend));
        Serial.printf("ALIVE ping to drone %d sent\n", abs(drone_table[i][0]) % 100000);
        ack_drone[i]=0;
        lastMessageDroneTime[i] = millis();
        ping_drone[i] = 1;
      }
    }
    delay(2000);
    }
}

//Thread per il Radar
void thRadar(void *pv) {
  while (true) {
    // Rotazione del servomotore da 0° a 180° e viceversa
    for (int pos = 0; pos < 180; pos++) {
      myServo.write(pos); // Imposta l'angolo del servo
      delay(15); // Aspetta per permettere al servo di muoversi

      // Controlla la distanza con il sensore ad ultrasuoni
      #ifdef DRONE_WITH_RADAR
        checkDistance();
      #else 
        obstacleToSend = 0;
      #endif
      delay(15);
    }

    for (int pos = 181; pos >= 0; pos--) {
      myServo.write(pos);
      delay(15);

      // Controlla la distanza con il sensore ad ultrasuoni
      #ifdef DRONE_WITH_RADAR
        checkDistance();
      #else 
        obstacleToSend = 0;
      #endif
      delay(15);
    }
  }
}

//Funzione per il Radar
int checkDistance() {
  // Genera un impulso di 10 microsecondi per il pin TRIG
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Misura la durata del segnale ricevuto
  duration_us = pulseIn(ECHO_PIN, HIGH);
  // Calcola la distanza in cm
  distance_cm = 0.017 * duration_us;

  if(distance_cm < DISTANCE_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
    obstacleToSend = 1;    
  } else {
    digitalWrite(LED_PIN, LOW);
    obstacleToSend = 0;
  }
  return obstacleToSend;
}

// Funzione per aggiungere un drone nella tabella se non esiste già
int addOrUpdateDrone(const uint8_t* mac, bool connected) {
  int macInt = 0;
  // Calcola un valore intero dalle prime MAC_SIZE cifre del MAC
  for (int i = 0; i < MAC_SIZE; i++) {
    macInt = macInt * 256 + mac[i];
  }
 
  // Cerca se il drone è già presente nella tabella
  for (int i = 0; i < MAX_CONNESSI-1; i++) {
    if (drone_table[i][0] == macInt) {
      // Aggiorna lo stato del drone
      drone_table[i][1] = connected ? 1 : 0;
      ping_drone[i] = 0;  
      return i;
    }
  }
 
  // Se non è presente, aggiungilo alla tabella
  for (int i = 0; i < MAX_CONNESSI-1; i++) {
    if (drone_table[i][0] == 0) { // Slot vuoto trovato
      drone_table[i][0] = macInt;
      drone_table[i][1] = connected ? 1 : 0;
      ping_drone[i] = 0;

      
      memcpy(droni_connessi[i], mac, 6);

      Serial.print("MAC_Received : ");
      for(int k=0; k<6; k++) {
        Serial.printf("%02X", droni_connessi[i][k]);
        if(k<5) Serial.print(":");
      }
      Serial.println();
      
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, mac, 6); // Copia l'indirizzo MAC del server
      peerInfo.channel = WiFi.channel(); // Canale Wi-Fi
      peerInfo.encrypt = false; // Nessuna crittografia

      // Aggiungi il peer
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Errore durante l'aggiunta del peer");
      }
      
      d++;
      return i;
    }
  }
 
  // Tabella piena
  Serial.println("Errore: tabella droni piena!");
  return -1;
}
 

// Funzione per gestire la ricezione dei dati ESP-NOW
void onDataRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (memcmp(info->src_addr, serverAddress, 6)==0) {
    Serial.println("Messagge received from server");
    memcpy(&receivedMessage, data, sizeof(receivedMessage));
    if(strcmp(receivedMessage.message, "OPERATIVO") == 0){
      Serial.println("OPERATIONAL command received");
      ack = 1;
      lastMessageTime = millis();
      missionStatus = 2;
    }
    else if (strcmp(receivedMessage.message, "ACK") == 0) {
      Serial.println("ACK Received");
      ack = 1;
      lastMessageTime = millis();
    }else if (strcmp(receivedMessage.message, "PING") == 0){
      Serial.println("PING Riceived");
      strcpy(dataToSend.message, "INFO");
      dataToSend.mStatus = missionStatus;
      esp_now_send(serverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
      Serial.println("INFO sent");
      ack=0;
      n=0;
    }else if (strcmp(receivedMessage.message, "PING_PANIK") == 0){
      Serial.println("PING_PANIK Received");
      tower = 1;
      ping_server = 0;
      ack = 1;
      lastMessageTime = millis(); 
    }else if (strcmp(receivedMessage.message, "IN_ATTESA") == 0){
      Serial.println("WAITING command received");
      missionStatus = 1;
      tower = 1;
      ack = 1;
      lastMessageTime = millis();
    }else if (strcmp(receivedMessage.message, "ATTERRAGGIO") == 0){
      Serial.println("LANDING command received");
      missionStatus = 0;
      ack = 1;
      lastMessageTime = millis();
    } else if (strcmp(receivedMessage.message, "PUBBLICAZIONE") == 0) {
        Serial.println("ANNOUNCEMENT received");
        int i, j=0;
        while(i<MAX_CONNESSI && j<MAX_CONNESSI) {
            if ((memcmp(WiFi.macAddress(myMAC), receivedMessage.MAC_RICEVUTI[i], 6) != 0) && (memcmp(zeroMAC, receivedMessage.MAC_RICEVUTI[i], 6) != 0)) {
              for(int k=0; k<6; k++) {
                Serial.printf("%02X", receivedMessage.MAC_RICEVUTI[j][k]);
                if(k<5) Serial.print(":");
              }
              Serial.println("");
              esp_now_peer_info_t peerInfo = {};
              memcpy(peerInfo.peer_addr, receivedMessage.MAC_RICEVUTI[i], 6); // Copia l'indirizzo MAC del server
              peerInfo.channel = WIFI_CHANNEL; // Canale Wi-Fi
              peerInfo.encrypt = false; // Nessuna crittografia

              // Aggiungi il peer
              if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                Serial.println("Errore durante l'aggiunta del peer");
              }else{
                strcpy(droneToSend.message, "VIVO");
                esp_now_send(receivedMessage.MAC_RICEVUTI[i], (uint8_t *)&droneToSend, sizeof(droneToSend));
                Serial.printf("ALIVE Message sent to drone %d\n", j+1);   
              }
            
              i++;
              j++;

            } else {
              i++;
            }
        }
        ack = 1;
        tower = 1;
        lastMessageTime = millis();
      }
      
  } else {
      memcpy(&receivedDrone, data, sizeof(receivedDrone));
      Serial.println("Message received from drone");
      int droneIndex = addOrUpdateDrone(info->src_addr, true);
      if(strcmp(receivedDrone.message, "VIVO") == 0){
        Serial.printf("ALIVE Message received from drone %d \n", abs(drone_table[droneIndex][0]) % 100000);
        ack_drone[droneIndex] = 1;
        lastMessageDroneTime[droneIndex] = millis();
      }

  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  //Inizializza il giroscopio
  while (!Serial)
    delay(10); 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }else{
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      Serial.print("Accelerometer range set to: ");
      switch (mpu.getAccelerometerRange()) {
      case MPU6050_RANGE_2_G:
        break;
      case MPU6050_RANGE_4_G:
        break;
      case MPU6050_RANGE_8_G:
        break;
      case MPU6050_RANGE_16_G:
        break;
      }
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG:
        break;
      case MPU6050_RANGE_500_DEG:
        break;
      case MPU6050_RANGE_1000_DEG:
        break;
      case MPU6050_RANGE_2000_DEG:
        break;
      }

      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ:
        break;
      case MPU6050_BAND_184_HZ:
        break;
      case MPU6050_BAND_94_HZ:
        break;
      case MPU6050_BAND_44_HZ:
        break;
      case MPU6050_BAND_21_HZ:
        break;
      case MPU6050_BAND_10_HZ:
        break;
      case MPU6050_BAND_5_HZ:
        break;
      }
  }
  
  //Inizializza Radar
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  myServo.attach(SERVO_PIN);
  
  // Inizializza ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true); // Necessario per impostare manualmente il canale
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Errore durante l'inizializzazione di ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, serverAddress, 6); // Copia l'indirizzo MAC del server
  peerInfo.channel = WIFI_CHANNEL; // Canale Wi-Fi
  peerInfo.encrypt = false; // Nessuna crittografia

  // Aggiungi il peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Errore durante l'aggiunta del peer");
  }

  esp_now_register_recv_cb(onDataRecv);

  xTaskCreate(
      thDrone,          // Funzione che implementa il primo task
      "Thread Drone",  // Nome del task (per debug)
      4096,           // Stack size in parole (1000 parole sono circa 4 KB)
      NULL,           // Parametro passato al task
      2,              // Priorità (1 = bassa, valori più alti = priorità maggiore)
      NULL);          // Handle del task (non necessario qui)

  xTaskCreate(
      thRadar,          // Funzione che implementa il secondo task
      "Thread Radar",   // Nome del task
      4096,           // Stack size
      NULL,           // Parametro
      1,              // Priorità
      NULL);          // Handle
}

void loop() {

}


