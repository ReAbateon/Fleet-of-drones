#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
 
#define NUM_DRONI 3 // Numero massimo di droni
#define MAC_SIZE 6  // Numero di byte del MAC address da salvare

// Tabella per MAC address e stato di connessione
int droneTable[NUM_DRONI][3] = {0}; // Inizializzata a 0 (MAC e stato e mission state)
int ping[NUM_DRONI] = {0}; // 0 = NON HO ANCORA PINGATO, 1 = HO PINGATO
int missionStatus = 1; //0 = ATTERRATO, 1 = IN ATTESA, 2 = OPERATIVO 
uint8_t MAC_ADDRESS[NUM_DRONI][6];
int drone_Connected[NUM_DRONI][NUM_DRONI-1] = {0}; //MANTIENE I macint di ogni drone a cui un drone è connesso
int n = 0; //numero di droni connessi 
int m = 0; //numero di droni operativi 
int m_max = 0; //vatiabile di supporto per mostrare il numero di droni operativi sul web server
int slopeArr[NUM_DRONI] = {0}; // Array per slope inviate dai droni
int tempArr[NUM_DRONI] = {0}; //Array per temperatures inviate dai droni
int obstacleArr[NUM_DRONI] = {0}; //Array per ostacoli rilevati dai droni

// Timeout per heartbeat
unsigned long lastHeartbeatTime[NUM_DRONI] = {0};
const unsigned long heartbeatTimeout = 5000; // Timeout di 5 secondi
 
// Struttura dati da inviare (vuota, usata solo come heartbeat)
typedef struct struct_message {
  char message[32];
  uint8_t MAC_RICEVUTI[NUM_DRONI][6];
} struct_message;

typedef struct info_drone{
  char message[32];
  int mStatus;
  int droni_c[NUM_DRONI-1];
  int slope;
  int temperature;
  int obstacle;
} struct_info;

struct_message dataToSend;
struct_info receivedMessage;
 
//const char* ssid = "GenteX";
//const char* password = "lorenzoaba";
const char* ssid = "moto_g8_plus";
const char* password = "geforce1";
 
// Configurazione Web Server
WebServer server(80);
 
// Funzione per aggiungere un drone nella tabella se non esiste già
int addOrUpdateDrone(const uint8_t* mac, bool connected) {
  int macInt = 0;
  // Calcola un valore intero dalle prime MAC_SIZE cifre del MAC
  for (int i = 0; i < MAC_SIZE; i++) {
    macInt = macInt * 256 + mac[i];
  }
 
  // Cerca se il drone è già presente nella tabella
  for (int i = 0; i < NUM_DRONI; i++) {
    if (droneTable[i][0] == macInt) {
      // Aggiorna lo stato del drone
      droneTable[i][1] = connected ? 1 : 0;
      ping[i] = 0;  
      return i;
    }
  }
 
  // Se non è presente, aggiungilo alla tabella
  for (int i = 0; i < NUM_DRONI; i++) {
    if (droneTable[i][0] == 0) { // Slot vuoto trovato
      droneTable[i][0] = macInt;
      droneTable[i][1] = connected ? 1 : 0;
      ping[i] = 0;

      
      memcpy(MAC_ADDRESS[i], mac, 6);

      Serial.print("MAC Received : ");
      for(int k=0; k<6; k++) {
        Serial.printf("%02X", MAC_ADDRESS[i][k]);
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

      n++;
      return i;
    }
  }
 
  // Tabella piena
  Serial.println("Errore: tabella droni piena!");
  return -1;
}
 
// Funzione per gestire la ricezione dei dati ESP-NOW
void onDataRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  // Aggiungi o aggiorna il drone nella tabella
  int droneIndex = addOrUpdateDrone(info->src_addr, true);
  memcpy(&receivedMessage, data, sizeof(receivedMessage));
  if (droneIndex != -1) {
    if(strcmp(receivedMessage.message, "INFO") == 0){
      Serial.printf("INFO received from Drone %d\n", abs(droneTable[droneIndex][0]) % 100000);
      lastHeartbeatTime[droneIndex] = millis();
      droneTable[droneIndex][2] = receivedMessage.mStatus;
      slopeArr[droneIndex] = receivedMessage.slope;
      tempArr[droneIndex] = receivedMessage.temperature;
      obstacleArr[droneIndex] = receivedMessage.obstacle;
      if(n>1){
        for(int i=0; i<NUM_DRONI-1; i++){
          drone_Connected[droneIndex][i]=receivedMessage.droni_c[i];
          //Serial.printf("Connesso a %d\n", drone_Connected[droneIndex][i]);
        }
      }
      //se si è raggiunto il giusto numero di droni si invia il comando di OPERATIVO AI DRONI
      if(n>=1 && n<NUM_DRONI && missionStatus == 1){
        strcpy(dataToSend.message, "IN_ATTESA");
        esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
      }else if(n==NUM_DRONI){
        strcpy(dataToSend.message, "OPERATIVO");
        esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
      }else if(n==NUM_DRONI-2 && missionStatus == 2){ //da modificare il ==0 perchè al momento ho solo un drone
        strcpy(dataToSend.message, "ATTERRAGGIO");
        esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
      }else{  //altrimenti si invia il classico ACK
        strcpy(dataToSend.message, "ACK");
        esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
      }
    }else if(strcmp(receivedMessage.message, "PING") == 0){
      Serial.printf("PING received from Drone %d\n", abs(droneTable[droneIndex][0]) % 100000);
      lastHeartbeatTime[droneIndex] = millis();
      strcpy(dataToSend.message, "PING_PANIK");
      esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
    }else if(strcmp(receivedMessage.message, "CONNECTION") == 0){
      Serial.printf("CONNECTION request from drone %d\n", abs(droneTable[droneIndex][0]) % 100000);
      if(n==1){
        lastHeartbeatTime[droneIndex] = millis();
        strcpy(dataToSend.message, "IN_ATTESA");
        esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
      }else if(n>1){
        lastHeartbeatTime[droneIndex] = millis();
        strcpy(dataToSend.message, "PUBBLICAZIONE");
        for(int j=0; j<NUM_DRONI; j++){
          memcpy(dataToSend.MAC_RICEVUTI[j], MAC_ADDRESS[j], 6);
        }
        esp_now_send(info->src_addr, (uint8_t *)&dataToSend, sizeof(dataToSend));
        Serial.println("PUBBLICAZIONE sent");
      }
    }
  } else {
    Serial.println("Drone non aggiunto (tabella piena)");
  }
}


// Pagina web principale
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Torre di Controllo</title>";
  html += "<meta http-equiv='refresh' content='2'>"; // Ricarica ogni 2 secondi
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; }";
  html += ".drone-container { display: flex; justify-content: center; align-items: center; }";
  html += ".drone { display: flex; flex-direction: column; gap: 30px; margin: 10px; padding: 10px; background: #05203c; border: 2px solid #0062e3; border-radius: 10px; width: 300px; font-size: 26px;}";
  html += ".sub-header { font-size: 1.5em; margin-top: 10px; color: #555; }";
  html += ".top-section { flex: 2; height: 290px; display: flex; justify-content: space-between; align-items: center; background: #05203c; border-radius: 20px; border: 2px solid #0062e3; }";
  html += ".stat-section {flex: 1; padding: 5px; text-align: center; display: flex; flex-direction: column; gap: 20px; align-items: center; align-self: flex-start; margin-top: 40px; font-size: 1.4em }";
  html += ".stat-large {flex: 1; padding: 5px; display: flex; flex-direction: column; gap: 20px; text-align: center; align-items: center; align-self: flex-start; margin-top: 10px; font-size: 2em; }";
  html += ".mission-status { font-size: 2em; color: blue; }";
  html += ".container_titoli {display: inline-block; padding: 2px 20px; border-radius: 20px; text-align: center, margin-top: 5px; background: #05203c; border: 2px solid #ffffff; color: #ffffff; }";
  html += ".container_numeri {display: inline-block; padding: 15px 25px; border-radius: 20px; text-align: center, margin-top: 10px; background: #0062e3; color: #ffffff; font-size: 1.6em; font-weight: bold;}";
  html += ".container_mission_waiting {display: inline-block; padding: 2px 20px; border-radius: 20px; text-align: center, margin-top: 10px; background: #ff8a00; color: #ffffff; font-size: 1.3em; font-weight: bold;}";
  html += ".container_mission_progress {display: inline-block; padding: 2px 20px; border-radius: 20px; text-align: center, margin-top: 10px; background: #0062e3; color: #ffffff; font-size: 1.3em; font-weight: bold;}";
  html += ".container_mission_aborted {display: inline-block; padding: 2px 20px; border-radius: 20px; text-align: center, margin-top: 10px; background: #ff0000; color: #ffffff; font-size: 1.3em; font-weight: bold;}";
  html += ".drone_text {display: inline-block; padding: 2px 5px; border-radius: 20px; text-align: center, margin-top: 10px; background: #05203c; border: 2px solid #ffffff; color: #ffffff; font-size: 1.4em; font-weight: bold;}";
  html += ".ID_Container {display: inline-block; padding: 2px 5px; border-radius: 20px; text-align: center, margin-top: 10px; background: #05203c; border: 2px solid #ffffff; color: #ffffff; font-size: 1.2em; font-weight: bold;}";
  html += ".connected {display: inline-block; padding: 5px 5px; border-radius: 20px; text-align: center, margin-top: 10px; background: #42ff00; color: #ffffff; font-size: 1.2em; font-weight: bold;}";
  html += ".disconnected {display: inline-block; padding: 5px 5px; border-radius: 20px; text-align: center, margin-top: 10px; background: #ff0000; color: #ffffff; font-size: 1.2em; font-weight: bold;}";
  html += ".suspected {display: inline-block; padding: 5px 5px; border-radius: 20px; text-align: center, margin-top: 10px; background: #ff8a00; color: #ffffff; font-size: 1.2em; font-weight: bold;}";
  html += ".connected_drones_container { display: flex; flex-direction: column; gap: 10px; padding: 5 px; text-align: center; justify-content: center; margin-top: 5px; background: #0062e3; border-radius: 20px;}";
  html += ".size_font {color: #ffffff; font-size: 1.2em; font-weight: bold;}";
  html += ".size_font2 {color: #ffffff; font-size: 1.0em; }";
  html += "</style></head><body>";

  // Stato Missione con logica condizionale
  html += "<div class='top-section'>";
  html += "<div class='stat-section'>";
  html += "<span class='container_titoli'>";
  html += "<h2>Connected Drones</h2>";
  html += "</span>";
  html += "<span class='container_numeri'>" + String(n) + "</span>";
  html += "</div>";
  html += "<div class='stat-large'>";
  html += "<span class='container_titoli'>";
  html += "<h2>Mission Status</h2>";
  html += "</span>";
  if (missionStatus == 1) {
      html += "<span class='container_mission_waiting'>Waiting for Drones</span>";
  } else if (missionStatus == 2) {
      html += "<span class='container_mission_progress'>Mission in Progress</span>";
  } else if (missionStatus == 0) {
      html += "<span class='container_mission_aborted'>Mission Aborted</span>";
  } else {
      html += "Stato Missione Non Disponibile";
  }
  html += "</div>";
  html += "<div class='stat-section'>";
  html += "<span class='container_titoli'>";
  html += "<h2>Operational Drones</h2>";
  html += "</span>";
  html += "<span class='container_numeri'>" + String(m_max) + "</span>";
  html += "</p></div></div>";


  html += "<div class='drone-container'>";
  for (int i = 0; i < NUM_DRONI; i++) {
    html += "<div class='drone'>";
    html += "<span class='drone_text'>Drone " + String(i + 1) + "</span>";
    html += "<span class='ID_Container'>ID: " + String(abs(droneTable[i][0]) % 100000) + "</span>";
   
    // Determina lo stato del drone
    if (droneTable[i][1] == 1) {
      html += "<span class='connected'>Connected</span>";
    } else if (droneTable[i][1] == 2) {
      html += "<span class='suspected'>Suspected</span>";
    } else {
      html += "<span class='disconnected'>Not Connected</span>";
    }

    if (droneTable[i][2] == 1){
      html += "<span class='suspected'>Waiting</span>";
    }else if (droneTable[i][2] == 2){
      html += "<span class='connected'>Operative</span>";
    }else{
      html += "<span class='disconnected'>Not Operative</span>";
    }

    html += "<span class='ID_Container'>Slope: "+ String(slopeArr[i]) + "</span>";
    html += "<span class='ID_Container'>Temperature: "+ String(tempArr[i]) + "</span>";

    if (obstacleArr[i] == 1){
      html += "<span class='disconnected'>Obstacles Detected</span>";
    }else{
      html += "<span class='connected'>No Obstacles</span>";
    }

    if (n>=2){
      if(droneTable[i][0]!=0) {
        html += "<div class='connected_drones_container'>";
        html += "<span class='size_font'>ID Connected Drones </span>";
        for(int j=0; j<NUM_DRONI-1; j++){
          if(drone_Connected[i][j]!=0){
            html += "<span class='size_font2'>ID: " + String(abs(drone_Connected[i][j]) % 100000) + " </span>";
          }
        }
        html += "</div>"; 
      }
    }

    html += "</div>";
  }
 
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}
 
void setup() {
  Serial.begin(115200);
  delay(2000);
 
  // Inizializza ESP-NOW
  WiFi.mode(WIFI_STA);
 
  WiFi.begin(ssid, password); // Sostituisci con i tuoi dati Wi-Fi
 
  Serial.print("Connection to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
 
  Serial.print("Server IP Address: ");
  Serial.println(WiFi.localIP());
 
  int serverChannel = WiFi.channel();
  Serial.printf("Server Channel: %d\n", serverChannel);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Errore durante l'inizializzazione di ESP-NOW");
    return;
  }
  
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = serverChannel; // Imposta il canale Wi-Fi
  peerInfo.encrypt = false;
  
 
  esp_now_register_recv_cb(onDataRecv);
 
  // Configura il Web Server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web Server ready!");
}
 
void loop() {
  for (int i = 0; i < NUM_DRONI; i++) {   
    if (droneTable[i][0] != 0){
      if(droneTable[i][2] == 2){
        m++;
      }
    } 
    else if (droneTable[i][0]==0) lastHeartbeatTime[i] = millis();
    

    if (droneTable[i][0] != 0 && millis() - lastHeartbeatTime[i] > heartbeatTimeout) {
      if(ping[i] == 2){ //Resetta
        droneTable[i][2] = 0;
        droneTable[i][1] = 0;
        n--;
        lastHeartbeatTime[i] = millis();
        if (esp_now_del_peer(MAC_ADDRESS[i]) == ESP_OK) {
                Serial.printf("Peer %d removed\n", abs(droneTable[i][0]) % 100000);
        }else{
                Serial.println("Errore durante la rimozione del peer");
        }
        droneTable[i][0] = 0;
        memset(MAC_ADDRESS[i], 0, sizeof(MAC_ADDRESS[i]));
        for(int j=0; j<NUM_DRONI-1; j++) {
          drone_Connected[i][j]=0;
        }
      }else if(ping[i] == 1){  
        droneTable[i][1] = 2;
        ping[i] = 2;
        lastHeartbeatTime[i] = millis();  
      }else{    
        droneTable[i][1] = 2;
        ping[i] = 1;
        Serial.printf("Ping sent to the drone %d\n", abs(droneTable[i][0]) % 100000);
        strcpy(dataToSend.message, "PING");
        esp_now_send(MAC_ADDRESS[i], (uint8_t *)&dataToSend, sizeof(dataToSend));
        lastHeartbeatTime[i] = millis();
      }  
    }
  }

  m_max = m;

  if(m==NUM_DRONI){
    missionStatus = 2;
  }else if(n==NUM_DRONI-2 && missionStatus == 2){
    missionStatus = 0;
  }
  m=0;
  // Gestisci richieste del Web Server
  server.handleClient();
}

