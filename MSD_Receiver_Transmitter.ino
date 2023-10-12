#include <esp_now.h>
#include <WiFi.h>

//--------------------------------------------------------
float Position_array[2000];
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x4F, 0xDC, 0xEC};
esp_now_peer_info_t peerInfo;
//Array
// this will be sent from the robot everytime the PID function is ran
typedef struct {
float position;
} Array_message;

Array_message Array;
//PID_param
// this will be received by the robot to update the PID parameters
typedef struct {
float Kp;
float Ki;
float Kd;
} PID_message;

PID_message PID_param;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//To act on the received message
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Array, incomingData, sizeof(Array));
  //Action on data received
  Serial.print(Array.position);
  Serial.print(',');
}

//----------------------------------------------------------
// Global Variables and MACROS
  float Kp, Ki, Kd;



//----------------------------------------------------------
void setup() {
  //---------------------    COMMUNICATION   -----------------------
  Serial.begin(115200);
  //Set WiFi mode to Stationa
  WiFi.mode(WIFI_STA);
  //Initialize esp_now
    if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
    }
    
  esp_now_register_recv_cb(OnDataRecv);//When a message is received, run OnDataRecv()
  esp_now_register_send_cb(OnDataSent);//get status on transmitted packet

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //--------------------------------------------------------------------
 }
 //--------------------------------------------------------------------

void send_param(float Kp, float Ki, float Kd)
{
  // Set value to send
    PID_param.Kp= Kp;
    PID_param.Ki= Ki;
    PID_param.Kd= Kd;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &PID_param, sizeof(PID_param));
}
//-----------------------------------------------------------------------
void loop(){
  if (Serial.available() >= sizeof(float) * 3) {
    Serial.readBytes((char*)&Kp, sizeof(Kp));
    Serial.readBytes((char*)&Ki, sizeof(Ki));
    Serial.readBytes((char*)&Kd, sizeof(Kd));
    send_param(Kp, Ki, Kd);
  }
 }
