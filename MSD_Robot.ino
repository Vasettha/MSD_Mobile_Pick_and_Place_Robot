#include <esp_now.h>
#include <WiFi.h>


//--------------------------------------------------------

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
  memcpy(&PID_param, incomingData, sizeof(PID_param));
  //Action on data received
}

//----------------------------------------------------------
//Pin Numbering
#define LEFT1 14  
#define LEFT2 12
#define RIGHT1 17
#define RIGHT2 16
//pwm attached channel
#define LEFT1_C 0 
#define LEFT2_C 1 
#define RIGHT1_C 2 
#define RIGHT2_C 3 

//----------------------------------------------------------


void setup() {
  //---------------------    COMMUNICATION   -----------------------
  Serial.begin(115200);
  //Set WiFi mode to Station
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

  // ----------------------------------------------------------------
  //    PINMODES
  pinMode(LEFT1,OUTPUT);
  pinMode(LEFT2,OUTPUT);
  pinMode(RIGHT1,OUTPUT);
  pinMode(RIGHT2,OUTPUT);

  //Connecting PWM channels
  //(channel,freq,res)
  ledcSetup(LEFT1_C,1000,8);
  ledcSetup(LEFT2_C,1000,8);
  ledcSetup(RIGHT1_C,1000,8);
  ledcSetup(RIGHT2_C,1000,8);
  //(pin, channel)
  ledcAttachPin(LEFT1,LEFT1_C); 
  ledcAttachPin(LEFT2,LEFT2_C);
  ledcAttachPin(RIGHT1,RIGHT1_C);
  ledcAttachPin(RIGHT2,RIGHT2_C);
  //--------------------------------------------------------------------
 }
 //--------------------------------------------------------------------

void send_array()
{
  // // Set value to send
  //   Array.index = 0;
  //   Array.position = 0.0;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Array, sizeof(Array));
}

 void loop(){
 //unused
 }