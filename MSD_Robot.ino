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
//Pins
//Buttons (Interrupt)
#define BUTTON1 11
#define BUTTON2 12
// I2C for LCD
#define SDA 22
#define SCL 21
//Encoder (Interrupt & digital inputs)
#define A_1 0
#define A_2 1 
#define B_1 2 
#define B_2 3 
//Servo control (PWM Output)
#define SERVO_0 0 //Base
#define SERVO_1 1 //Component 1
#define SERVO_2 2 //Component 2
#define SERVO_3 3 //Component 3
#define SERVO_4 3 //Gripper Servo
// Motor control (BTS7960) (PWM Output)
#define MOTOR_A 2
#define MOTOR_B 3

//---------------------PWM CHANNEL--------------------------
#define MOTORA 0 
#define MOTORB 1
#define SERVO0 2 //Base
#define SERVO1 3 //Component 1
#define SERVO2 4 //Component 2
#define SERVO3 5 //Component 3
#define SERVO4 6 //Gripper Servo

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
  ledcSetup(MOTORA,1000,8);
  ledcSetup(MOTORB,1000,8);
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
