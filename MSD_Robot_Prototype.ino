#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

//---- FUNCTION DECLARATION ------------------------------
double distance_as5600(); //distance for prototype robot
double distance(); //distance for real robot
void IRAM_ATTR ENC_ISR(); // encoder ISR
void IRAM_ATTR BUTTON1_ISR(); // to reset distance
void IRAM_ATTR BUTTON2_ISR(); // to run the robot cycle once
void send_position(float distance); // ESPNOW communication to laptop
void send_array(float position_array[], size_t array_length);// send an entire array one by one
void servo_control(int angle, int motorOut); // Control servo 
void motor_control(int direction, int speed); // Control motor
void LCD_disp(float distance);// display LCD
void buzzer(); // buzz the buzzer
void PID_execute(double target); // Trigger a PID cycle




//-----------AS5600---------------------------------------
#include <AS5600.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

AS5600 as5600;   
float initialPos =0.0;

//-----------PID---------------------------------------
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 0.0, Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//-----------ESPNOW---------------------------------------

//laptop esp32's MacAddress 
uint8_t laptopMacAddress[] = {0xA0, 0xB7, 0x65, 0x61, 0xC8, 0x8C};
esp_now_peer_info_t peerInfo;
//Array
float position_array[2000];
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
  Kp = PID_param.Kp;
  Ki = PID_param.Ki;
  Kd = PID_param.Kd;
  buzzer();
}

//------- PINS --------------------------------------------------
//Buttons (Interrupt)
#define BUTTON1 25
#define BUTTON2 26
//Buzzer 
#define BUZZ_PIN 23
// I2C for LCD & Servos
#define SDA 21
#define SCL 22
//Encoder (Interrupt & digital inputs)
#define ENC_A 18
#define ENC_B 19 
// Motor control (BTS7960) (PWM Output)
#define MOTOR_A 13
#define MOTOR_B 14
//leftover pins: 27,4,5
//---------------------PWM CHANNEL--------------------------
#define MOTORA 0 
#define MOTORB 1
//---------------------PCA9685------------------------------
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50
//Pins on PCA9685 board
#define SERVO_0 0 //Base
#define SERVO_1 1 //Component 1
#define SERVO_2 2 //Component 2
#define SERVO_3 3 //Component 3
#define SERVO_4 4 //Gripper Servo
//initialize object named servos
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
//-------------- LCD --------------------------------------
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
//--------------Encoder-------------------------------------
volatile int encoderValue =0;
//distance
float diameter =10.0;
float perimeter = diameter*PI;






//-----MAIN PROGRAM---------------------------------------------
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
  memcpy(peerInfo.peer_addr, laptopMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //---------------------PCA9685---------------------------------
  servos.begin();
  servos.setPWMFreq(FREQUENCY);
  // ----------- LCD -----------------------------------------------
  // initialize LCD
  lcd.begin();
  // turn on LCD backlight                      
  lcd.backlight();
  // ----------------------------------------------------------------
  //    PINMODES
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUZZ_PIN,OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(MOTOR_A,OUTPUT);
  pinMode(MOTOR_B,OUTPUT);

  //Connecting PWM channels
  //(channel,freq,res)
  ledcSetup(MOTORA,5000,10);
  ledcSetup(MOTORB,5000,10);
  //(pin, channel)
  ledcAttachPin(MOTOR_A,MOTORA); 
  ledcAttachPin(MOTOR_B,MOTORB);
  //start with buzzing off
  digitalWrite(BUZZ_PIN,HIGH);
  //start with motor off
  ledcWrite(MOTORA, 0);
  ledcWrite(MOTORB, 0);
  // -----Encoder------------------------------------------------------
  attachInterrupt(digitalPinToInterrupt(ENC_A), ENC_ISR, FALLING);
  //-------BUTTONS-----------------------------------------------------------
  attachInterrupt(digitalPinToInterrupt(BUTTON1), BUTTON1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2), BUTTON2_ISR, FALLING);
  //-------PID---------------------------------------------------------
  myPID.SetMode(AUTOMATIC);  // turn the PID on
  myPID.SetOutputLimits(0, 1023);//make the output 10 bit
  //-----AS5600----------------------------------------------------------
  as5600.begin(21, 22);
  initialPos = as5600.getCumulativePosition();
 }

  //--------------------------------------------------------------------
 void loop(){

 }
//--------------------------------------------------------------------







 //-------FUNCTION DEFINITIONS---------------------------------------------------------
double distance_as5600()
{
return ((as5600.getCumulativePosition()-initialPos)/4095.0)*perimeter;
}
double distance()
{
  return (encoderValue/3840)*perimeter;
}
void IRAM_ATTR ENC_ISR()
{
  if (digitalRead(ENC_B) == digitalRead(ENC_A)) {
    encoderValue--;
  } else {
    encoderValue++;
  }
}
void IRAM_ATTR BUTTON1_ISR()
{
  encoderValue =0;
}
void IRAM_ATTR BUTTON2_ISR()
{
  // BUTTON2
}

void send_position(float distance)
{
  // Set value to send
    Array.position = distance;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(laptopMacAddress, (uint8_t *) &Array, sizeof(Array));
}
void send_array(float position_array[], size_t array_length)
{
  for (int i =0; i < array_length; i++)
  {
    send_position(position_array[i]);
  }
}

void servo_control(int angle, int motorOut)//angle 0-180 
{
  // Convert to pulse width
  int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  int pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Motor
  servos.setPWM(motorOut, 0, pulse_width);
}

void motor_control(int direction, int speed)
{
  /*
  direction == 
  -1 means backward
  0 means stop,
  1 means forward,
  speed is 10 bit (0-1023)
  */
  if (direction ==1)//forward
  {
  ledcWrite(MOTORA, speed);
  ledcWrite(MOTORB, 0);
  }
  else if (direction ==-1)//backward
  {
  ledcWrite(MOTORA, 0);
  ledcWrite(MOTORB, speed);
  }
  else //stop
  {
  ledcWrite(MOTORA, 0);
  ledcWrite(MOTORB, 0);
  }
}

void LCD_disp(float distance)
{
  lcd.clear(); 
  // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Distance:");
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  lcd.print(String(distance));
}

void buzzer()
{
int interval = 200;//every 200ms
int prev_millis = millis();

digitalWrite(BUZZ_PIN,LOW);
while(millis() < prev_millis + interval){
  digitalWrite(BUZZ_PIN,LOW);
  }  
  prev_millis = millis();
while(millis() < prev_millis + interval){
  digitalWrite(BUZZ_PIN,HIGH);
  }
  prev_millis = millis();
while(millis() < prev_millis + interval){
  digitalWrite(BUZZ_PIN,LOW);
  }
  prev_millis = millis();
while(millis() < prev_millis + interval){
  digitalWrite(BUZZ_PIN,HIGH);
  }
  prev_millis = millis();
}
void PID_execute(double target)
{
  int startTime = millis();
  Setpoint = target;
  int prevMill =0;
  int index =0;
  myPID.SetTunings(Kp, Ki, Kd);
  while (millis() - startTime < 20000)//run PID for 20s
  {
  //for real robot
  // Input = distance();

  //for prototype
  Input = distance_as5600();

  if(millis() > prevMill+10)
  {
    position_array[index] = Input;
    index++;
    prevMill = millis();
  }

  if (target - Input > 0)
  {
    motor_control(1,myPID.Compute());
  }
  else if (target - Input < 0)
  {
    motor_control(-1,myPID.Compute());
  }
  else
  {
    motor_control(0,0);
    break;
  }
  }
  LCD_disp(Input);
  send_array(position_array, sizeof(position_array) / sizeof(position_array[0]));
}
//----------------------------------------------------------
