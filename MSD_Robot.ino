#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

//-----------PID---------------------------------------
double Setpoint, Input, Output;
double Kp = 5.0, Ki = 2.0, Kd = 1.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//-----------Inverse Kinematics---------------------------------------
//replace with the proper initial condition
int num_step = 100;
double init_X = 0.0;
double init_Y = 0.0;
double init_Z = 0.0;
double desired_X,desired_Y,desired_Z;
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
//Laptop_comm
// this will be received by the robot to update the PID parameters or IK coordinates
typedef struct {
float F; // 1 means PID, 2 means IK
float Kp;// or X
float Ki;// or Y
float Kd;// or Z
} Laptop_message;

Laptop_message Laptop_comm;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//To act on the received message
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Laptop_comm, incomingData, sizeof(Laptop_comm));
  if (Laptop_comm.F == 1.0){
  Kp = Laptop_comm.Kp;
  Ki = Laptop_comm.Ki;
  Kd = Laptop_comm.Kd;
  }
  else if (Laptop_comm.F == 2.0)
  {
    desired_X = Laptop_comm.Kp;
    desired_Y = Laptop_comm.Ki;
    desired_Z = Laptop_comm.Kd;
  }
  buzzer();
}

//------- PINS --------------------------------------------------
//Buttons (Interrupt)
#define BUTTON1 25
#define BUTTON2 26
#define BUTTON3 27
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
//leftover pins: ,4,5
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

 //-------FUNCTION DEFINITIONS---------------------------------------------------------

double distance()
{
  return (encoderValue/3840.0)*perimeter;
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
void IRAM_ATTR BUTTON3_ISR()
{
  // BUTTON3
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
  Serial.println("Speed: " + String(speed));
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
  Serial.println("Setpoint: "+ String(Setpoint));
  int prevMill =0;
  int index =0;
  myPID.SetTunings(Kp, Ki, Kd);
  while (millis() - startTime < 20000)//run PID for 20s
  {
  //for real robot
  // Input = distance();

  //for prototype
  Input = distance();
  Serial.println("Input: "+ String(Input));
  if(millis() > prevMill+10)
  {
    position_array[index] = Input;
    index++;
    prevMill = millis();
  }
  myPID.Compute();
  if (Output < 0)
  {
    motor_control(1,abs(Output));
  }
  else if (Output > 0)
  {
    motor_control(-1,abs(Output));
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

void IK_execute()
{
  //Prepare the Coordinate steps
  double increment_X[num_step];
  double increment_Y[num_step];
  double increment_Z[num_step];
  double step_X = (init_X - desired_X)/num_step;
  double step_Y = (init_Y - desired_Y)/num_step;
  double step_Z = (init_Z - desired_Z)/num_step;
  for (int i; i < num_step;i++)
  {
    increment_X[i] = init_X - (step_X*(i+1));
    increment_Y[i] = init_Y - (step_Y*(i+1));
    increment_Z[i] = init_Z - (step_Z*(i+1));
  }
  //IK
  //take in X,Y,Z and output servo control 0,1,2,3
  double Servo_0[num_step];
  double Servo_1[num_step];
  double Servo_2[num_step];
  double Servo_3[num_step];

  //Take servo control array and use it to 
  for (int i;i < num_step;i++)
  {
    servo_control(Servo_0[i],0);
    servo_control(Servo_1[i],1);
    servo_control(Servo_2[i],2);
    servo_control(Servo_3[i],3);
    delay(i);
  }
}
//----------------------------------------------------------




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
  attachInterrupt(digitalPinToInterrupt(BUTTON3), BUTTON3_ISR, FALLING);
  //-------PID---------------------------------------------------------
  myPID.SetMode(AUTOMATIC);  // turn the PID on
  myPID.SetOutputLimits(-1023, 1023);//make the output 10 bit

 }

  //--------------------------------------------------------------------
 void loop(){
  delay(1000);
  Serial.println("PID start");
  PID_execute(100.0);
  Serial.println("PID done");

  delay(100000);

 }
