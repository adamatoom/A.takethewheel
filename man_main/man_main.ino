#include <Arduino.h>
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

using namespace std;
int diff = 26;
int baseV = 170;
float cont_out=0;
unsigned int CANL_pin=3;
unsigned int CANH_pin=5;
unsigned int manual_pin = 7;
unsigned int low_read_pin = A0;
unsigned int high_read_pin=A1;
char nb=0;
float I = 0;
float D1 = 0;
byte start = 1;
byte start1 = 1;
int contr;
signed char str_angle=0;
signed char req_str_angle=0;
signed char str_angle0=0;
signed char req_str_angle0=0;
byte wasmanual = 0;
int count = 0;
unsigned long startMillis;  
unsigned long currentMillis;
unsigned long oldmillis;
const unsigned long period = 1000;  //the value is a number of milliseconds
int newvoltageL = 60;
int newvoltageH = 190;


void setup() {
  //setting up pins
  pinMode(low_read_pin, INPUT_PULLUP);
  pinMode(high_read_pin, INPUT_PULLUP);
  pinMode(CANL_pin, OUTPUT);
  pinMode(CANL_pin, OUTPUT);
  pinMode(manual_pin, INPUT_PULLUP); 
  
  Serial.begin(115200);  
  Serial.println("CAN Read - Testing receival of CAN Bus message");  
  delay(1000);
  
  if(Canbus.init(CANSPEED_500))  //Initialise MCP2515 CAN controller at the specified speed
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
    
  delay(1000);
  startMillis = millis();  //initial start time
  startMillis = millis();  //initial start time
}


void loop() {

// getting the steering angle
    tCAN message;
if (mcp2515_check_message()) 
    {
      if (mcp2515_get_message(&message)) 
    {
        if(message.id == 0x2B0)  //uncomment when you want to filter
             {
                str_angle = message.data[1];
                if(start == 1){str_angle0 = str_angle;start = 0;}
             }
    }}
  byte count = 0;
  byte cont = 0;
  int i =0;
  
if(Serial.available()>0){
  
  cont_out = ((Serial.read())-127)/127.0;
  cont=1;
}
if(start1==0){
}
if(cont==1){  
}

float k =1;

//applying voltage
analogWrite(CANH_pin,int(baseV+diff*cont_out)); 
analogWrite(CANL_pin,int(baseV-diff*cont_out));
//
//if(millis()%100==0){
//  Serial.print("High  ");
//  Serial.print(int(baseV+diff*cont_out));
//  Serial.print("   Low  ");
//  Serial.print(int(baseV-diff*cont_out));
//  Serial.print("   ang ");
//  Serial.print(str_angle);
//  Serial.print("   out ");
//  Serial.println(cont_out);
//  Serial.print("   k ");
//  Serial.println(k);
//  
//  }


// reading the torque sensor value
  int sensorValueL = int(analogRead(low_read_pin)/6);
  int sensorValueH = int(analogRead(high_read_pin)/6);
  int manual = digitalRead(7);
  manual =1; //comment to activate manual switch
  
    
    
}
int clamp1(int a){if(a < -21){a=-21;}if(a > 20){a=20;} return a;}
float clamp2(float a){if(a > 0.95){a=0.95;}if(a < -0){a=0;} return a;}
float clamp3(float a){if(a > 0){a=0;}if(a < -0.95){a=-0.95;} return a;}