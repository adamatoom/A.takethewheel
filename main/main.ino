#include <Arduino.h>
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <memory.h>

using namespace std;
int diff = 26;
signed char sertmp=0;
int baseV = 170;
float cont_out=0;
char CANL_pin=3;
char CANH_pin=5;
char manual_pin = 7;
char low_read_pin = A0;
char high_read_pin= A1;
char nb=0;
float I = 0;
float D1 = 0;
byte start = 1;
byte start1 = 1;
int contr;
signed short int str_angle=0;
signed short int req_str_angle=0;
signed short int str_angle0=0;
signed short int req_str_angle0=0;
byte wasmanual = 0;
int count = 0;
unsigned long startMillis;  
unsigned long currentMillis;
unsigned long oldmillis;
const unsigned long period = 1000;  //the value is a number of milliseconds
int newvoltageL = 60;
int newvoltageH = 190;
char c=0;
char[4] buff=0;

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
		char buff[2] = {message.data[0],message.data[1]};
		memcpy(&str_angle,buff,sizeof(buff));
                //str_angle = message.data[1];
                if(start == 1){str_angle0 = str_angle;start = 0;}
             }
    }}
  byte count = 0;
  int i =0;
  buff = 0;
if(Serial.available()>0){
  char condition = Serial.read();
  if(condition =='t'){
    sertmp = Serial.read();
    cont_out = sertmp/127.0;
    start1=1;
  }else{
    if(condition =='a'){
      c = Serial.read();
      while(c != 'e'){
        buff[count] = c;
        c = Serial.read();
        count = count + 1;
      }
      req_str_angle = atof(buff);  

      start1=0;
    }
  }  
  
}

if(start1==0){ //making sure the the variables of the previous angles and required angles are correctly set before applying PID
cont_out = PID(float(req_str_angle),float(str_angle));
}

if(start1 == 1){req_str_angle0 = req_str_angle;start1 = 0;}
 

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
//  Serial.print("   reqang ");
//  Serial.print(req_str_angle);
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
float PID(float reqang,float ang){
  float Ip=0;
  float out=0;             
  float P = 0;
  
  
  I=I+Ip;
  
  if(abs(int(ang-reqang))>4){I=0;} //zero integral part to insure no overshoot in large angle differences
  
    
  oldmillis=millis();str_angle0=ang;req_str_angle0=reqang;//set the variables for the next run
  out = P+I;
  if( out>1){return 1;}else{if(out<-1){return -1;}else{return out;}}
    
  

}
