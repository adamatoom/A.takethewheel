#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <SoftwareSerial.h>
#define STEERING 0x2b0
#define WHLSPD 0x4b0

SoftwareSerial mySerial(5, 6); // RX, TX

Adafruit_MCP4728 mcp;

int got_can = 0;
float penalty = 1;
float penalty1 = 1;
byte type = 1;
float rang1 =0;
float pid1 =0;
float pidoutput = 0;
float P = 0;
float P1 = 0;
unsigned long time1[5] = {millis(), millis(), millis(), millis(), millis()};
float I = 0;
float D = 0;
float p = 20.0 / 100.0;
float i = 15.0 / 200.0;
float d = 0;
float dt = 1000000;
int counter = 0;
byte pid = 0;
int str = 0;
int output = 0;
int str0 = 0;
byte count;
int ang[5] = {0, 0, 0, 0, 0};
int rang[5] = {0, 0, 0, 0, 0};
char buff[2];
char buffer[456];
unsigned char packet[255];
int count11 = 0;
signed int combined = 0;
signed int w_spd = 0;
float rate[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup(void)
{
  Serial.begin(115200);
  mySerial.setTimeout(1);
  mySerial.begin(19200);

  if (Canbus.init(CANSPEED_500)) /* Initialise MCP2515 CAN controller at the specified speed */
  {
   Serial.println("CAN Init ok");
  }
  else
  {
    Serial.println("Can't init CAN");
  }
 Serial.println("Adafruit MCP4728 test!");

  if (!mcp.begin())
  {
    Serial.println("Failed to find MCP4728 chip");
    while (1)
    {
      delay(10);
    }
  }
}

void loop()
{
  get_data();
  got_can = get_can();
  if (type == 0 && got_can == 1)
  {
    pidoutput = PID();
    output = (int)clamp((float)(-2047), (float)2047, PID1(clamp((float)(-90), (float)90, pidoutput)));
  }
  str = clamp((-2047), 2047, clamp(str0-20,str0+20,output));
  mcp.setChannelValue(MCP4728_CHANNEL_A, clamp(0, 4095, 2047 - 1.0 / 5 * str));
  mcp.setChannelValue(MCP4728_CHANNEL_B, clamp(0, 4095, 2047 + 1.0 / 5 * str));

  str0 = str;
  counter++;
}

int get_can()
{
  tCAN message;

  message.id = PID_REQUEST; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 8; //formatted in DEC
  message.data[0] = 0x02;
  message.data[1] = 0x01;
  message.data[2] = STEERING;
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;

  mcp2515_bit_modify(CANCTRL, (1 << REQOP2) | (1 << REQOP1) | (1 << REQOP0), 0);
  count11++;
  if (mcp2515_get_message(&message))
  {if ((message.id == 0x4B0)) // Check message is the reply and its the right PID
    {
      /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
      w_spd = message.data[1] << 8 | message.data[0];
      w_spd = w_spd / 10.0;

      if (w_spd < 1){
       
      penalty1= 1; 
      }else{
        
      penalty1 = 40.0/(80.0 + w_spd);
      }
      penalty1 = 1;

      
    }
    if ((message.id == 0x2b0)) // Check message is the reply and its the right PID
    {
      /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
      combined = message.data[1] << 8 | message.data[0];
      
      
      ang[0] = ang[1];
      ang[1] = ang[2];
      ang[2] = ang[3];
      ang[3] = ang[4];
      ang[4] = -combined;
      
      rate[0] = rate[1];
      rate[1] = rate[2];
      rate[2] = rate[3];
      rate[3] = rate[4];
      rate[4] = rate[5];
      rate[5] = rate[6];
      rate[6] = rate[7];
      rate[7] = rate[8];
      rate[8] = rate[9];
      rate[9] = rate[10];
      if (ang[4] >= ang[3]){
      rate[10] = message.data[2];  
      }else{
      rate[10] = -message.data[2];    
      }

    
      time1[0] = time1[1];
      time1[1] = time1[2];
      time1[2] = time1[3];
      time1[3] = time1[4];
      time1[4] = millis();
      mySerial.write((byte)10);
      return 1;
    }
  }
  return 0;
}

void get_data()
{
  if (mySerial.available())
  {
    type = mySerial.read();
    switch (type)
    {
    case (byte)0:
      if (mySerial.available() > 1)
      {
        for (int i = 0; i < 2; i++)
        {
          packet[i] = mySerial.read();
        }
        mySerial.write((byte)10);
      }
      //code to be executed;
      rang[0] = rang[1];
      rang[1] = rang[2];
      rang[2] = rang[3];
      rang[3] = rang[4];
      rang[4] = ((packet[0] << 8) | packet[1]) - 4000;

      break;
    case (byte)1:
      if (mySerial.available() > 0)
      {
        for (int i = 0; i < 1; i++)
        {
          packet[i] = mySerial.read();
        }
        mySerial.write((byte)10);
      }
      //code to be executed
      output = ((int)packet[0] - 127) * 16;
      pid = 0;
      break;
    case (byte)3:
      if (mySerial.available() > 2)
      {
        for (int i = 0; i < 3; i++)
        {
          packet[i] = mySerial.read();
        }
        mySerial.write((byte)10);
      }
      //code to be executed
      p = float(packet[0]) / 100.0;
      i = float(packet[1]) / 200.0;
      d = float(packet[2]) / 2.0;
      Serial.println("PID");
      Serial.println(p);
      Serial.println(i);
      Serial.println(d);
      break;
    }
  }
}

byte clamp(byte lower, byte higher, byte input)
{
  if (input < lower)
  {
    input = lower;
  }
  if (input > higher)
  {
    input = higher;
  }
  return input;
}
int clamp(int lower, int higher, int input)
{
  if (input < lower)
  {
    input = lower;
  }
  if (input > higher)
  {
    input = higher;
  }
  return input;
}
float clamp(float lower, float higher, float input)
{
  if (input < lower)
  {
    input = lower;
  }
  if (input > higher)
  {
    input = higher;
  }
  return input;
}
float PID()
{
  dt = (float)(time1[4] - time1[3]);
  rang1 = clamp(rang1 - 15.0,rang1 + 15.0,rang[4]);
  P1 = clamp((float)-15.0, (float)15.0, 1 / 10.0 * ((float)rang1 - (float)ang[4]));
  float D1 = clamp((float)-20.0, (float)10.0,5.0* (ang[4] - ang[0])/((float)time1[4]-(float)time1[0]));
  if (counter % 200 == 0)
  {
  }
  return P1 + D1;
}
float PID1(float pid)
{
  pid1 = clamp(pid1 - 0.5,pid1 + 0.5,pid);
  float rate1 = (rate[10] * 10.0 + rate[9] * 5.0 + rate[8] * 2.5 + rate[7] * 1.75 + rate[6] * 0.825 + rate[5] * 0.4125 + rate[4] * 0.20625 + rate[3] * 0.103125)/20.8;

  I = clamp(-2000.0, 2000.0, I + ((pid - (rate1) )) * i*penalty1 * ((float)(time1[4] - time1[3])));

  return (I);
}
