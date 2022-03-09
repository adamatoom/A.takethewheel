
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <CAN.h>
long id = 0;
byte byteLow = 0;
byte byteHigh = 0;
byte byterate = 0;
float rate1 = 0;
float rate0 = 0 ;
int got_can = 0;
int got_can1 = 0;
float penalty = 2.0;
float penalty1 = 1.0;
float pid_penalty = 1.0;
byte type = 1;
float rang1 = 0;
float pid1 = 0;
float pidoutput = 0;
float P = 0;
float P1 = 0;
unsigned long time1[5] = {millis(), millis(), millis(), millis(), millis()};
float I = 0;
float D = 0;
float p = 20.0 / 100.0;
float i = 300.0 / 200.0;
float d = 0;
float dt = 1000000;
int counter = 0;
int pid = 0;
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
short int combined = 0;
short int w_spd = 0;
float rate[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

SoftwareSerial serial1(8, 9); // RX, TX
Adafruit_MCP4728 mcp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }else{
    Serial.println("Starting CAN Done");
    //CAN.filter(0x2);
  // register the receive callback
  CAN.onReceive(get_can);
  }
//  serial2.begin(19200);

  delay(1000);

  while (!mcp.begin())
  {
    Serial.println("Failed to find MCP4728 chip");

      delay(10);
  }
      Serial.println("P, I, rate, pid, anf, rang, D");

  
  mcp.setChannelValue(MCP4728_CHANNEL_A, clamp(0, 4095, 2047));
  mcp.setChannelValue(MCP4728_CHANNEL_B, clamp(0, 4095, 2047));
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  str = (int)clamp((float)(-2047), (float)2047, mea((float)output, (float)str, 0.5));
  mcp.setChannelValue(MCP4728_CHANNEL_A, clamp(0, 4095, 2047 - 1.0 / 5 * str));
  mcp.setChannelValue(MCP4728_CHANNEL_B, clamp(0, 4095, 2047 + 1.0 / 5 * str));
  counter++;
}


void get_can(int packetSize)
{
  id = CAN.packetId();
  if (id == 0x2b0)
  {
    if (CAN.available())
    {
      got_can = 1;
      byteLow = CAN.read();
      byteHigh = CAN.read();
      byterate = CAN.read();
      
      combined = ((byteHigh << 8) + byteLow);
      rang[0] = rang[1];
      rang[1] = rang[2];
//      if (w_spd < 15){
//      rang[2] = mea(-clamp(-4000,4000,(analogRead(A0)-512)*9), rang[1], 0.5);}
//      else{
      rang[2] = mea(clamp(-4000,4000,-(analogRead(A0)-512)*24/(2+w_spd/10)), rang[1], 0.5);
      rang[2] = mea(clamp(-4000,4000,-(analogRead(A0)-512)*8), rang[1], 0.5);
//      }
      ang[0] = ang[1];
      ang[1] = ang[2];
      ang[2] = -combined;
      penalty = 2;
      rate[0] = rate[1];
      rate[1] = rate[2];
      if (ang[2] >= ang[1])
      {
        rate[2] = mea(byterate, rate[1], 0.4);
      }
      else
      {
        rate[2] = mea(-byterate, rate[1], 0.4);
      }
  
      time1[0] = time1[1];
      time1[1] = time1[2];
      time1[2] = millis();
//      if (got_can && got_can1){
      pidoutput = PID();
//      got_can = 0;
//      got_can1 = 0;
//      }
      
      output = (int)clamp((float)(-2047), (float)2047, pidoutput);

    }}
  if (id == 0x4b0)
  {
    got_can1 = 1;
    if (CAN.available())
    {
      byteLow = CAN.read();
      byteHigh = CAN.read();
      w_spd = byteHigh << 8 | byteLow;
    w_spd = w_spd / 10.0;
    if (w_spd < 1)
    {
      penalty1 = 1;
    }
    else
    {
      penalty1 = 40.0 / (40.0 + w_spd/4);
    }
    }
  }

}
float PID()
{
  dt = (float)(time1[2] - time1[1]);
  P = clamp((float)-1500, (float)1500, (float)4 / (float)3 * ((float)rang[2] - (float)ang[2]));
  I = clamp((float)-2000, (float)2000, I + 2 / (float)2 * (((float)rang[2] - (float)ang[2]) + ((float)rang[1] - (float)ang[1]))*(dt/(float)1000));
  D = clamp((float)-800, (float)800, mea(1 / (float)10 * (((float)rang[2] - (float)ang[2]) - ((float)rang[1] - (float)ang[1]))/(dt/(float)1000), D, 0.4));
  
  Serial.print(P);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(rate[2]);
  Serial.print(",");
  Serial.print(pidoutput);
  Serial.print(",");
  Serial.print(ang[2]);
  Serial.print(",");
  Serial.print(rang[2]);
  Serial.print(",");
  Serial.println(D);
  return P + I + D;
//  return 0;
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
float mea(float input, float average, float filter1){
  output = (input * (1 - filter1) + average * filter1);
  return output;
}
