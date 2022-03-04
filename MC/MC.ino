#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <SoftwareSerial.h>
#include <CAN.h>

#define STEERING 0x2b0
#define WHLSPD 0x4b0
#define mcp1 0x60
#define mcp2 0x61

SoftwareSerial mySerial(8, 9); // RX, TX

Adafruit_MCP4728 mcp;

long id = 0;
int got_can = 0;
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
float i = 15.0 / 200.0;
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
signed int combined = 0;
signed int w_spd = 0;
float rate[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup(void)
{
  Serial.begin(115200);
  mySerial.setTimeout(1);
  mySerial.begin(9600);

  if (!CAN.begin(500E3))
  {
    Serial.println("Starting CAN failed!");
  }
  else
  {
    Serial.println("Starting CAN Done");
  }

  if (!mcp.begin())
  {
    Serial.println("Failed to find MCP4728 chip");
  }

  Serial.println("P, I, rate, pid, rang, penalty, ang");
  mySerial.write((byte)1);
}

void loop()
{
  get_data();
  if (type == 0)
  {
    get_can();
  }

  str = clamp((-2047), 2047, clamp(str - 40, str + 40, output));
  mcp.setChannelValue(MCP4728_CHANNEL_A, clamp(0, 4095, 2047 - 1.0 / 5 * str));
  mcp.setChannelValue(MCP4728_CHANNEL_B, clamp(0, 4095, 2047 + 1.0 / 5 * str));

  counter++;
}

void get_data()
{
  if (mySerial.available())
  {
    type = mySerial.read();
    Serial.println(type);

    switch (type)
    {
    case (byte)0:
      while (mySerial.available() < 2)
      {
      }

      packet[0] = mySerial.read();
      packet[1] = mySerial.read();
      mySerial.write((byte)1);
      rang[0] = rang[1];
      rang[1] = rang[2];
      rang[2] = rang[3];
      rang[3] = rang[4];
      rang[4] = ((packet[0] << 8) | packet[1]) - 4000;
      rang[4] = rang[4] / 4.0;
      break;

    case (byte)1:
      while (mySerial.available() < 1)
      {
      }
      packet[0] = mySerial.read();
      mySerial.write((byte)1);
      output = ((int)packet[0] - 127) * 16;
      break;

    case (byte)3:
      while (mySerial.available() < 3)
      {
      }
      packet[0] = mySerial.read();
      packet[1] = mySerial.read();
      packet[2] = mySerial.read();
      mySerial.write((byte)1);
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
  rang1 = clamp(rang1 - 20.0, rang1 + 20.0, rang[4]);
  P1 = clamp((float)-20.0, (float)20.0, 1 / 10.0 * ((float)rang1 - (float)ang[4]));
  Serial.print(P1 * 20);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print((rate[10] * 10.0 + rate[9] * 5.0 + rate[8] * 2.5 + rate[7] * 1.75 + rate[6] * 0.825 + rate[5] * 0.4125 + rate[4] * 0.20625 + rate[3] * 0.103125) * 5.0);
  Serial.print(",");
  Serial.print(pid1 * 100);
  Serial.print(",");
  Serial.print(rang1);
  Serial.print(",");
  Serial.print(penalty * 200);
  Serial.print(",");
  Serial.println(ang[4]);
  return P1;
}
float PID1(float pid)
{
  if (pid1 <= 0)
  {
    float pid_penalty = 60 / (15 - 4.0 * pid1);
  }
  else
  {
    float pid_penalty = 60 / (15 + 4.0 * pid1);
  }
  pid1 = clamp(pid1 - 0.5 * pid_penalty, pid1 + 0.5 * pid_penalty, pid);
  float rate1 = (rate[10] * 10.0 + rate[9] * 5.0 + rate[8] * 2.5 + rate[7] * 1.75 + rate[6] * 0.825 + rate[5] * 0.4125 + rate[4] * 0.20625 + rate[3] * 0.103125) / 20.8;
  I = clamp(-2000.0, 2000.0, I + ((pid1 - (rate1))) * i * ((float)4000 / ((float)2000 + (float)abs((double)I))) * penalty1 * ((float)(time1[4] - time1[3])));
  return (I);
}
void get_can()
{
  while (got_can == 0)
  {
    CAN.parsePacket();
    id = CAN.packetId();
    if (id == 0x2b0)
    {
      got_can = 1;
      if (CAN.available())
      {
        byte byteLow = CAN.read();
        byte byteHigh = CAN.read();
        byte byterate = CAN.read();

        combined = byteHigh << 8 | byteLow;

        ang[0] = ang[1];
        ang[1] = ang[2];
        ang[2] = ang[3];
        ang[3] = ang[4];
        ang[4] = -combined;
        penalty = 2;
        if (ang[4] <= 0)
        {
          penalty = (12000) / (3000 - 8.0 * ang[4]);
        }
        else
        {
          penalty = (12000) / (3000 + 8.0 * ang[4]);
        }
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
        if (ang[4] >= ang[3])
        {
          rate[10] = byterate;
        }
        else
        {
          rate[10] = -byterate;
        }

        time1[0] = time1[1];
        time1[1] = time1[2];
        time1[2] = time1[3];
        time1[3] = time1[4];
        time1[4] = millis();
        if (type == 0)
        {
          pidoutput = PID();
          output = (int)clamp((float)(-2047), (float)2047, PID1(clamp((float)(-90), (float)90, pidoutput)));
        }
      }
    }
    else if (id == 0x4b0)
    {
      byte byteLow = CAN.read();
      byte byteHigh = CAN.read();
      w_spd = byteHigh << 8 | byteLow;
      w_spd = w_spd / 10.0;
      if (w_spd < 1)
      {
        penalty1 = 1;
      }
      else
      {
        penalty1 = 40.0 / (80.0 + w_spd);
      }
      penalty1 = 1;
    }
  }
  got_can = 0;
}
