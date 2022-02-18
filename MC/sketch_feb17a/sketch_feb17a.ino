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

byte type = 0;
float P = 0;
unsigned long time1 = millis();
float I = 0;
float D = 0;
float p = 7.0 / 100.0;
float i = 3.0 / 200.0;
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

void setup(void)
{
  Serial.begin(115200);
  mySerial.begin(38400);

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
  get_can();

  if (type == 0)
  {
    output = (int)clamp((float)(-2047), (float)2047, PID());
  }
  if (type == 0 || type == 1)
  {
    if (ang[4] >= 0)
    {
      if (str < 3000 && str > 1095)
      {
        if (counter % 1 == 0)
        {
          str = (int)clamp(str0 - 4, str0 + 10, output);
        }
      }
      else
      {
        if (counter % 2 == 0)
        {

          str = (int)clamp(str0 - 4, str0 + 10, output);
        }
      }
    }
    else
    {
      if (str < 3000 && str > 1095)
      {
        if (counter % 1 == 0)
        {
          str = (int)clamp(str0 - 10, str0 + 4, output);
        }
      }
      else
      {
        if (counter % 2 == 0)
        {

          str = (int)clamp(str0 - 10, str0 + 4, output);
        }
      }
    }
  }
  mcp.setChannelValue(MCP4728_CHANNEL_A, clamp(0, 4095, 2047 - 1.0 / 5 * str));
  mcp.setChannelValue(MCP4728_CHANNEL_B, clamp(0, 4095, 2047 + 1.0 / 5 * str));

  str0 = str;
  counter++;
}

void get_can()
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
  if (count11 % 10 == 0)
  {
    mcp2515_send_message(&message);
  }
  count11++;
  if (mcp2515_get_message(&message))
  {
    if ((message.id == 0x2b0)) // Check message is the reply and its the right PID
    {
      /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
      combined = message.data[1] << 8 | message.data[0];
      ang[3] = ang[4];
      ang[2] = ang[3];
      ang[1] = ang[2];
      ang[0] = ang[1];
      ang[4] = -combined;

      //      Serial.print("Steering: ");
      //      Serial.println(combined);
      mySerial.write((byte)10);
    }
  }
}

void get_data()
{
  if (mySerial.available())
  {
    type = mySerial.read();
    //    Serial.println(type);
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
      rang[3] = rang[4];
      rang[2] = rang[3];
      rang[1] = rang[2];
      rang[0] = rang[1];
      rang[4] = ((packet[0] << 8) | packet[1]) - 4000;
      //      Serial.println("rang");
      //
      //      Serial.println(rang[4]);
      //      Serial.println(ang[4]);
      //      Serial.println(str);
      //      Serial.println(int(packet[0]));
      //      Serial.println(int(packet[1]));

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
      // Serial.println((int)output);
      Serial.println("torque");
      Serial.println(str);
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
      //    default:
      //      Serial.println("Invalid Type");
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
  dt = (float)(millis() - time1);
  time1 = millis();
  P = clamp(-1600.0, 1600.0, p * ((float)rang[4] - (float)ang[4]));
  I = clamp(-1400.0, 1400.0, I + (((float)rang[4] - (float)ang[4]) + (float)rang[3] - (float)ang[3]) * i * dt);
  D = clamp(-2000.0, 2000.0, (((float)rang[4] - (float)ang[4]) - ((float)rang[4] - (float)ang[1])) * d / dt / 4);
  if (counter % 200 == 0)
  {
    Serial.println("D   :  - ");
    Serial.println(D);
    Serial.println(rang[4]);
    Serial.println(ang[4]);
    Serial.println(rang[1]);
    Serial.println(ang[1]);
  }

  return P + I + D;
}
