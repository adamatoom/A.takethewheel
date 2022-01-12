/****************************************************************************
CAN-Bus Demo

Toni Klopfenstein @ SparkFun Electronics
September 2015
https://github.com/sparkfun/CAN-Bus_Shield

This example sketch works with the CAN-Bus shield from SparkFun Electronics.

It enables the MCP2515 CAN controller and MCP2551 CAN-Bus driver, and demos
using the chips to communicate with a CAN-Bus.

Resources:

Development environment specifics:
Developed for Arduino 1.6.5

Based off of original example ecu_reader_logger by:
Sukkin Pang
SK Pang Electronics www.skpang.co.uk

This code is beerware; if you see me (or any other SparkFun employee) 
at the local, and you've found our code helpful, please buy us a round!

For the official license, please check out the license file included with the library.

Distributed as-is; no warranty is given.
*************************************************************************/

#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <SoftwareSerial.h>
#define STEERING 0x2b0
#define WHLSPD 0x4b0

SoftwareSerial mySerial(5, 6); // RX, TX
char buff[2];
char UserInput;
int data;
char buffer[456]; //Data will be temporarily stored to this buffer before being written to the file
signed int combined = 5000;
//********************************Setup Loop*********************************//

void setup()
{
  Serial.begin(115200);
  mySerial.begin(38400);
  Serial.println("CAN-Bus Demo");

  if (Canbus.init(CANSPEED_500)) /* Initialise MCP2515 CAN controller at the specified speed */
  {
    Serial.println("CAN Init ok");
  }
  else
  {
    Serial.println("Can't init CAN");
  }
  //
  //  delay(1000);
  //
  //Serial.println("Please choose a menu option.");
  //Serial.println("1.Speed");
  //Serial.println("2.RPM");
  //Serial.println("3.Throttle");
  //Serial.println("4.Coolant Temperature");
  //Serial.println("5.O2 Voltage");
  //Serial.println("6.MAF Sensor");
}

//********************************Main Loop*********************************//

int count11 = 0;
void loop()
{
  //
  //while(Serial.available()){
  //   UserInput = Serial.read();
  //if (UserInput=='1'){
  // Canbus.ecu_req(STEERING, buffer);
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
    count11++;
  }
  if (mcp2515_get_message(&message))
  {
    if ((message.id == 0x2b0)) // Check message is the reply and its the right PID
    {
      /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
      combined = message.data[1] << 8 | message.data[0];
      combined = combined + 5000;
      buff[1] = combined >> 8 & 0xff;
      buff[0] = combined >> 0 & 0xff;

      Serial.print("Steering: ");
      Serial.println(combined);
    }
  }
  //}
  mySerial.write(buff, 2);
  Serial.println("done");
  Serial.println(buff[0]);
  Serial.println(buff[1]);

  //}
}
