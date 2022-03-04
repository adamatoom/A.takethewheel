#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

#define MYPORT_TX 14 //D5
#define MYPORT_RX 13 //D7

#define WIFI_SSID "Internet"
#define WIFI_PASS "adam1999"
//#define WIFI_SSID "ZWnet"
//#define WIFI_PASS "Zolom.123"
#define UDP_PORT 4210

WiFiUDP UDP;
SoftwareSerial mySerial;

char buff[2];
char packet[255];
byte type = 0;
byte count;
int ang[5] = {0, 0, 0, 0, 0};
int rang[5] = {0, 0, 0, 0, 0};
float p = 7.0 / 400.0;
float i = 3.0 / 200000.0;
float d = 0;
char reply[] = "Packet received!";
int counter = 0;
int str = 127;
byte output = 127;
int str0 = 127;
double Setpoint, Input, Output;
byte pid = 0;
byte ready1 = 0;
int len = 0;

void setup()
{
  // Set your Static IP address
  //  IPAddress local_IP(192, 168, 20, 63);
  //  // Set your Gateway IP address
  //  IPAddress gateway(192, 168, 20, 227);
  //
  //  IPAddress subnet(255, 255, 255, 0);
  // Setup serial port
  Serial.begin(115200);
  mySerial.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  if (!mySerial)
  {
    Serial.println("NO");
  }

  // Begin WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Connecting to WiFi...
  //  Serial.print("Connecting to ");
  //  Serial.print(WIFI_SSID);
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    //    Serial.print(".");
  }

  //  Connected to WiFi
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Begin listening to UDP port
  UDP.begin(UDP_PORT);
  //  Serial.print("Listening on UDP port ");
  //  Serial.println(UDP_PORT);
  while (ready1 != 1)
  {
    if (mySerial.available())
    {
      ready1 = mySerial.read();
    }
  }
}

void loop()
{
  if (!ready1)
  {
    if (mySerial.available())
    {
      ready1 = mySerial.read();
    }
  }
  counter++;
  // put your main code here, to run repeatedly:

  int packetSize = UDP.parsePacket();
  if (packetSize)
  {
    Serial.println("UDP1");
    len = UDP.read(packet, 255);
  }
  if (len > 0 && ready1)
  {
    len = 0;
    ready1 = 0;
    Serial.println("UDP2");
    type = packet[0];
    mySerial.write(type);
    Serial.println((int)type);
    switch (type)
    {
    case (byte)0:
      //code to be executed;
      mySerial.write(packet[1]);
      mySerial.write(packet[2]);

      break;
    case (byte)1:
      //code to be executed
      mySerial.write(packet[1]);
      break;
    case (byte)3:
      //code to be executed
      mySerial.write(packet[1]);
      mySerial.write(packet[2]);
      mySerial.write(packet[3]);
      break;
    default:
      Serial.println("Invalid Type");
      //code to be executed if all cases are not matched
    }
  }
  count = 0;
}
