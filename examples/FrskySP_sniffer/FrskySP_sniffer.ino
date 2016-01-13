/*
 * origin: https://github.com/jcheger/frsky-arduino
 * author: Jean-Christophe Heger <jcheger@ordinoscope.net>
 */
#include <FrskySP.h>
#include <SoftwareSerial.h>
#include <Streaming.h>

FrskySP FrskySP (10, 11);

/*
 * Physical IDs + CRC
 */
uint8_t ids[28] = {0x00,0xA1,0x22,0x83,0xE4,0x45,0xC6,0x67,0x48,0xE9,0x6A,0xCB,0xAC,0x0D,0x8E,0x2F,0xD0,0x71,0xF2,0x53,0x34,0x95,0x16,0xB7,0x98,0x39,0xBA,0x1B};

void setup () {
  Serial.begin (115200);
  Serial.println ("FrSky Smart Port active sniffer");
}

void loop () {
  static int i = 0;
  
  union packet_tag {
    uint8_t  byte[8];
    uint64_t uint64;
  } packet;

  packet.uint64 = 0;
  
  FrskySP.write (0x7E);
  FrskySP.write (ids[i]);
  
  Serial << "(" << i << ") " << _HEX(0x7E) << " " << _HEX(ids[i]) << " - ";
  
  delay (11);  // wait for 11ms
  
  if (FrskySP.available () == 8) {
    Serial << "sensor found - ";
    for (int j=0; j<=7; j++) {
      packet.byte[j] = FrskySP.read ();
      Serial << _HEX(packet.byte[j]) << " ";
    }
    Serial << endl;
    decode (packet.byte);
  } else if (FrskySP.available () > 8) {
    Serial << "buffer overflow (" << FrskySP.available () << ") - too many sensors on the same physical ID ?" << endl;
  } else if (FrskySP.available () == 0) {
    //Serial << "no sensor" << endl;
  } else {
    Serial << "buffer underflow - sensor too slow ?" << endl;
  }

  while (FrskySP.available ()) {
    Serial << FrskySP.read ();
  }
  Serial << endl;
  
  i++;
  if (i >= sizeof (ids)) i = 0;
  delay (100);
}

void decode (byte *packet) {
  uint16_t lid = packet[2] << 8 | packet[1];
  //uint32_t val = packet[6] << 24 | packet[5] << 16 | packet[4] << 8 | packet[3];
  uint32_t val = packet[3] << 24 | packet[4] << 16 | packet[5] << 8 | packet[6];
  
  switch (lid & 0xfff0) {
    case 0x0100:
      Serial << _HEX(lid) << " (Altitude): " << val << endl;
      break;
    case 0x0110:
      Serial << _HEX(lid) << " (Vario): " << val << endl;
      break;
    case 0x0200:
      Serial << _HEX(lid) << " (Current): " << val << endl;
      break;
    case 0x0210:
      Serial << _HEX(lid) << " (VFAS): " << val << endl;
      break;
    case 0x0300:
      Serial << _HEX(lid) << " (Lipo Cells): " << val << endl;
      break;
    case 0x0400:
      Serial << _HEX(lid) << " (Temp1): " << val << endl;
      break;
    case 0x0410:
      Serial << _HEX(lid) << " (Temp2): " << val << endl;
      break;
    case 0x0500:
      Serial << _HEX(lid) << " (RPM): " << val << endl;
      break;
    case 0x0600:
      Serial << _HEX(lid) << " (Fuel level): " << val << endl;
      break;
    case 0x0700:
      Serial << _HEX(lid) << " (AccX): " << val << endl;
      break;
    case 0x0710:
      Serial << _HEX(lid) << " (AccY): " << val << endl;
      break;
    case 0x0720:
      Serial << _HEX(lid) << " (AccZ): " << val << endl;
      break;
    case 0x0800:
      Serial << _HEX(lid) << " (GPS Long Lati): " << val << endl;
      break;
    case 0x0820:
      Serial << _HEX(lid) << " (GPS Altitude): " << val << endl;
      break;
    case 0x0830:
      Serial << _HEX(lid) << " (GPS Speed): " << val << endl;
      break;
    case 0x0840:
      Serial << _HEX(lid) << " (GPS Course): " << val << endl;
      break;
    case 0x0850:
      Serial << _HEX(lid) << " (GPS Date Time): " << val << endl;
      break;
    case 0x0900:
      Serial << _HEX(lid) << " (A3): " << val << endl;
      break;
    case 0x0910:
      Serial << _HEX(lid) << " (A4): " << val << endl;
      break;
    case 0x0a00:
      Serial << _HEX(lid) << " (Air Speed): " << val << endl;
      break;
    case 0xf100:
      switch (lid) {
        case 0xd101:
          Serial << _HEX(lid) << " (RSSI): " << endl;
          break;
        case 0xd102:
          Serial << _HEX(lid) << " (A1): " << endl;
          break;
        case 0xd103:
          Serial << _HEX(lid) << " (A2): " << endl;
          break;
        case 0xd104:
          Serial << _HEX(lid) << " (BATT): " << endl;
          break;
        case 0xd105:
          Serial << _HEX(lid) << " (SWR): " << endl;
          break;
      }
      break;
    }
}

