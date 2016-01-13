/*
 * Requirements
 * ------------
 * - FrskySP library - https://github.com/jcheger/frsky-arduino
 * - Recent version of Arduino's IDE (ex. 1.6.1), else SoftwareSerial will fail at 57600 bds.
 * 
 * origin: https://github.com/jcheger/frsky-arduino
 * author: Jean-Christophe Heger <jcheger@ordinoscope.net>
 */

#include <FrskySP.h>
#include <SoftwareSerial.h>

FrskySP FrskySP (10, 11);

void setup () {
  FrskySP.ledSet (13);
}

void loop () {
  static unsigned int i = 0;  // increment used when several values must be sent within the same physical ID - only one per cycle
  static float alt = 100.0;   // for demonstration only - altitude must be over 0 to be set as a reference in OpenTX
  
  while (FrskySP.available ()) {
    
    if (FrskySP.read () == 0x7E) {
      while (!FrskySP.available ());  // wait for the next byte
      switch (FrskySP.read ()) {

        case 0x00:  // Physical ID 1 - Vario2 (altimeter high precision)
          break;
          
        case 0xA1:  // Physical ID 2 - FLVSS Lipo sensor (can be sent with one or two cell voltages)
          // works better by sending only on cell voltage for a large amount of cells
          if (i % 8 == 0) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (0, 1.01, 1.02));
          if (i % 8 == 1) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (2, 1.03, 1.04));
          if (i % 8 == 2) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (4, 1.05, 1.06));
          if (i % 8 == 3) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (6, 1.07, 1.08));
          if (i % 8 == 4) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (8, 1.09));
          if (i % 8 == 5) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (9, 1.10));
          if (i % 8 == 6) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (10, 1.11));
          if (i % 8 == 7) FrskySP.sendData (FRSKY_SP_CELLS, FrskySP.lipoCell (11, 1.12));
          break;
          
        case 0x22:  // Physical ID 3 - FAS-40S current sensor
          if (i % 2 == 0) FrskySP.sendData (FRSKY_SP_CURR, 11.5 * 10);
          if (i % 2 == 1) FrskySP.sendData (FRSKY_SP_VFAS, 22.2 * 100);
          break;
          
        case 0x83:  // Physical ID 4 - GPS / altimeter (normal precision)
          if (i % 3 == 0) FrskySP.sendData (FRSKY_SP_GPS_ALT, alt * 100);
          if (i % 3 == 1) FrskySP.sendData (FRSKY_SP_GPS_SPEED, (float) 100 / 1.852 * 1000);
          //if (i % 3 == 2) FrskySP.sendData (FRSKY_SP_ALT, alt * 100);
          alt += 0.1;
          break;
          
        case 0xE4:  // Physical ID 5 - RPM
          FrskySP.sendData (FRSKY_SP_RPM, 11111);
          break;
          
        case 0x45:  // Physical ID 6 - SP2UART(Host)
          if (i % 3 == 0) FrskySP.sendData (FRSKY_SP_ADC2_ID, 1);
          if (i % 3 == 1) FrskySP.sendData (FRSKY_SP_A3,      10);
          if (i % 3 == 2) FrskySP.sendData (FRSKY_SP_A4,      100);
          break;
          
        case 0xC6:  // Physical ID 7 - SPUART(Remote)
          break;
          
        case 0x67:  // Physical ID 8 - 
          break;
          
        case 0x48:  // Physical ID 9 - 
          break;
          
        case 0xE9:  // Physical ID 10 - 
          break;
          
        case 0x6A:  // Physical ID 11 - 
          break;
          
        case 0xCB:  // Physical ID 12 - 
          break;
          
        case 0xAC:  // Physical ID 13 - 
          break;
          
        case 0x0D:  // Physical ID 14 - 
          break;
          
        case 0x8E:  // Physical ID 15 - 
          break;
          
        case 0x2F:  // Physical ID 16 - 
          break;
          
        case 0xD0:  // Physical ID 17 - 
          break;
          
        case 0x71:  // Physical ID 18 - 
          break;
          
        case 0xF2:  // Physical ID 19 - 
          break;
          
        case 0x53:  // Physical ID 20 - 
          break;
          
        case 0x34:  // Physical ID 21 - 
          break;
          
        case 0x95:  // Physical ID 22 - 
          break;
          
        case 0x16:  // Physical ID 23 - 
          break;
          
        case 0xB7:  // Physical ID 24 - 
          if (i % 3 == 0) FrskySP.sendData (FRSKY_SP_ACCX,  1.11 * 100);
          if (i % 3 == 1) FrskySP.sendData (FRSKY_SP_ACCY, -2.22 * 100);
          if (i % 3 == 2) FrskySP.sendData (FRSKY_SP_ACCZ,  3.33 * 100);
          break;
          
        case 0x98:  // Physical ID 25 - 
          FrskySP.sendData (FRSKY_SP_FUEL, 41);
          break;
          
        case 0x39:  // Physical ID 26 - 
          break;
          
        case 0xBA:  // Physical ID 27 - 
          if (i % 2 == 0) FrskySP.sendData (FRSKY_SP_T1, 28);
          if (i % 2 == 1) FrskySP.sendData (FRSKY_SP_T2, 18);
          break;
          
        case 0x1B:  // Physical ID 28 - 
          break;
      }
    }
  }
  i++;
}
