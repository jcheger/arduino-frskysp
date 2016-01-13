/*
 * This code is not ready for production yet !
 *
 * Pinout
 * ------
 * Red    - VCC (3-16V)
 * White  - GND
 * Yellow - SDA
 * Brown  - SCL
 *
 * Remarks
 * -------
 * The Wire library is not really respective of the standards, and there is no popular replacement of it.
 * Anyway it works with the EagleTree sensors. The Wire.begin function enables the internal pull-ups, so
 * the is no need to add some external ones.
 *
 * Documentation
 * -------------
 * http://www.eagletreesystems.com/Manuals/microsensor-i2c.pdf
 *
 *  
 * origin: https://github.com/jcheger/frsky-arduino
 * author: Jean-Christophe Heger <jcheger@ordinoscope.net>
 */

#include <FrskySP.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Streaming.h>

#define ASP_V3 0xEA >> 1

FrskySP FrskySP (10, 11);

void setup () {
  Serial.begin (115200);
  Serial.println ("BEGIN");

  Wire.begin();
}

void loop () {
  static float mph = 0;
  static unsigned long sensor_millis = millis ();

  if ((millis() - sensor_millis) >= 100) {
    mph = read_sensor (ASP_V3);  // In third party I2C mode, the airspeed sensor returns mph
    sensor_millis = millis ();
    Serial.println (mph);
  }

  while (FrskySP.available ()) {

    if (FrskySP.read () == 0x7E) {

      while (!FrskySP.available ());  // wait for the next byte

      switch (FrskySP.read ()) {

        case 0x67:  // Physical ID 7
          /*
           * The is a little drift on OpenTX, that was discussed here:
           * https://github.com/opentx/opentx/issues/1422
           *
           * Although, the value will be recored correctly, as so in Companion.
           * Don't try to resolve the shown value on the transmitter if you want
           * to rely on the logged values.
           *
           * real conversion          | value shown OpenTX 
           * -------------------------|----------------------
           * 1 mph = 1.15077945 knots | 23 / 20 = 1.15 (up to 2.0.5: 31 / 27 = 1.148148148)
           * 1 kph = 1.852 knots      | 50 / 27 = 1.851851852
           */
          FrskySP.sendData (FRSKY_SP_AIR_SPEED, mph * 10 / 1.15077945 + 0.5);
          break;
      }
    }
  }
}

/*
 * Airspeed sensor answers in 756 us
 */
int16_t read_sensor (int16_t id) {
  int16_t raw;
  
  Wire.beginTransmission (ASP_V3);
  Wire.write (0x07);
  Wire.endTransmission ();

  Wire.beginTransmission (ASP_V3);
  Wire.requestFrom (ASP_V3, 2);
  if (Wire.available ()) raw  = Wire.read ();
  else                   Serial.println ("ERROR on byte 1");
   
  if (Wire.available ()) raw |= (int) Wire.read () << 8;
  else                   Serial.println ("ERROR on byte 2");
  Wire.endTransmission ();

  return raw;
}

