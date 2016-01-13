/*
 * RPM sensor for Frsky Smart Port protocol.
 * 
 * The RPM pin is 2 and cannot be changed (interrupt 0). The RPMs is refreshed once per second.
 *
 * Requirements
 * ------------
 * - FrskySP library: https://github.com/jcheger/frsky-arduino
 * 
 * See the the images in the example folder to see the pinout.
 *
 * EagleTree sensors
 * -----------------
 * Be very cautious, the pinout is not standard ! Any mistake will make the optical sensor to blow up instantly !
 * - red: GND !!!
 * - black: VCC for optical sensor IR LED. Must have a 100 Ω @ 5V. No more, no less.
 * - white: Signal. Must have a pull-up resistor (see below)
 *
 * Tests with:
 * - EagleTree Brushless RPM Sensor V2 - pull-up: 1 kΩ
 * - EagleTree Optical RPM Sensor - pull-up: 10 kΩ
 * - EagleTree Hall RPM Sensor - pull-up: 1 kΩ or 10 kΩ
 * 
 * 
 * origin: https://github.com/jcheger/frsky-arduino
 * author: Jean-Christophe Heger <jcheger@ordinoscope.net>
 */

#include <FrskySP.h>
#include <SoftwareSerial.h>

// Use DEBUG 1 to compile the serial debug support
#define DEBUG 1

FrskySP FrskySP (10, 11);

unsigned long rpm_count = 0;
unsigned long rpm_micros  = micros ();

/*
 * This is the ratio between the sensor and the final stage.
 * - if the sensor is measuring the final stage, the ratio is oviously 1.0
 * - if the ratio is an integer, you may cheat by declaring the ratio as the number of blades in OpenTX (up to 102)
 * - if the ratio is a float (reducer, helicopter), you will have to modify this value
 */
float rpm_ratio = 1.0;

void setup () {
  #if DEBUG
  Serial.begin (115200);
  Serial.println ("FrskySP rpm sensor interrupt");
  #endif
  
  attachInterrupt (0, rpmISR, FALLING);
}

void loop () {
  static unsigned int i = 0;

  static float    rpm_freq   = 0;
  float           rpm_period = 0;
  static uint32_t rpm_send   = 0;

  rpm_period = (micros () - rpm_micros) / 1000000;
  if (rpm_period >= 1) {
    rpm_freq = rpm_count / rpm_period;
    // The brushless sensor triggers 1~10 pulses per second when no RPM is detected. Erase them.
    rpm_send = (rpm_freq > 10) ? rpm_freq * 60 / rpm_ratio : 0;
    rpm_micros  = micros ();
    rpm_count = 0;
  }

  while (FrskySP.available ()) {

    if (FrskySP.read () == 0x7E) {

      while (!FrskySP.available ());  // wait for the next byte

      switch (FrskySP.read ()) {

        case 0xE4:
          #if DEBUG
          Serial.print ("rpm_freq: ");
          Serial.print (rpm_freq);
          Serial.print (", rpm_out: ");
          Serial.print (rpm_send);
          Serial.print (", rpm_send: ");
          Serial.println (rpm_send);
          #endif
          
          FrskySP.sendData (FRSKY_SP_RPM, rpm_send);
          break;
      }
    }
  }
}

void rpmISR () {
  rpm_count++;
}
