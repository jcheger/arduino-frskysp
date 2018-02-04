/** \mainpage
 * Arduino library for Frsky Smart Port protocol.
 * 
 * This library is not designed to decode data on the transmitter, but on the receiver side. OpenTX makes the rest of
 *  the job on the transmitter.
 *
 * This development is completely independent of Frsky or OpenTX.
 * 
 * What to know about Smart Port
 * =============================
 * * use an inverted serial communication at 57600bds muxed on one port
 * * the receiver polls 28 physical sensors periodically
 * * compared to D, the SP protocol seems much faster, but maybe not - it's easier to use for data encoding, and is
 *   nicer to have a bus than a hub, but the polling of 28 IDs takes time (i.e. if a sensor has many values, it can
 *   send only one at a time, ex. GPS).
 * 
 * Default physical IDs
 * --------------------
 * Every sensor board muste haves a physical ID, that can be changed by the way. Here is a list of the default IDs.
 * 
 * ID | with CRC | sensor
 * ---|----------|-------
 * 1  | 0x00     | Vari-H (altimeter high precision)
 * 2  | 0xA1     | FLVSS / MLVSS (LiPo)
 * 3  | 0x22     | FAS (current)
 * 4  | 0x83     | GPS / Vari-N (altimeter normal precision)
 * 5  | 0xE4     | RPM
 * 6  | 0x45     | SP2UH
 * 7  | 0xC6     | SP2UR
 * 8  | 0x67     | -
 * 9  | 0x48     | -
 * 10 | 0xE9     | ASS (air speed)
 * 11 | 0x6A     | -
 * 12 | 0xCB     | -
 * 13 | 0xAC     | -
 * 14 | 0x0D     | -
 * 15 | 0x8E     | -
 * 16 | 0x2F     | -
 * 17 | 0xD0     | -
 * 18 | 0x71     | -
 * 19 | 0xF2     | -
 * 20 | 0x53     | -
 * 21 | 0x34     | -
 * 22 | 0x95     | -
 * 23 | 0x16     | -
 * 24 | 0xB7     | -
 * 25 | 0x98     | RX / TX internal telemetry
 * 26 | 0x39     | PowerBox (aka Redudancy Bus)
 * 27 | 0xBA     | -
 * 28 | 0x1B     | -
 * 
 * Receiver behavior
 * -----------------
 * The receiver polls in a cyle of ~11 ms.
 * 
 * byte | description
 * -----|------------
 * 0x7E | poll header
 * ID   | physical ID (1-28) computed with a CRC (see \ref FrskySP_sensor_demo/FrskySP_sensor_demo.ino for the full list of polled IDs)
 * 
 * * The receiver will poll the IDs in sequence to find which one is present.
 * * If only one physical ID is found, the receiver will alternate the sensor polling and the search (present sensor,
 *   next ID to search, present sensor, next ID and so on).
 * * If more sensors are found, the poll sequence returns almost to a normal search pattern.
 * 
 * Sensor behavior
 * ---------------
 * Genuine Frsky sensors: the sensor answers to every pool on its physical ID to announce its presence. If no data can
 * be transmitted (no refresh), the sensor answers by an empty packet and a false CRC (type 0x00, ID 0x0000,
 * value 0x00000000, CRC 0xFF).
 * 
 * byte(s) | descrption
 * --------|-----------
 * 1       | type (only 0x10 at now)
 * 2       | sensor logical ID (see \ref FrskySP.h for the full list of IDs)
 * 4 (1)   | value
 * 1       | CRC
 * 
 * (1) length may be up to 8 bytes, while escaping 0x7D and 7x7E values
 * 
 * Slowness considerations
 * =======================
 * There are 2 recurrent discussions on Internet, that are related to what is used in this code and examples:
 * * SoftwareSerial is too slow - no, it is not (at least in Arduino 1.0+). The answers from the Arduino are even faster
 *   than the genuine Frksy sensors. Allthough, the annoying point is the conflict with the PinChangeInt library.
 * * Floating point computing is slow - yes and no. The slowness is reversed if the input or the output of the formula
 *   is a float. Example:
 * ~~~~~
 * int b = (int) a / 10;     // faster (a & b are integers)
 * int b = (int) a * 0.1;    // slower
 * 
 * float b = (int) a / 10;   // slower (b is a float)
 * float b = (int) a * 0.1;  // faster
 * ~~~~~
 * A float computing can take up to 40us. Unless you make a lot of them, there is plenty of time to answer within the
 * poll cycle (11 ms). Allthough, OpenTX has many computing to do and has no time to lose. There is a little drift for
 * the GPS and airspeed values shown on the remote control.
 * 
 * Although, you must be careful around those issues:
 * * only one sensor per physical ID (ex. GPS and normal precision altimeter share the same physical ID 3)
 * * only one answer per poll cycle (the [FrskySP_sensor_demo.ino](\ref FrskySP_sensor_demo/FrskySP_sensor_demo.ino)
 *   example shows how to handle multiple answers for one physical ID)
 * * take care about the polling time of the sensor. For instance, polling a DS18x20 temperature sensor takes up to
 *   750ms. The polling must be asynchronous to be answered within the cycle of 11ms.
 * 
 * Connections
 * ===========
 * Simply connect the smart port data line to any pin SoftwareSerial can be used on.
 *
 * \version devel
 * \author Jean-Christophe Heger
 * \see https://github.com/jcheger/arduino-frskysp - source of this library
 * \see http://www.frsky-rc.com/
 * \see http://www.open-tx.org/
 * \copyright 2014-2016 - Jean-Christophe Heger - Released under the LGPL 3.0 license.
 */
 
#include "Arduino.h"
#include "FrskySP.h"
#include "SoftwareSerial.h"

/**
 * Open a SoftwareSerial connection
 * \param pin Communications pin
 * \brief Class constructor
 * \todo allow the use [AltSoftSerial] (http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html) instead - much faster
 *   than SerialSoftware, and no conflict with [PinChangeInt] (https://code.google.com/p/arduino-pinchangeint/) - see
 *  [bugs] (https://code.google.com/p/arduino-pinchangeint/wiki/Bugs).
 */
FrskySP::FrskySP (int pin) {
    this->mySerial = new SoftwareSerial (pin, pin, true);
    this->mySerial->begin (57600);
}

/**
 * Check if a byte is available on Smart Port
 * 
 * \brief SoftwareSerial.available() passthrough
 */
int FrskySP::available () {
    return this->mySerial->available ();
}

/**
 * Enable LED toggling while sending data
 * \param pin (usually 13)
 */
void FrskySP::ledSet (int pin) {
	this->_pinLed = pin;
	pinMode (pin, OUTPUT);
}

/**
 * Toggle LED thile sending data
 * \param state (LOW or HIGH)
 */
void FrskySP::_ledToggle (int state) {
	if (this->_pinLed >= 0) digitalWrite (this->_pinLed, state);
}

/**
 * \brief Calculate the CRC of a packet
 * \see https://github.com/opentx/opentx/blob/next/radio/src/telemetry/frsky_sport.cpp
 * \param packet packet pointer (in byte[8] presentation)
 */
uint8_t FrskySP::CRC (uint8_t *packet) {
    short crc = 0;
    for (int i=0; i<8; i++) {
        crc += packet[i]; //0-1FF
        crc += crc >> 8;  //0-100
        crc &= 0x00ff;
        crc += crc >> 8;  //0-0FF
        crc &= 0x00ff;
    }
    return ~crc;
}

/**
 * \brief Check the CRC of a packet
 * \see https://github.com/opentx/opentx/blob/next/radio/src/telemetry/frsky_sport.cpp
 * \param packet packet pointer
 * \return true if CRC if false
 */
bool FrskySP::CRCcheck (uint8_t *packet) {
    short crc = 0;
    for (int i=1; i<8; i++) {
        crc += packet[i]; //0-1FF
        crc += crc >> 8;  //0-100
        crc &= 0x00ff;
        crc += crc >> 8;  //0-0FF
        crc &= 0x00ff;
    }
    return (crc == 0x00ff);
}

/**
 * FLVSS does not work the same way than the D8 series. Sending a battery ID over 5
 * will overflow the next sensor on the Taranis. In order to have more than 6 cells,
 * either use another logical ID (ex. FRSKY_SP_CELLS+1), or use another physical ID.
 * 
 * \brief Same as lipoCell (uint8_t id, float val1, float val2), but with only one cell.
 * \param id cell ID (0~5)
 * \param val cell voltage
 * \return formated data for cell voltage (1 cell)
 */
uint32_t FrskySP::lipoCell (uint8_t id, float val) {
	if (this->_cellMax < id + 1) this->_cellMax = id + 1;
    val *= 500;
    return (uint32_t) val << 8 | this->_cellMax << 4 | id;
}

/**
 * A cell packet is formated this way:
 * content    | length
 * ---------- | ------
 * volt[id+1] | 12 bits
 * volt[id]   | 12-bits
 * celltotal  | 4 bits
 * cellid     | 4 bits
 * 
 * The cell total is not used on OpenTX. The cell count is modified by the highest id,
 * but 12 at a maximum.
 *
 * \brief Lipo voltage data format for 2 cells
 * \param id cell ID (0~5)
 * \param val1 cell voltage (for cell ID)
 * \param val2 cell voltage (for cell ID+1)
 * \return formated data for cell voltage (2 cells)
 */
uint32_t FrskySP::lipoCell (uint8_t id, float val1, float val2) {
	if (this->_cellMax < id + 2) this->_cellMax = id + 2;
    val1 *= 500;
    val2 *= 500;
    return ((uint32_t) val2 & 0x0fff) << 20 | ((uint32_t) val1 & 0x0fff) << 8 | (this->_cellMax & 0x0f) << 4 | (id & 0x0f);
}

/**
 * \brief SoftwareSerial.read() passthrough
 */
byte FrskySP::read () {
    return this->mySerial->read ();
}

/**
 * Based on the CleanFlight's code
 * 
 * \brief Send byte and calculate CRC
 * \param c byte value
 * \param *crcp crc pointer
 * \see https://github.com/cleanflight/cleanflight/blob/master/src/main/telemetry/smartport.c
 */
void FrskySP::sendByte (uint8_t c, uint16_t *crcp) {
    // smart port escape sequence
    if (c == 0x7D || c == 0x7E) {
        mySerial->write (0x7D);
        c ^= 0x20;
    }

    this->write (c);

    if (crcp == NULL) return;

    uint16_t crc = *crcp;
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
}

/**
 * Sensors logical IDs and value formats are documented in FrskySP.h.
 * 
 * \brief Simplified version of sendData(), while the type is only 0x10 at now.
 * \param id sensor ID
 * \param val value
 */
void FrskySP::sendData (uint16_t id, int32_t val) {
    this->sendData (0x10, id, (uint32_t) val);
}

/**
 * Sensors logical IDs and value formats are documented in FrskySP.h.
 * Based on the CleanFlight's code
 * 
 * Packet format:
 * content   | length | remark
 * --------- | ------ | ------
 * type      | 8 bit  | always 0x10 at now
 * sensor ID | 16 bit | sensor's logical ID (see FrskySP.h for values)
 * data      | 32 bit | preformated data
 * crc       | 8 bit  | calculated by CRC()
 * 
 * \brief Prepare the packet and send it.
 * \param type value type
 * \param id sensor ID
 * \param val value
 * \return return the CRC for control
 * \see https://github.com/cleanflight/cleanflight/blob/master/src/main/telemetry/smartport.c
 */
void FrskySP::sendData (uint8_t type, uint16_t id, int32_t val) {
    int i = 0;
	uint16_t crc = 0;
	uint8_t *u8p;

	// type
	FrskySP::sendByte (type, &crc);
	
	// id
	u8p = (uint8_t*)&id;
	FrskySP::sendByte (u8p[0], &crc);
	FrskySP::sendByte (u8p[1], &crc);
	
	// val
	u8p = (uint8_t*)&val;
	FrskySP::sendByte (u8p[0], &crc);
	FrskySP::sendByte (u8p[1], &crc);
	FrskySP::sendByte (u8p[2], &crc);
	FrskySP::sendByte (u8p[3], &crc);
	
	// crc
	FrskySP::sendByte (0xFF - (uint8_t)crc, NULL);
}

/**
 * \brief SoftwareSerial.write() passthrough
 */
byte FrskySP::write (byte val) {
    return this->mySerial->write (val);
}
