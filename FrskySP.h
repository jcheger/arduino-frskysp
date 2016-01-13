/**
 * \file FrskySP.h
 */
 
#include "Arduino.h"
#include "SoftwareSerial.h"

/**
 * unused
 */
#define FRSKY_ALT_ZERO          0x0000

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_ALT ~ FRSKY_SP_ALT+15 (0x0100 ~ 0x010f)
 * physical ID(s) | 0 - Altimeter high precision / 3 - Altimeter normal precision
 * value          | (int) float * 100 [m]
 * 
 * N.B. OpenTX use the first non-zero value and set it as offset reference.
 * 
 * \brief altimeter (barometric altitude)
 * \warning normal precision altimeter conflicts with GPS physical ID 3
 */
#define FRSKY_SP_ALT            0x0100

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_VARIO ~ FRSKY_SP_VARIO+15 (0x0110 ~ 0x011f)
 * physical ID(s) | 0 - Altimeter high precision
 * value          | ?
 * 
 * \todo what difference with ALT ?
 */
#define FRSKY_SP_VARIO          0x0110

/**
 * info  | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_CURR ~ FRSKY_SP_CURR+15 (0x0200 ~ 0x020f)
 * physical ID(s) | 2 - FSC current sensor
 * value          | (int) float * 10 [A]
 * 
 * \brief VFAS/FSC current
 */
#define FRSKY_SP_CURR           0x0200

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_VFAS ~ FRSKY_SP_VFAS+15 (0x0210 ~ 0x021f)
 * physical ID(s) | 2 - FSC current sensor
 * value          | (int) float * 100 [V]
 * 
 * \brief VFAS/FSC voltage
 */
#define FRSKY_SP_VFAS           0x0210

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_CELLS ~ FRSKY_SP_CELLS+15 (0x0300 ~ 0x030f)
 * physical ID(s) | ?
 * value          | see FrskySP::lipoCell(uint8_t id, float val1, float val2) for data format
 * 
 * \brief FLVSS Lipo cell voltage
 */
#define FRSKY_SP_CELLS          0x0300

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_T1 ~ FRSKY_SP_T1+15 (0x0400 ~ 0x040f)
 * physical ID(s) | 1 - FLVSS Lipo sensor
 * value          | int [°C]
 * 
 * \brief Temperature
 */
#define FRSKY_SP_T1             0x0400

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_T2 ~ FRSKY_SP_T2+15 (0x0410 ~ 0x041f)
 * physical ID(s) | ?
 * value          | int [°C]
 * 
 * \brief Temperature
 */
#define FRSKY_SP_T2             0x0410

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_RPM ~ FRSKY_SP_RPM+15 (0x0500 ~ 0x050f)
 * physical ID(s) | 4
 * value          | int [rpm]
 * 
 * \brief RPM
 */
#define FRSKY_SP_RPM            0x0500

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_FUEL ~ FRSKY_SP_FUEL+15 (0x0600 ~ 0x060f)
 * physical ID(s) | ?
 * value          | int 0~100 [%]
 * 
 * \brief Fuel level
 */
#define FRSKY_SP_FUEL           0x0600

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_ACCX ~ FRSKY_SP_ACCX+15 (0x0700 ~ 0x071f)
 * physical ID(s) | ?
 * value          | (int) float * 100 [g]
 * 
 * \brief Accelerometer (X)
 */
#define FRSKY_SP_ACCX           0x0700

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_ACCY ~ FRSKY_SP_ACCY+15 (0x0710 ~ 0x071f)
 * physical ID(s) | ?
 * value          | (int) float * 100 [g]
 * 
 * \brief Accelerometer (Y)
 */
#define FRSKY_SP_ACCY           0x0710

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_ACCZ ~ FRSKY_SP_ACCZ+15 (0x0720 ~ 0x072f)
 * physical ID(s) | ?
 * value          | (int) float * 100 [g]
 * 
 * \brief Accelerometer (Z)
 */
#define FRSKY_SP_ACCZ           0x0720

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_GPS_LONG_LATI ~ FRSKY_SP_GPS_LONG_LATI+15 (0x0800 ~ 0x080f)
 * physical ID(s) | 3 - GPS
 * value          | ?
 * 
 * \todo
 */
#define FRSKY_SP_GPS_LONG_LATI  0x0800

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_GPS_ALT ~ FRSKY_SP_GPS_ALT+15 (0x0820 ~ 0x082f)
 * physical ID(s) | 3 - GPS
 * value          | (int) float * 100 [m]
 * 
 * N.B. OpenTX:
 * * must have a GpsFix (FRSKY_SP_GPS_LAT_B or FRSKY_SP_GPS_LONG_B must be non null)
 * * use the first non-zero value and set it as offset reference
 * 
 * \brief GPS altitude
 */
#define FRSKY_SP_GPS_ALT        0x0820

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_GPS_SPEED ~ FRSKY_SP_GPS_SPEED+15 (0x0830 ~ 0x083f)
 * physical ID(s) | 3 - GPS
 * value          | (int) float * 1000 [knots]
 * 
 * \brief GPS speed
 * \warning The speed shown on OpenTX has a little drift, because the knots to shown value conversion is simplified.
 * Allthough, raw knots will be recorded in the logs, and the conversion will be correctly in Companion.
 * This was discussed in this issue: https://github.com/opentx/opentx/issues/1422
 */
#define FRSKY_SP_GPS_SPEED      0x0830

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_GPS_COURSE ~ FRSKY_SP_GPS_COURSE+15 (0x0840 ~ 0x084f)
 * physical ID(s) | 3 - GPS
 * value          | (int) float * 100 [°]
 * limits         | 0~359.99°
 * 
 * \brief GPS course (heading)
 */
#define FRSKY_SP_GPS_COURSE      0x0840

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_GPS_TIME_DATE ~ FRSKY_SP_GPS_TIME_DATE+15 (0x0850 ~ 0x085f)
 * physical ID(s) | 3 - GPS
 * value          | ?
 * 
 * \brief GPS time and date
 * \todo
 */
#define FRSKY_SP_GPS_TIME_DATE  0x0850

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_A3 ~ FRSKY_SP_A3+15 (0x0900 ~ 0x090f)
 * physical ID(s) | 5- SP2UART(Host)
 * value          | ?
 * 
 * \brief A3 ADC sensor
 * \todo scale and calibration
 */
#define FRSKY_SP_A3             0x0900

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_A4 ~ FRSKY_SP_A4+15 (0x0910 ~ 0x091f)
 * physical ID(s) | 5 - SP2UART(Host)
 * value          | ?
 * 
 * \brief A4 ADC sensor
 * \todo scale and calibration
 */
#define FRSKY_SP_A4             0x0910

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_AIR_SPEED ~ FRSKY_SP_AIR_SPEED+15 (0x0a00 ~ 0x0a0f)
 * physical ID(s) | ?
 * value          | knots * 10
 * 
 * \brief Air speed sensor
 * \warning The speed shown on OpenTX has a little drift, because the knots to shown value conversion is simplified.
 * Allthough, raw knots will be recorded in the logs, and the conversion will be correctly in Companion.
 * This was discussed in this issue: https://github.com/opentx/opentx/issues/1422
 */
#define FRSKY_SP_AIR_SPEED      0x0a00

/**
 * Not used - already handled by X8R receiver
 *
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_RSSI_ID (0xf101)
 * physical ID(s) | ?
 * 
 * \brief RX RSSI (N/U)
 */
#define FRSKY_SP_RSSI_ID        0xf101

/**
 * Not used - already handled by X8R receiver
 *
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_ADC1_ID (0xf102)
 * physical ID(s) | ?
 *
 * \brief A1 ADC sensor (N/U)
 */
#define FRSKY_SP_ADC1_ID        0xf102

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_ADC2_ID (0xf103)
 * physical ID(s) | ?
 * value          | ?
 * 
 * \brief A2 ADC sensor
 * \todo scale and calibration
 */
#define FRSKY_SP_ADC2_ID        0xf103

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_BATT_ID (0xf104)
 * physical ID(s) | ?
 * value          | ?
 * 
 * \brief unknown
 * \todo what is this sensor ?
 */
#define FRSKY_SP_BATT_ID        0xf104

/**
 * info | comment
 * ---- | -------
 * sensor ID(s)   | FRSKY_SP_SWR_ID (0xf105)
 * physical ID(s) | ?
 * value          | ?
 * 
 * \brief unknown
 * \todo what is this sensor ?
 */
#define FRSKY_SP_SWR_ID         0xf105

/**
 * Frsky Smart Port class
 */
class FrskySP {
    
    public:
        // methods
        FrskySP (int pinRx, int pinTx);
        int      available ();
        uint8_t  CRC (uint8_t *packet);
        bool     CRCcheck (uint8_t *packet);
		void     ledSet (int pin);
        uint32_t lipoCell (uint8_t id, float val);
        uint32_t lipoCell (uint8_t id, float val1, float val2);
        byte     read ();
        void     sendData (uint16_t id, int32_t val);
        void     sendData (uint8_t type, uint16_t id, int32_t val);
        byte     write (byte val);

        // attributes
        SoftwareSerial *mySerial;                                   //!<SoftwareSerial object
        union packet;                                               //!<Packet union (byte[8], uint64)

		uint8_t _cellMax = 0;

    private:
		void    _ledToggle (int state);
		int     _pinLed = -1;										//!<LED pin (-1 = disabled)
        int     _pinRx;												//!<RX pin used by SoftwareSerial
        int     _pinTx;												//!<TX pin used by SoftwareSerial
    
};

/**
 * \example FrskySP_sensor_demo/FrskySP_sensor_demo.ino
 */

/**
 * \example FrskySP_rpm_sensor_freqcount/FrskySP_rpm_sensor_freqcount.ino
 */

/**
 * \example FrskySP_rpm_sensor_interrupt/FrskySP_rpm_sensor_interrupt.ino
 */

/**
 * \example FrskySP_airspeed_sensor_eagletree/FrskySP_airspeed_sensor_eagletree.ino
 */
