/*
	This is a library written for the u-blox ZED-F9P and NEO-M8P-2
	SparkFun sells these at its website: www.sparkfun.com
	Do you like this library? Help support SparkFun. Buy a board!
	https://www.sparkfun.com/products/16481
	https://www.sparkfun.com/products/15136
	https://www.sparkfun.com/products/15005
	https://www.sparkfun.com/products/15733
	https://www.sparkfun.com/products/15193
	https://www.sparkfun.com/products/15210

	Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
	v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020

	This library handles configuring and handling the responses
	from a u-blox GPS module. Works with most modules from u-blox including
	the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

	https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

	Development environment specifics:
	Arduino IDE 1.8.13

	SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
	The MIT License (MIT)
	Copyright (c) 2016 SparkFun Electronics
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
	associated documentation files (the "Software"), to deal in the Software without restriction,
	including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
	and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
	do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial
	portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
	NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * EWING modified
 * Stream library -> Serial library
 * Note: functions required
 *   [o] _serialPort->write()
 *   [o] _serialPort->available()
 *   [o] _serialPort->read()
 *   [o] _debugSerial->print()
 *        [o] statusString()
 *   [O] digitalWrite
 *   [o] _i2cPort->*
 *   [o] wiringPiSetup()
 *   [o]-lwiringPi 
 */

#ifndef UBLOX_GNSS_LIBRARY_H
#define UBLOX_GNSS_LIBRARY_H
#include <chrono>

#include "ublox_config_keys.h"
#include "ublox_structs.h"
#include "ublox_class_id.h"

// serial library from wjwood
#include "serial/serial.h"
using namespace serial;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Define a digital pin to aid debugging
//Leave set to -1 if not needed
const int debugPin = -1;

// Global Status Returns
typedef enum
{
	SFE_UBLOX_STATUS_SUCCESS,
	SFE_UBLOX_STATUS_FAIL,
	SFE_UBLOX_STATUS_CRC_FAIL,
	SFE_UBLOX_STATUS_TIMEOUT,
	SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
	SFE_UBLOX_STATUS_OUT_OF_RANGE,
	SFE_UBLOX_STATUS_INVALID_ARG,
	SFE_UBLOX_STATUS_INVALID_OPERATION,
	SFE_UBLOX_STATUS_MEM_ERR,
	SFE_UBLOX_STATUS_HW_ERR,
	SFE_UBLOX_STATUS_DATA_SENT,		// This indicates that a 'set' was successful
	SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
	SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
	SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;

// ubxPacket validity
typedef enum
{
	SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
	SFE_UBLOX_PACKET_VALIDITY_VALID,
	SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
	SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
// packetAuto is used to store expected "automatic" messages
typedef enum
{
	SFE_UBLOX_PACKET_PACKETCFG,
	SFE_UBLOX_PACKET_PACKETACK,
	SFE_UBLOX_PACKET_PACKETBUF,
	SFE_UBLOX_PACKET_PACKETAUTO
} sfe_ublox_packet_buffer_e;

//Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;



enum dynModel // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
{
	DYN_MODEL_PORTABLE = 0, //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
	// 1 is not defined
	DYN_MODEL_STATIONARY = 2, //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
	DYN_MODEL_PEDESTRIAN,	  //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
	DYN_MODEL_AUTOMOTIVE,	  //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
	DYN_MODEL_SEA,			  //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
	DYN_MODEL_AIRBORNE1g,	  //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
	DYN_MODEL_AIRBORNE2g,	  //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
	DYN_MODEL_AIRBORNE4g,	  //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
	DYN_MODEL_WRIST,		  // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
	DYN_MODEL_BIKE,			  // Supported in protocol versions 19.2
	DYN_MODEL_UNKNOWN = 255 // getDynamicModel will return 255 if sendCommand fails
};

// The GNSS identifiers - used by UBX-CFG-GNSS (0x06 0x3E) GNSS system configuration
enum sfe_ublox_gnss_ids_e
{
	SFE_UBLOX_GNSS_ID_GPS,
	SFE_UBLOX_GNSS_ID_SBAS,
	SFE_UBLOX_GNSS_ID_GALILEO,
	SFE_UBLOX_GNSS_ID_BEIDOU,
	SFE_UBLOX_GNSS_ID_IMES,
	SFE_UBLOX_GNSS_ID_QZSS,
	SFE_UBLOX_GNSS_ID_GLONASS
};

#ifndef MAX_PAYLOAD_SIZE
// v2.0: keep this for backwards-compatibility, but this is largely superseded by setPacketCfgPayloadSize
#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules
//#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values
#endif

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
	uint8_t cls;
	uint8_t id;
	uint16_t len; //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter; //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload; // We will allocate RAM for the payload if/when needed.
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
typedef struct
{
	uint8_t status;	   // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
	uint8_t numFences; // Number of geofences
	uint8_t combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
	uint8_t states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
} geofenceState;

// Struct to hold the current geofence parameters
typedef struct
{
	uint8_t numFences; // Number of active geofences
	int32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
	int32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
	uint32_t rads[4];  // Radii of geofences (in m * 10^-2)
} geofenceParams_t;

// Struct to hold the module software version
typedef struct
{
	uint8_t versionLow;		 //Loaded from getProtocolVersion().
	uint8_t versionHigh;
	bool moduleQueried;
} moduleSWVersion_t;

class SFE_UBLOX_GNSS
{
public:
	SFE_UBLOX_GNSS(void);

// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce defaultMaxWait to 250.
#ifndef defaultMaxWait // Let's allow the user to define their own value if they want to
#define defaultMaxWait 1100
#endif

	//New in v2.0: allow the payload size for packetCfg to be changed
	void setPacketCfgPayloadSize(size_t payloadSize); // Set packetCfgPayloadSize

	//By default use the default I2C address, and use Wire port
	//bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x42); //Returns true if module is detected
	bool begin(uint8_t deviceAddress);
	
	//serialPort needs to be perviously initialized to correct baud rate
	bool begin(Serial &serialPort); //Returns true if module is detected

	void setI2CpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the I2C polling wait if required

	//Control the size of the internal I2C transaction amount
	void setI2CTransactionSize(uint8_t bufferSize);
	uint8_t getI2CTransactionSize(void);

	//Set the max number of bytes set in a given I2C transaction
	uint8_t i2cTransactionSize = 32; //Default to ATmega328 limit

	//Returns true if device answers on _gpsI2Caddress address or via Serial
	bool isConnected(uint16_t maxWait = 1100);

	// Enable debug messages using the chosen Serial port (Serial)
	// Boards like the RedBoard Turbo use SerialUSB (not Serial).
	// But other boards like the SAMD51 Thing Plus use Serial (not SerialUSB).
	// These lines let the code compile cleanly on as many SAMD boards as possible.
	#if defined(ARDUINO_ARCH_SAMD)	// Is this a SAMD board?
	#if defined(USB_VID)						// Is the USB Vendor ID defined?
	#if (USB_VID == 0x1B4F)					// Is this a SparkFun board?
	#if !defined(ARDUINO_SAMD51_THING_PLUS) & !defined(ARDUINO_SAMD51_MICROMOD) // If it is not a SAMD51 Thing Plus or SAMD51 MicroMod
	void enableDebugging(Serial &debugPort, bool printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	#else
	void enableDebugging(Serial &debugPort, bool printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	#endif
	#else
	void enableDebugging(Serial &debugPort, bool printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	#endif
	#else
	void enableDebugging(Serial &debugPort, bool printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	#endif
	#else
	void enableDebugging(Serial &debugPort, bool printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
	#endif

	void disableDebugging(void); //Turn off debug statements
	void disableNmeaDebugPrint(void); //Turn off cli printing of NMEA debug message
	void debugPrint(char *message); //Safely print debug statements
	void debugPrintln(char *message); //Safely print debug statements
	const char *statusString(sfe_ublox_status_e stat); //Pretty print the return value

	// Check for the arrival of new I2C/Serial data

	void disableUBX7Fcheck(bool disabled = true); // When logging RAWX data, we need to be able to disable the "7F" check in checkUbloxI2C

	//Changed in V1.8.1: provides backward compatibility for the examples that call checkUblox directly
	//Will default to using packetCfg to look for explicit autoPVT packets so they get processed correctly by processUBX
	bool checkUblox(uint8_t requestedClass = 0, uint8_t requestedID = 0); //Checks module with user selected commType

	//bool checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);	   //Method for I2C polling of data, passing any new bytes to process()
	bool checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); //Method for serial polling of data, passing any new bytes to process()

	// Process the incoming data

	void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);	//Processes NMEA and UBX binary sentences one byte at a time
	void processNMEA(char incoming) __attribute__((weak)); //Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries
	void processRTCMframe(uint8_t incoming); //Monitor the incoming bytes for start and length bytes
	void processRTCM(uint8_t incoming) __attribute__((weak)); //Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.
	void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); //Given a character, file it away into the uxb packet structure
	void processUBXpacket(ubxPacket *msg); //Once a packet has been received and validated, identify this packet's class/id and update internal flags

	// Send I2C/Serial commands to the module

	void calcChecksum(ubxPacket *msg); //Sets the checksumA and checksumB of a given messages
	sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait = defaultMaxWait, bool expectACKonly = false); //Given a packet and payload, send everything including CRC bytes, return true if we got a response
	sfe_ublox_status_e sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait = defaultMaxWait);
	void sendSerialCommand(ubxPacket *outgoingUBX);

	void printPacket(ubxPacket *packet, bool alwaysPrintPayload = false); //Useful for debugging

	// After sending a message to the module, wait for the expected response (data+ACK or just data)

	sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait);	 //Poll the module until a config packet and an ACK is received, or just an ACK
	sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait); //Poll the module until a config packet is received

	// Check if any callbacks need to be called
	void checkCallbacks(void);

	// Push (e.g.) RTCM data directly to the module
	// Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
	bool pushRawData(uint8_t *dataBytes, size_t numDataBytes);

	// Support for data logging
	void setFileBufferSize(uint16_t bufferSize); // Set the size of the file buffer. This must be called _before_ .begin.
	uint16_t extractFileBufferData(uint8_t *destination, uint16_t numBytes); // Extract numBytes of data from the file buffer. Copy it to destination. It is the user's responsibility to ensure destination is large enough.
	uint16_t fileBufferAvailable(void); // Returns the number of bytes available in file buffer which are waiting to be read
	uint16_t getMaxFileBufferAvail(void); // Returns the maximum number of bytes which the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
	void getSentenceTime(std::chrono::time_point<std::chrono::steady_clock,std::chrono::nanoseconds> *tpStart, std::chrono::time_point<std::chrono::steady_clock,std::chrono::nanoseconds> *tpEnd);

	// Specific commands

	//Port configurations
	bool getPortSettings(uint8_t portID, uint16_t maxWait = defaultMaxWait);					   //Returns the current protocol bits in the UBX-CFG-PRT command for a given port
	bool setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = defaultMaxWait); //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setPortInput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = defaultMaxWait);  //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof

	bool setI2CAddress(uint8_t deviceAddress, uint16_t maxTime = defaultMaxWait);										 //Changes the I2C address of the u-blox module
	void setSerialRate(uint32_t baudrate, uint8_t uartPort = COM_PORT_UART1, uint16_t maxTime = defaultMaxWait); //Changes the serial baud rate of the u-blox module, uartPort should be COM_PORT_UART1/2

	//bool setI2COutput(uint8_t comSettings, uint16_t maxWait = defaultMaxWait);				//Configure I2C port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setUART1Output(uint8_t comSettings, uint16_t maxWait = defaultMaxWait); //Configure UART1 port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setUART2Output(uint8_t comSettings, uint16_t maxWait = defaultMaxWait); //Configure UART2 port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setUSBOutput(uint8_t comSettings, uint16_t maxWait = defaultMaxWait);				//Configure USB port to output UBX, NMEA, RTCM3 or a combination thereof
	bool setSPIOutput(uint8_t comSettings, uint16_t maxWait = defaultMaxWait);				//Configure SPI port to output UBX, NMEA, RTCM3 or a combination thereof
	void setNMEAOutputPort(Serial &nmeaOutputPort);																 //Sets the internal variable for the port to direct NMEA characters to

	//Reset to defaults

	void factoryReset(); //Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
	void hardReset();	 //Perform a reset leading to a cold start (zero info start-up)
	bool factoryDefault(uint16_t maxWait = defaultMaxWait);							 //Reset module to factory defaults

	//Save configuration to BBR / Flash

	bool saveConfiguration(uint16_t maxWait = defaultMaxWait);						 //Save current configuration to flash and BBR (battery backed RAM)
	bool saveConfigSelective(uint32_t configMask, uint16_t maxWait = defaultMaxWait); //Save the selected configuration sub-sections to flash and BBR (battery backed RAM)

	//Functions to turn on/off message types for a given port ID (see COM_PORT_I2C, etc above)
	bool configureMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate, uint16_t maxWait = defaultMaxWait);
	bool enableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = defaultMaxWait);
	bool disableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint16_t maxWait = defaultMaxWait);
	bool enableNMEAMessage(uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = defaultMaxWait);
	bool disableNMEAMessage(uint8_t msgID, uint8_t portID, uint16_t maxWait = defaultMaxWait);
	bool enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t sendRate, uint16_t maxWait = defaultMaxWait); //Given a message number turns on a message ID for output over given PortID
	bool disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = defaultMaxWait); //Turn off given RTCM message from a given port

	//Functions used for RTK and base station setup
	//It is probably safe to assume that users of the RTK will be using I2C / Qwiic. So let's leave maxWait set to 250ms.
	bool getSurveyMode(uint16_t maxWait = 250); //Get the current TimeMode3 settings
	bool setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Control survey in mode
	bool enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = 250); //Begin Survey-In for NEO-M8P
	bool disableSurveyMode(uint16_t maxWait = 250); //Stop Survey-In mode
	// Given coordinates, put receiver into static position. Set latlong to true to pass in lat/long values instead of ecef.
	// For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
	// For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
	bool setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong = false, uint16_t maxWait = 250);
	bool setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latLong = false, uint16_t maxWait = 250);

	//Read the module's protocol version
	uint8_t getProtocolVersionHigh(uint16_t maxWait = defaultMaxWait); //Returns the PROTVER XX.00 from UBX-MON-VER register
	uint8_t getProtocolVersionLow(uint16_t maxWait = defaultMaxWait);	//Returns the PROTVER 00.XX from UBX-MON-VER register
	bool getProtocolVersion(uint16_t maxWait = defaultMaxWait);		//Queries module, loads low/high bytes
	moduleSWVersion_t *moduleSWVersion = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	//Support for geofences
	bool addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, uint8_t confidence = 0, uint8_t pinPolarity = 0, uint8_t pin = 0, uint16_t maxWait = defaultMaxWait); // Add a new geofence
	bool clearGeofences(uint16_t maxWait = defaultMaxWait); //Clears all geofences
	bool clearAntPIO(uint16_t maxWait = defaultMaxWait); //Clears the antenna control pin settings to release the PIOs
	bool getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait = defaultMaxWait); //Returns the combined geofence state
	// Storage for the geofence parameters. RAM is allocated for this if/when required.
	geofenceParams_t *currentGeofenceParams = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	//Power save / off
	bool powerSaveMode(bool power_save = true, uint16_t maxWait = defaultMaxWait);
	uint8_t getPowerSaveMode(uint16_t maxWait = defaultMaxWait); // Returns 255 if the sendCommand fails
	bool powerOff(uint32_t durationInMs, uint16_t maxWait = defaultMaxWait);
	bool powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources = VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, bool forceWhileUsb = true, uint16_t maxWait = 1100);

	//Change the dynamic platform model using UBX-CFG-NAV5
	bool setDynamicModel(dynModel newDynamicModel = DYN_MODEL_PORTABLE, uint16_t maxWait = defaultMaxWait);
	uint8_t getDynamicModel(uint16_t maxWait = defaultMaxWait); // Get the dynamic model - returns 255 if the sendCommand fails

	//Reset the odometer
	bool resetOdometer(uint16_t maxWait = defaultMaxWait); // Reset the odometer

	//Enable/Disable individual GNSS systems using UBX-CFG-GNSS
	//Note: you must leave at least one major GNSS enabled! If in doubt, enable GPS before disabling the others
	//TO DO: Add support for sigCfgMask and maxTrkCh. (Need to resolve ambiguity with maxWait)
	bool enableGNSS(bool enable, sfe_ublox_gnss_ids_e id, uint16_t maxWait = defaultMaxWait);
	bool isGNSSenabled(sfe_ublox_gnss_ids_e id, uint16_t maxWait = defaultMaxWait);

	//Reset ESF automatic IMU-mount alignment
	bool resetIMUalignment(uint16_t maxWait = defaultMaxWait);

	//Configure Time Pulse Parameters
	bool getTimePulseParameters(UBX_CFG_TP5_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Get the time pulse parameters using UBX_CFG_TP5
	bool setTimePulseParameters(UBX_CFG_TP5_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Set the time pulse parameters using UBX_CFG_TP5

	//General configuration (used only on protocol v27 and higher - ie, ZED-F9P)

	//It is probably safe to assume that users of the ZED-F9P will be using I2C / Qwiic.
	//If they are using Serial then the higher baud rate will also help. So let's leave maxWait set to 250ms.
	uint32_t createKey(uint16_t group, uint16_t id, uint8_t size); //Form 32-bit key from group/id/size
	sfe_ublox_status_e getVal(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);					 //Load payload with response
	uint8_t getVal8(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);								 //Returns the value at a given key location
	uint16_t getVal16(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);							 //Returns the value at a given key location
	uint32_t getVal32(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);							 //Returns the value at a given key location
	uint8_t getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250);	 //Returns the value at a given group/id/size location
	uint16_t getVal16(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
	uint32_t getVal32(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = 250); //Returns the value at a given group/id/size location
	uint8_t setVal(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);				 //Sets the 16-bit value at a given group/id/size location
	uint8_t setVal8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);				 //Sets the 8-bit value at a given group/id/size location
	uint8_t setVal16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);			 //Sets the 16-bit value at a given group/id/size location
	uint8_t setVal32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = 250);			 //Sets the 32-bit value at a given group/id/size location
	uint8_t newCfgValset8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_ALL);								 //Define a new UBX-CFG-VALSET with the given KeyID and 8-bit value
	uint8_t newCfgValset16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL);								 //Define a new UBX-CFG-VALSET with the given KeyID and 16-bit value
	uint8_t newCfgValset32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_ALL);								 //Define a new UBX-CFG-VALSET with the given KeyID and 32-bit value
	uint8_t addCfgValset8(uint32_t keyID, uint8_t value);																 //Add a new KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
	uint8_t addCfgValset16(uint32_t keyID, uint16_t value);																 //Add a new KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
	uint8_t addCfgValset32(uint32_t keyID, uint32_t value);																 //Add a new KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
	uint8_t sendCfgValset8(uint32_t keyID, uint8_t value, uint16_t maxWait = 250);										 //Add the final KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
	uint8_t sendCfgValset16(uint32_t keyID, uint16_t value, uint16_t maxWait = 250);									 //Add the final KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
	uint8_t sendCfgValset32(uint32_t keyID, uint32_t value, uint16_t maxWait = 250);									 //Add the final KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it

// getPVT will only return data once in each navigation cycle. By default, that is once per second.
// Therefore we should set defaultMaxWait to slightly longer than that.
// If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
// then you should use a shorter maxWait. 300msec would be about right: getPVT(300)

	// get and set functions for all of the "automatic" message processing

	// Navigation (NAV)

	bool getNAVPOSECEF(uint16_t maxWait = defaultMaxWait); // NAV POSECEF
	bool setAutoNAVPOSECEF(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic POSECEF reports at the navigation frequency
	bool setAutoNAVPOSECEF(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic POSECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVPOSECEFcallback(void (*callbackPointer)(UBX_NAV_POSECEF_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic POSECEF reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVPOSECEF(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and POSECEF is send cyclically already
	void flushNAVPOSECEF(); //Mark all the data as read/stale
	void logNAVPOSECEF(bool enabled = true); // Log data to file buffer

	bool getNAVSTATUS(uint16_t maxWait = defaultMaxWait); // NAV STATUS
	bool setAutoNAVSTATUS(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic STATUS reports at the navigation frequency
	bool setAutoNAVSTATUS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic STATUS reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVSTATUScallback(void (*callbackPointer)(UBX_NAV_STATUS_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVSTATUS(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and STATUS is send cyclically already
	void flushNAVSTATUS(); //Mark all the data as read/stale
	void logNAVSTATUS(bool enabled = true); // Log data to file buffer

	bool getDOP(uint16_t maxWait = defaultMaxWait); //Query module for latest dilution of precision values and load global vars:. If autoDOP is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new DOP is available.
	bool setAutoDOP(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic DOP reports at the navigation frequency
	bool setAutoDOP(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic DOP reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoDOPcallback(void (*callbackPointer)(UBX_NAV_DOP_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic DOP reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoDOP(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and DOP is send cyclically already
	void flushDOP(); //Mark all the DOP data as read/stale
	void logNAVDOP(bool enabled = true); // Log data to file buffer

	bool getVehAtt(uint16_t maxWait = defaultMaxWait); // NAV ATT Helper
	bool getNAVATT(uint16_t maxWait = defaultMaxWait); // NAV ATT
	bool setAutoNAVATT(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic vehicle attitude reports at the navigation frequency
	bool setAutoNAVATT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic vehicle attitude reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVATTcallback(void (*callbackPointer)(UBX_NAV_ATT_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVATT(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and vehicle attitude is send cyclically already
	void flushNAVATT(); //Mark all the data as read/stale
	void logNAVATT(bool enabled = true); // Log data to file buffer

	bool getPVT(uint16_t maxWait = defaultMaxWait);	//Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new PVT is available.
	bool setAutoPVT(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic PVT reports at the navigation frequency
	bool setAutoPVT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoPVTcallback(void (*callbackPointer)(UBX_NAV_PVT_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoPVT(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and PVT is send cyclically already
	void flushPVT(); //Mark all the PVT data as read/stale
	void logNAVPVT(bool enabled = true); // Log data to file buffer

	bool getNAVODO(uint16_t maxWait = defaultMaxWait); // NAV ODO
	bool setAutoNAVODO(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic ODO reports at the navigation frequency
	bool setAutoNAVODO(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ODO reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVODOcallback(void (*callbackPointer)(UBX_NAV_ODO_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic ODO reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVODO(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and ODO is send cyclically already
	void flushNAVODO(); //Mark all the data as read/stale
	void logNAVODO(bool enabled = true); // Log data to file buffer

	bool getNAVVELECEF(uint16_t maxWait = defaultMaxWait); // NAV VELECEF
	bool setAutoNAVVELECEF(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic VELECEF reports at the navigation frequency
	bool setAutoNAVVELECEF(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic VELECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVVELECEFcallback(void (*callbackPointer)(UBX_NAV_VELECEF_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic VELECEF reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVVELECEF(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and VELECEF is send cyclically already
	void flushNAVVELECEF(); //Mark all the data as read/stale
	void logNAVVELECEF(bool enabled = true); // Log data to file buffer

	bool getNAVVELNED(uint16_t maxWait = defaultMaxWait); // NAV VELNED
	bool setAutoNAVVELNED(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic VELNED reports at the navigation frequency
	bool setAutoNAVVELNED(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic VELNED reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVVELNEDcallback(void (*callbackPointer)(UBX_NAV_VELNED_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic VELNED reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVVELNED(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and VELNED is send cyclically already
	void flushNAVVELNED(); //Mark all the data as read/stale
	void logNAVVELNED(bool enabled = true); // Log data to file buffer

	bool getNAVHPPOSECEF(uint16_t maxWait = defaultMaxWait); // NAV HPPOSECEF
	bool setAutoNAVHPPOSECEF(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic HPPOSECEF reports at the navigation frequency
	bool setAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HPPOSECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVHPPOSECEFcallback(void (*callbackPointer)(UBX_NAV_HPPOSECEF_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic HPPOSECEF reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and HPPOSECEF is send cyclically already
	void flushNAVHPPOSECEF(); //Mark all the data as read/stale
	void logNAVHPPOSECEF(bool enabled = true); // Log data to file buffer

	bool getHPPOSLLH(uint16_t maxWait = defaultMaxWait); //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new HPPOSLLH is available.
	bool setAutoHPPOSLLH(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HPPOSLLH reports at the navigation frequency
	bool setAutoHPPOSLLH(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HPPOSLLH reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoHPPOSLLHcallback(void (*callbackPointer)(UBX_NAV_HPPOSLLH_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic HPPOSLLH reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
	void flushHPPOSLLH(); //Mark all the HPPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
	void logNAVHPPOSLLH(bool enabled = true); // Log data to file buffer

	bool getNAVCLOCK(uint16_t maxWait = defaultMaxWait); // NAV CLOCK
	bool setAutoNAVCLOCK(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic clock reports at the navigation frequency
	bool setAutoNAVCLOCK(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic clock reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoNAVCLOCKcallback(void (*callbackPointer)(UBX_NAV_CLOCK_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic CLOCK reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoNAVCLOCK(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and clock is send cyclically already
	void flushNAVCLOCK(); //Mark all the data as read/stale
	void logNAVCLOCK(bool enabled = true); // Log data to file buffer

	// Add "auto" support for NAV SVIN - to avoid needing 'global' storage
	bool getSurveyStatus(uint16_t maxWait); //Reads survey in status

	bool getRELPOSNED(uint16_t maxWait = defaultMaxWait); //Get Relative Positioning Information of the NED frame
	bool setAutoRELPOSNED(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic RELPOSNED reports
	bool setAutoRELPOSNED(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic RELPOSNED, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoRELPOSNEDcallback(void (*callbackPointer)(UBX_NAV_RELPOSNED_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic RELPOSNED reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoRELPOSNED(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and RELPOSNED is send cyclically already
	void flushNAVRELPOSNED(); //Mark all the data as read/stale
	void logNAVRELPOSNED(bool enabled = true); // Log data to file buffer

	// Receiver Manager Messages (RXM)

	bool getRXMSFRBX(uint16_t maxWait = defaultMaxWait); // RXM SFRBX
	bool setAutoRXMSFRBX(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic RXM SFRBX reports at the navigation frequency
	bool setAutoRXMSFRBX(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic RXM SFRBX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoRXMSFRBXcallback(void (*callbackPointer)(UBX_RXM_SFRBX_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic SFRBX reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoRXMSFRBX(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and RXM SFRBX is send cyclically already
	void flushRXMSFRBX(); //Mark all the data as read/stale
	void logRXMSFRBX(bool enabled = true); // Log data to file buffer

	bool getRXMRAWX(uint16_t maxWait = defaultMaxWait); // RXM RAWX
	bool setAutoRXMRAWX(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic RXM RAWX reports at the navigation frequency
	bool setAutoRXMRAWX(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic RXM RAWX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoRXMRAWXcallback(void (*callbackPointer)(UBX_RXM_RAWX_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic RAWX reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoRXMRAWX(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and RXM RAWX is send cyclically already
	void flushRXMRAWX(); //Mark all the data as read/stale
	void logRXMRAWX(bool enabled = true); // Log data to file buffer

	// Configuration (CFG)

	// Add "auto" support for CFG RATE - because we use it for isConnected (to stop it being mugged by other messages)
	bool getNavigationFrequencyInternal(uint16_t maxWait = defaultMaxWait); //Get the number of nav solutions sent per second currently being output by module

	// Timing messages (TIM)

	bool getTIMTM2(uint16_t maxWait = defaultMaxWait); // TIM TM2
	bool setAutoTIMTM2(bool enabled, uint16_t maxWait = defaultMaxWait);  //Enable/disable automatic TIM TM2 reports at the navigation frequency
	bool setAutoTIMTM2(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic TIM TM2 reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoTIMTM2callback(void (*callbackPointer)(UBX_TIM_TM2_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic TM2 reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoTIMTM2(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and TIM TM2 is send cyclically already
	void flushTIMTM2(); //Mark all the data as read/stale
	void logTIMTM2(bool enabled = true); // Log data to file buffer

	// Sensor fusion (dead reckoning) (ESF)

	bool getEsfAlignment(uint16_t maxWait = defaultMaxWait); // ESF ALG Helper
	bool getESFALG(uint16_t maxWait = defaultMaxWait); // ESF ALG
	bool setAutoESFALG(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF ALG reports
	bool setAutoESFALG(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF ALG reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoESFALGcallback(void (*callbackPointer)(UBX_ESF_ALG_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic ALG reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoESFALG(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and ESF ALG is send cyclically already
	void flushESFALG(); //Mark all the data as read/stale
	void logESFALG(bool enabled = true); // Log data to file buffer

	bool getEsfInfo(uint16_t maxWait = defaultMaxWait); // ESF STATUS Helper
	bool getESFSTATUS(uint16_t maxWait = defaultMaxWait); // ESF STATUS
	bool setAutoESFSTATUS(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF STATUS reports
	bool setAutoESFSTATUS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF STATUS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoESFSTATUScallback(void (*callbackPointer)(UBX_ESF_STATUS_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoESFSTATUS(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and ESF STATUS is send cyclically already
	void flushESFSTATUS(); //Mark all the data as read/stale
	void logESFSTATUS(bool enabled = true); // Log data to file buffer

	bool getEsfIns(uint16_t maxWait = defaultMaxWait); // ESF INS Helper
	bool getESFINS(uint16_t maxWait = defaultMaxWait); // ESF INS
	bool setAutoESFINS(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF INS reports
	bool setAutoESFINS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF INS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoESFINScallback(void (*callbackPointer)(UBX_ESF_INS_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoESFINS(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and ESF INS is send cyclically already
	void flushESFINS(); //Mark all the data as read/stale
	void logESFINS(bool enabled = true); // Log data to file buffer

	bool getEsfDataInfo(uint16_t maxWait = defaultMaxWait); // ESF MEAS Helper
	bool getESFMEAS(uint16_t maxWait = defaultMaxWait); // ESF MEAS
	bool setAutoESFMEAS(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF MEAS reports
	bool setAutoESFMEAS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF MEAS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoESFMEAScallback(void (*callbackPointer)(UBX_ESF_MEAS_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic MEAS reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoESFMEAS(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and ESF MEAS is send cyclically already
	void flushESFMEAS(); //Mark all the data as read/stale
	void logESFMEAS(bool enabled = true); // Log data to file buffer

	bool getEsfRawDataInfo(uint16_t maxWait = defaultMaxWait); // ESF RAW Helper
	bool getESFRAW(uint16_t maxWait = defaultMaxWait); // ESF RAW
	bool setAutoESFRAW(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF RAW reports
	bool setAutoESFRAW(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic ESF RAW reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoESFRAWcallback(void (*callbackPointer)(UBX_ESF_RAW_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic RAW reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoESFRAW(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and ESF RAW is send cyclically already
	void flushESFRAW(); //Mark all the data as read/stale
	void logESFRAW(bool enabled = true); // Log data to file buffer

	// High navigation rate (HNR)

	bool getHNRAtt(uint16_t maxWait = defaultMaxWait); // HNR ATT Helper
	bool getHNRATT(uint16_t maxWait = defaultMaxWait); // Returns true if the get HNR attitude is successful
	bool setAutoHNRATT(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR Attitude reports at the HNR rate
	bool setAutoHNRATT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR Attitude reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoHNRATTcallback(void (*callbackPointer)(UBX_HNR_ATT_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoHNRATT(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and HNR Attitude is send cyclically already
	void flushHNRATT(); //Mark all the data as read/stale
	void logHNRATT(bool enabled = true); // Log data to file buffer

	bool getHNRDyn(uint16_t maxWait = defaultMaxWait); // HNR INS Helper
	bool getHNRINS(uint16_t maxWait = defaultMaxWait); // Returns true if the get HNR dynamics is successful
	bool setAutoHNRINS(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR dynamics reports at the HNR rate
	bool setAutoHNRINS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR dynamics reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoHNRINScallback(void (*callbackPointer)(UBX_HNR_INS_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoHNRINS(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and HNR dynamics is send cyclically already
	void flushHNRINS(); //Mark all the data as read/stale
	void logHNRINS(bool enabled = true); // Log data to file buffer

	bool getHNRPVT(uint16_t maxWait = defaultMaxWait); // Returns true if the get HNR PVT is successful
	bool setAutoHNRPVT(bool enabled, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR PVT reports at the HNR rate
	bool setAutoHNRPVT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait); //Enable/disable automatic HNR PVT reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
	bool setAutoHNRPVTcallback(void (*callbackPointer)(UBX_HNR_PVT_data_t), uint16_t maxWait = defaultMaxWait); //Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
	bool assumeAutoHNRPVT(bool enabled, bool implicitUpdate = true); //In case no config access to the GPS is possible and HNR PVT is send cyclically already
	void flushHNRPVT(); //Mark all the data as read/stale
	void logHNRPVT(bool enabled = true); // Log data to file buffer

	// Helper functions for CFG RATE

	bool setNavigationFrequency(uint8_t navFreq, uint16_t maxWait = defaultMaxWait);	 //Set the number of nav solutions sent per second
	uint8_t getNavigationFrequency(uint16_t maxWait = defaultMaxWait);					 //Get the number of nav solutions sent per second currently being output by module

	// Helper functions for DOP

	uint16_t getGeometricDOP(uint16_t maxWait = defaultMaxWait);
	uint16_t getPositionDOP(uint16_t maxWait = defaultMaxWait);
	uint16_t getTimeDOP(uint16_t maxWait = defaultMaxWait);
	uint16_t getVerticalDOP(uint16_t maxWait = defaultMaxWait);
	uint16_t getHorizontalDOP(uint16_t maxWait = defaultMaxWait);
	uint16_t getNorthingDOP(uint16_t maxWait = defaultMaxWait);
	uint16_t getEastingDOP(uint16_t maxWait = defaultMaxWait);

	// Helper functions for ATT

	float getATTroll(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	float getATTpitch(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	float getATTheading(uint16_t maxWait = defaultMaxWait); // Returned as degrees

	// Helper functions for PVT
	// All the function below sets the internal "queried" flagg to false, indacate the data is read by user.

	uint32_t getTimeOfWeek(uint16_t maxWait = defaultMaxWait);
	uint16_t getYear(uint16_t maxWait = defaultMaxWait);
	uint8_t getMonth(uint16_t maxWait = defaultMaxWait);
	uint8_t getDay(uint16_t maxWait = defaultMaxWait);
	uint8_t getHour(uint16_t maxWait = defaultMaxWait);
	uint8_t getMinute(uint16_t maxWait = defaultMaxWait);
	uint8_t getSecond(uint16_t maxWait = defaultMaxWait);
	uint16_t getMillisecond(uint16_t maxWait = defaultMaxWait);
	int32_t getNanosecond(uint16_t maxWait = defaultMaxWait);

	bool getDateValid(uint16_t maxWait = defaultMaxWait);
	bool getTimeValid(uint16_t maxWait = defaultMaxWait);

	uint8_t getFixType(uint16_t maxWait = defaultMaxWait); //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning

	bool getGnssFixOk(uint16_t maxWait = defaultMaxWait); //Get whether we have a valid fix (i.e within DOP & accuracy masks)
	bool getDiffSoln(uint16_t maxWait = defaultMaxWait); //Get whether differential corrections were applied
	bool getHeadVehValid(uint16_t maxWait = defaultMaxWait);
	uint8_t getCarrierSolutionType(uint16_t maxWait = defaultMaxWait); //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution

	uint8_t getSIV(uint16_t maxWait = defaultMaxWait); //Returns number of sats used in fix
	int32_t getLongitude(uint16_t maxWait = defaultMaxWait); //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getLatitude(uint16_t maxWait = defaultMaxWait); //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
	int32_t getAltitude(uint16_t maxWait = defaultMaxWait); //Returns the current altitude in mm above ellipsoid
	int32_t getAltitudeMSL(uint16_t maxWait = defaultMaxWait); //Returns the current altitude in mm above mean sea level
	int32_t getHorizontalAccEst(uint16_t maxWait = defaultMaxWait);
	int32_t getVerticalAccEst(uint16_t maxWait = defaultMaxWait);
	int32_t getNedNorthVel(uint16_t maxWait = defaultMaxWait);
	int32_t getNedEastVel(uint16_t maxWait = defaultMaxWait);
	int32_t getNedDownVel(uint16_t maxWait = defaultMaxWait);
	int32_t getGroundSpeed(uint16_t maxWait = defaultMaxWait); //Returns speed in mm/s
	int32_t getHeading(uint16_t maxWait = defaultMaxWait); //Returns heading in degrees * 10^-5
	uint32_t getSpeedAccEst(uint16_t maxWait = defaultMaxWait);
	uint32_t getHeadingAccEst(uint16_t maxWait = defaultMaxWait);
	uint16_t getPDOP(uint16_t maxWait = defaultMaxWait); //Returns positional dillution of precision * 10^-2 (dimensionless)

	bool getInvalidLlh(uint16_t maxWait = defaultMaxWait);

	int32_t getHeadVeh(uint16_t maxWait = defaultMaxWait);
	int16_t getMagDec(uint16_t maxWait = defaultMaxWait);
	uint16_t getMagAcc(uint16_t maxWait = defaultMaxWait);

	int32_t getGeoidSeparation(uint16_t maxWait = defaultMaxWait);
	
	void setPVTQueried();

	// Helper functions for HPPOSECEF

	uint32_t getPositionAccuracy(uint16_t maxWait = 1100); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,

	// Helper functions for HPPOSLLH

	uint32_t getTimeOfWeekFromHPPOSLLH(uint16_t maxWait = defaultMaxWait);
	int32_t getHighResLongitude(uint16_t maxWait = defaultMaxWait);
	int32_t getHighResLatitude(uint16_t maxWait = defaultMaxWait);
	int32_t getElipsoid(uint16_t maxWait = defaultMaxWait);
	int32_t getMeanSeaLevel(uint16_t maxWait = defaultMaxWait);
	int8_t getHighResLongitudeHp(uint16_t maxWait = defaultMaxWait);
	int8_t getHighResLatitudeHp(uint16_t maxWait = defaultMaxWait);
	int8_t getElipsoidHp(uint16_t maxWait = defaultMaxWait);
	int8_t getMeanSeaLevelHp(uint16_t maxWait = defaultMaxWait);
	uint32_t getHorizontalAccuracy(uint16_t maxWait = defaultMaxWait);
	uint32_t getVerticalAccuracy(uint16_t maxWait = defaultMaxWait);

	// Helper functions for SVIN

	bool getSurveyInActive(uint16_t maxWait = defaultMaxWait);
	bool getSurveyInValid(uint16_t maxWait = defaultMaxWait);
	uint16_t getSurveyInObservationTime(uint16_t maxWait = defaultMaxWait); // Truncated to 65535 seconds
	float getSurveyInMeanAccuracy(uint16_t maxWait = defaultMaxWait); // Returned as m

	// Helper functions for RELPOSNED

	float getRelPosN(uint16_t maxWait = defaultMaxWait); // Returned as m
	float getRelPosE(uint16_t maxWait = defaultMaxWait); // Returned as m
	float getRelPosD(uint16_t maxWait = defaultMaxWait); // Returned as m
	float getRelPosAccN(uint16_t maxWait = defaultMaxWait); // Returned as m
	float getRelPosAccE(uint16_t maxWait = defaultMaxWait); // Returned as m
	float getRelPosAccD(uint16_t maxWait = defaultMaxWait); // Returned as m

	// Helper functions for ESF

	float getESFroll(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	float getESFpitch(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	float getESFyaw(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	bool getSensorFusionMeasurement(UBX_ESF_MEAS_sensorData_t *sensorData, uint8_t sensor, uint16_t maxWait = defaultMaxWait);
	bool getSensorFusionMeasurement(UBX_ESF_MEAS_sensorData_t *sensorData, UBX_ESF_MEAS_data_t ubxDataStruct, uint8_t sensor);
	bool getRawSensorMeasurement(UBX_ESF_RAW_sensorData_t *sensorData, uint8_t sensor, uint16_t maxWait = defaultMaxWait);
	bool getRawSensorMeasurement(UBX_ESF_RAW_sensorData_t *sensorData, UBX_ESF_RAW_data_t ubxDataStruct, uint8_t sensor);
	bool getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, uint8_t sensor, uint16_t maxWait = defaultMaxWait);
	bool getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, UBX_ESF_STATUS_data_t ubxDataStruct, uint8_t sensor);

	// Helper functions for HNR

	bool setHNRNavigationRate(uint8_t rate, uint16_t maxWait = 1100); // Returns true if the setHNRNavigationRate is successful
	uint8_t getHNRNavigationRate(uint16_t maxWait = 1100); // Returns 0 if the getHNRNavigationRate fails
	float getHNRroll(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	float getHNRpitch(uint16_t maxWait = defaultMaxWait); // Returned as degrees
	float getHNRheading(uint16_t maxWait = defaultMaxWait); // Returned as degrees

	// Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
	// From v2.0: These are public. The user can call these to extract data from custom packets
	uint32_t extractLong(ubxPacket *msg, uint8_t spotToStart); //Combine four bytes from payload into long
	int32_t extractSignedLong(ubxPacket *msg, uint8_t spotToStart); //Combine four bytes from payload into signed long (avoiding any ambiguity caused by casting)
	uint16_t extractInt(ubxPacket *msg, uint8_t spotToStart); //Combine two bytes from payload into int
	int16_t extractSignedInt(ubxPacket *msg, int8_t spotToStart);
	uint8_t extractByte(ubxPacket *msg, uint8_t spotToStart); //Get byte from payload
	int8_t extractSignedChar(ubxPacket *msg, uint8_t spotToStart); //Get signed 8-bit value from payload

	// Pointers to storage for the "automatic" messages
	// RAM is allocated for these if/when required.

	UBX_NAV_POSECEF_t *packetUBXNAVPOSECEF = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_STATUS_t *packetUBXNAVSTATUS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_DOP_t *packetUBXNAVDOP = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_ATT_t *packetUBXNAVATT = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_PVT_t *packetUBXNAVPVT = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_ODO_t *packetUBXNAVODO = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_VELECEF_t *packetUBXNAVVELECEF = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_VELNED_t *packetUBXNAVVELNED = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_HPPOSECEF_t *packetUBXNAVHPPOSECEF = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_HPPOSLLH_t *packetUBXNAVHPPOSLLH = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_CLOCK_t *packetUBXNAVCLOCK = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_SVIN_t *packetUBXNAVSVIN = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_NAV_RELPOSNED_t *packetUBXNAVRELPOSNED = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	UBX_RXM_SFRBX_t *packetUBXRXMSFRBX = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_RXM_RAWX_t *packetUBXRXMRAWX = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	UBX_CFG_RATE_t *packetUBXCFGRATE = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	UBX_TIM_TM2_t *packetUBXTIMTM2 = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	UBX_ESF_ALG_t *packetUBXESFALG = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_ESF_INS_t *packetUBXESFINS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_ESF_MEAS_t *packetUBXESFMEAS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_ESF_RAW_t *packetUBXESFRAW = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_ESF_STATUS_t *packetUBXESFSTATUS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	UBX_HNR_PVT_t *packetUBXHNRPVT = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_HNR_ATT_t *packetUBXHNRATT = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
	UBX_HNR_INS_t *packetUBXHNRINS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

	uint16_t rtcmFrameCounter = 0; //Tracks the type of incoming byte inside RTCM frame

private:
	//Depending on the sentence type the processor will load characters into different arrays
	enum SentenceTypes
	{
		NONE = 0,
		NMEA,
		UBX,
		RTCM
	} currentSentence = NONE;

	//Depending on the ubx binary response class, store binary responses into different places
	enum classTypes
	{
		CLASS_NONE = 0,
		CLASS_ACK,
		CLASS_NOT_AN_ACK
	} ubxFrameClass = CLASS_NONE;

	enum commTypes
	{
		COMM_TYPE_I2C = 0,
		COMM_TYPE_SERIAL,
		COMM_TYPE_SPI
	} commType = COMM_TYPE_I2C; //Controls which port we look to for incoming bytes

	//Functions
	bool checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass = 255, uint8_t requestedID = 255); //Checks module with user selected commType
	void addToChecksum(uint8_t incoming);																		 //Given an incoming byte, adjust rollingChecksumA/B

	//Return true if this "automatic" message has storage allocated for it
	bool checkAutomatic(uint8_t Class, uint8_t ID);

	//Calculate how much RAM is needed to store the payload for a given automatic message
	uint16_t getMaxPayloadSize(uint8_t Class, uint8_t ID);

	bool initGeofenceParams(); // Allocate RAM for currentGeofenceParams and initialize it
	bool initModuleSWVersion(); // Allocate RAM for moduleSWVersion and initialize it

	// The initPacket functions need to be private as they don't check if memory has already been allocated.
	// Functions like setAutoNAVPOSECEF will check that memory has not been allocated before calling initPacket.
	bool initPacketUBXNAVPOSECEF(); // Allocate RAM for packetUBXNAVPOSECEF and initialize it
	bool initPacketUBXNAVSTATUS(); // Allocate RAM for packetUBXNAVSTATUS and initialize it
	bool initPacketUBXNAVDOP(); // Allocate RAM for packetUBXNAVDOP and initialize it
	bool initPacketUBXNAVATT(); // Allocate RAM for packetUBXNAVATT and initialize it
	bool initPacketUBXNAVPVT(); // Allocate RAM for packetUBXNAVPVT and initialize it
	bool initPacketUBXNAVODO(); // Allocate RAM for packetUBXNAVODO and initialize it
	bool initPacketUBXNAVVELECEF(); // Allocate RAM for packetUBXNAVVELECEF and initialize it
	bool initPacketUBXNAVVELNED(); // Allocate RAM for packetUBXNAVVELNED and initialize it
	bool initPacketUBXNAVHPPOSECEF(); // Allocate RAM for packetUBXNAVHPPOSECEF and initialize it
	bool initPacketUBXNAVHPPOSLLH(); // Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
	bool initPacketUBXNAVCLOCK(); // Allocate RAM for packetUBXNAVCLOCK and initialize it
	bool initPacketUBXNAVSVIN(); // Allocate RAM for packetUBXNAVSVIN and initialize it
	bool initPacketUBXNAVRELPOSNED(); // Allocate RAM for packetUBXNAVRELPOSNED and initialize it
	bool initPacketUBXRXMSFRBX(); // Allocate RAM for packetUBXRXMSFRBX and initialize it
	bool initPacketUBXRXMRAWX(); // Allocate RAM for packetUBXRXMRAWX and initialize it
	bool initPacketUBXCFGRATE(); // Allocate RAM for packetUBXCFGRATE and initialize it
	bool initPacketUBXTIMTM2(); // Allocate RAM for packetUBXTIMTM2 and initialize it
	bool initPacketUBXESFALG(); // Allocate RAM for packetUBXESFALG and initialize it
	bool initPacketUBXESFSTATUS(); // Allocate RAM for packetUBXESFSTATUS and initialize it
	bool initPacketUBXESFINS(); // Allocate RAM for packetUBXESFINS and initialize it
	bool initPacketUBXESFMEAS(); // Allocate RAM for packetUBXESFMEAS and initialize it
	bool initPacketUBXESFRAW(); // Allocate RAM for packetUBXESFRAW and initialize it
	bool initPacketUBXHNRATT(); // Allocate RAM for packetUBXHNRATT and initialize it
	bool initPacketUBXHNRINS(); // Allocate RAM for packetUBXHNRINS and initialize it
	bool initPacketUBXHNRPVT(); // Allocate RAM for packetUBXHNRPVT and initialize it

	//Variables
	//TwoWire *_i2cPort;				//The generic connection to user's chosen I2C hardware
	int _i2cFd;
	Serial *_serialPort;			//The generic connection to user's chosen Serial hardware
	Serial *_nmeaOutputPort = NULL; //The user can assign an output port to print NMEA sentences if they wish
	Serial *_debugSerial;			//The stream to send debug messages to if enabled

	uint8_t _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
	//This can be changed using the ublox configuration software

	bool _printDebug = false;		//Flag to print the serial commands we are sending to the Serial port for debug
	bool _printLimitedDebug = false; //Flag to print limited debug messages. Useful for I2C debugging or high navigation rates

	bool ubx7FcheckDisabled = false; // Flag to indicate if the "7F" check should be ignored in checkUbloxI2C

	//The packet buffers
	//These are pointed at from within the ubxPacket
	uint8_t payloadAck[2];				  // Holds the requested ACK/NACK
	uint8_t payloadBuf[2];				  // Temporary buffer used to screen incoming packets or dump unrequested packets
	size_t packetCfgPayloadSize = 0; // Size for the packetCfg payload. .begin will set this to MAX_PAYLOAD_SIZE if necessary. User can change with setPacketCfgPayloadSize
	uint8_t *payloadCfg = NULL;
	uint8_t *payloadAuto = NULL;

	//Init the packet structures and init them with pointers to the payloadAck, payloadCfg, payloadBuf and payloadAuto arrays
	ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
	ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
	ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
	ubxPacket packetAuto = {0, 0, 0, 0, 0, payloadAuto, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

	//Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
	bool ignoreThisPayload = false;

	//Identify which buffer is in use
	//Data is stored in packetBuf until the requested class and ID can be validated
	//If a match is seen, data is diverted into packetAck or packetCfg
	//"Automatic" messages which have RAM allocated for them are diverted into packetAuto
	sfe_ublox_packet_buffer_e activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;

	//Limit checking of new data to every X ms
	//If we are expecting an update every X Hz then we should check every half that amount of time
	//Otherwise we may block ourselves from seeing new data
	uint8_t i2cPollingWait = 100; //Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate()

	unsigned long lastCheck = 0;

	uint16_t ubxFrameCounter;			  //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]

	uint8_t rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
	uint8_t rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

	uint16_t rtcmLen = 0;

	// Flag to prevent reentry into checkCallbacks
	// Prevent badness if the user accidentally calls checkCallbacks from inside a callback
	volatile bool checkCallbacksReentrant = false;

	// Support for data logging
	uint8_t *ubxFileBuffer = NULL; // Pointer to the file buffer. RAM is allocated for this if required in .begin
	uint16_t fileBufferSize = 0; // The size of the file buffer. This can be changed by calling setFileBufferSize _before_ .begin
	uint16_t fileBufferHead; // The incoming byte is written into the file buffer at this location
	uint16_t fileBufferTail; // The next byte to be read from the buffer will be read from this location
	uint16_t fileBufferMaxAvail = 0; // The maximum number of bytes the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
	bool createFileBuffer(void); // Create the file buffer. Called by .begin
	uint16_t fileBufferSpaceAvailable(void); // Check how much space is available in the buffer
	uint16_t fileBufferSpaceUsed(void); // Check how much space is used in the buffer
	bool storePacket(ubxPacket *msg); // Add a UBX packet to the file buffer
	bool storeFileBytes(uint8_t *theBytes, uint16_t numBytes); // Add theBytes to the file buffer
	void writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes); // Write theBytes to the file buffer
	
	std::chrono::time_point<std::chrono::steady_clock,std::chrono::nanoseconds> tpSentenceStart, tpSentenceEnd;
};

#endif
