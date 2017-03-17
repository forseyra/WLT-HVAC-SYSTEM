#define SKETCH_VERSION "WLT_HVAC 2.5.0"	
#define SKETCH_DESCRIPTION "WLT_HVAC.ino	Version 2.5.0	10 March 2017	Original rforsey@tpg.com.au"

/*
	Arduino-based multi-zone HVAC (Heating, Cooling, Ventilation and Air Conditioning) system with web-based control,
	configuration and monitoring user interface.

	Copyright (C) 2015-2017 Roger Forsey

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
*/

/*	1.0.0	Original
	1.1.0	7 December 2015	Rewrite of temperatureControl - Theatre cooling.
	2.0.0	17 December 2015 Added controls to Weekly Schedule page, revised log file and SdFat error handling.
	2.0.1	21 December 2015 Added HTML logs.
	2.0.2	24 December 2015 Removed fan start delay from case where humidity has dropped below set point.
	2.1.0	2 January 2016 Added button press beep. Weekly Schedule page renamed Control. Web UI changes. Added sensor fault detection.
	2.1.1	12 January 2016 Streamed XML responces through xmlDataFormat() to reduce number of network packets.
	2.1.2	20 January 2016 Added version to web pages
	2.2.0	7 February 2016 Added ventilation function.
	2.2.1	15 December 2016 Ventilation bug fixes. Run web server every 2 ms
	2.3.0	29 January 2017 Fan speed controller support.
	2.4.0	9 February 2017 Removed DS18B20 from Stage and Dressing Room & replaced with DHT22 Temperature/Humidity sensors
	2.5.0	10 March 2017 New Fan, Heat and Humidity device handlers, added Foyer fan ventilate control. Updated zone state control.
	
*/


/*

 Udp NTP Client
 
 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket 
 For more on NTP time servers and the messages needed to communicate with them, 
 see http://en.wikipedia.org/wiki/Network_Time_Protocol
 
 Warning: NTP Servers are subject to temporary failure or IP address change.
 Plese check 
 
 http://tf.nist.gov/tf-cgi/servers.cgi
 
 if the time server used in the example didn't work.
 
 created 4 Sep 2010 
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe
 
 This code is in the public domain.
 
 */
 
// System Configuration

#define TRACE 4	// enable any tracing.

	const int LOG_INTERVAL = 5;	// Logging interval (minutes) interval in hour

	const int LOW_HEAT_BAND = 2;	// Temperatue difference between low and high heat
	const int alarmIdFlameout = 0;	//	Alarm ID
	const int alarmIdResetDelay = 1;	// Flameout reset timeout
	const char *sketchVersion = SKETCH_VERSION;
	const char *sketchDescription = SKETCH_DESCRIPTION;
	
	// SoftTimer DelayRun time in milliseconds
	const unsigned long FLAMEOUT_WATCHDOG_TIME = 30000;	// Flameout watchdog timeout time in milliseconds
	const unsigned long FLAMEOUT_RESET_TIME = 30000;	// Flameout reset time in milliseconds	
	const unsigned long COOL_PUMP_FAN_DELAY = 120000;	// Braemar recommend 2 - 5 minutes to wet pads.
	const unsigned long FAN_STOP_DELAY = 30000;	// Fan stop delay. Fan run time to cool gas heater after heating
	
	// Humidity fan control
	const bool maxHumidityControlFan = true;	// Humidity control fan state. 
	const bool noMaxHumidityControlFan = false;	// No fan control if zoneMaxHumidity changes state

	// Control Panel zone mode switch digital input pins
/*
	Control Panel zone mode switch digital input pins.
	
	Pin 1	Pin 0			Return	
		1 		0 = Off			0
		1		1 = Run			1
		0		1 = Auto		2	
*/
	const byte zoneTheatreSwitchPin0 = 34;
	const byte zoneTheatreSwitchPin1 = 36;

	const byte zoneFoyerSwitchPin0 = 38;
	const byte zoneFoyerSwitchPin1 = 40;
	

// #define WEBDUINO_SERIAL_DEBUGGING  2
#define WEBDUINO_SUPRESS_SERVER_HEADER
#define WEBDUINO_FAIL_MESSAGE "<h1>Request Failed</h1>"
#define VERSION_STRING "0.1"
#define WEBDUINO_OUTPUT_BUFFER_SIZE 254	// Limit = 254 because buffer is uint8_t ( 8 bits)
#define WEBDUINO_COMMANDS_COUNT 20 // Number of commands
// How long to wait before considering a connection as dead when
// reading the HTTP request.  Used to avoid DOS attacks.
#define WEBDUINO_READ_TIMEOUT_IN_MS 2000

#define  XML_DATA_FORMAT_BUFFER_LIMIT 180	// xmlDataFormat() auto server.print limit. (Send TCP/IP packet)

// #define dtNBR_ALARMS 20	// Max number of TimeAlarms
 
#include <SPI.h>

#include "avr/pgmspace.h"        
#include <Ethernet.h>
#include <WebServer.h>


// #include <EthernetUdp.h>

// RTC Time Set

// #include <SoftI2C.h> // initialise required libraries

#include <Time.h>	//
#include <TimeAlarms.h>	// https://github.com/johnmccombs/arduino-libraries/tree/master/TimeAlarms

#include <Wire.h>
#include <DS3232RTC.h>	// https://github.com/Tecsmith/DS3232RTC

#include <Timezone.h>    // https://github.com/JChristensen/Timezone

#include <EEPROMex.h>	// https://github.com/thijse/Arduino-Libraries/tree/master/EEPROMEx

#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <SoftTimer.h>	// https://github.com/prampec/arduino-softtimer/blob/wiki/SoftTimer.md
#include <DelayRun.h>	// https://github.com/prampec/arduino-softtimer/blob/wiki/SoftTimer.md
#include <SdFat.h>	// https://github.com/greiman/SdFat
#include <WString.h>	//

// arduino-softtimer  https://code.google.com/p/arduino-softtimer/

// Configure softTimer Timed tasks

// softTimer functional prototypes - required for IDE 1.6.6

// -- taskTempHumidity will be launched every 10 seconds.
	void tempHumidity() {}
Task taskTempHumidity(10000, tempHumidity);

// -- taskTemperatureControl will be launched on every 15 seconds.
	void controlHVAC() {}
Task taskTemperatureControl(15000, controlHVAC);

// -- taskDisplayLCD will be launched on every 3 seconds.
	void displayLCD() {}
Task taskDisplayLCD(3000, displayLCD);

// -- taskWebServer will be launched every 5 Ms.
	void runWebServer() {}
Task taskWebServer(5, runWebServer);

// -- taskSensorLoger will be launched every second.
	void sensorLogger() {}
 Task taskSensorLogger(1000, sensorLogger);

 // -- taskScheduler will be launched every 5 seconds.
	void runScheduler() {}
 Task taskScheduler(5000, runScheduler); 
 
 // -- taskSchedulerAutoSave will be started after any update to the Weekly Schedule.
	void runSchedulerAutoSave() {}
	DelayRun schedulerAutoSaveTask(30000, runSchedulerAutoSave);
 
// -- taskSetupAutoSave will be started after any update to the Setup.
 	void runSetupAutoSave() {}
	DelayRun setupAutoSaveTask(30000, runSetupAutoSave);


	// Delay run the get temp task. Delay time ms  >= conversion time
	void getDsTemperatures() {}
	DelayRun getTempTask(200, getDsTemperatures);

	// Delay run Foyer fan after start of Cool pump. NB Not required for Foyer
	void runFoyerFanSpeed() {}
	DelayRun runFoyerFanTask(COOL_PUMP_FAN_DELAY, runFoyerFanSpeed);
		
	// Delay run Theatre fan after start of Cool pump. Braemar recommend 2 - 5 minutes to wet pads.
	void runTheatreFanSpeed() {}
	DelayRun runTheatreFanTask(COOL_PUMP_FAN_DELAY, runTheatreFanSpeed);
	

	// Delay stop Foyer fan after heating
	void delayedFoyerFanStop() {}
	DelayRun delayedFoyerFanTask(FAN_STOP_DELAY, delayedFoyerFanStop);
	
	// Delay stop Theatre fan after heating
	void delayedTheatreFanStop() {}
	DelayRun delayedTheatreFanTask(FAN_STOP_DELAY, delayedTheatreFanStop);

	// Delared Theatre flameout check - checks duct temperature after heater start timeout
	void flameoutCheck() {}
	DelayRun delayedFlameoutTask(FLAMEOUT_WATCHDOG_TIME, flameoutCheck);
	
	// Theatre flameout reset timer 
	void theatreFlameoutTimer() {}
	DelayRun theatreFlameoutTimerTask(FLAMEOUT_RESET_TIME, theatreFlameoutTimer);
	
	// Init for system log.

	// SD chip select pin
	// const uint8_t chipSelect = SS;
	const uint8_t chipSelect = 4;
	
	//------------------------------------------------------------------------------
	// Permit SD to be wiped if ALLOW_WIPE is true.
	const bool ALLOW_WIPE = false;

	// File system object.
	SdFat sd;

	// Use for file creation in folders.
	SdFile file;

	// Create a Serial output stream.
	ArduinoOutStream cout(Serial);

	// Buffer for Serial input.
	char cinBuf[40];

	// Create a serial input stream.
	ArduinoInStream cin(Serial, cinBuf, sizeof(cinBuf));
	//==============================================================================
	// Error messages stored in flash.
	#define error(msg) sd.errorHalt(F(msg))
	//------------------------------------------------------------------------------
	
	// End init system log.

	const int addressAuMelbourneTZ = 0;	// Address in EEPROM for timezone change rules.
	
	
	// Timezone ausET(addressAuMelbourneTZ);       // Timezone rules stored at EEPROM address.
	TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev


	// initialize the library with the numbers of the interface pins
	//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
	LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );

	// SoftI2C i2c(20, 21); // assign pins to SDA and SCL
	// DS3232RTC rtc(i2c);

	// Enter a MAC address for your controller below.
	// Newer Ethernet shields have a MAC address printed on a sticker on the shield
	byte mac[] = {  
	  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

	unsigned int localPort = 8888;      // local port to listen for UDP packets
	
	// For more on NTP time servers and the messages needed to communicate with them, 
	// see http://en.wikipedia.org/wiki/Network_Time_Protocol

	// IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov NTP server
	// IPAddress timeServer(132, 163, 4, 102); // time-b.timefreq.bldrdoc.gov NTP server
	// IPAddress timeServer(132, 163, 4, 103); // time-c.timefreq.bldrdoc.gov NTP server
	IPAddress timeServer(129, 127, 40, 3); //  ntp.adelaide.edu.au NTP server	// Use AU time source.

	const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message

	byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets 

	// Global Strings
	// Create the local date string object
	String strLocalDate;
	
	// Create the local time string object
	String strLocalTime;
	// Create log year month string object
	String strLogYearMonth;
	// Create html log year month string object
	String strHtmlLogYearMonth;	
	
	String strLog;
	
	String strXmlData;	// String for xml data buffer

	String strBuffer0;	// mask for ?idxx=0 responce
	String strBuffer1;	// mask for ?idxx=1 responce

	// A UDP instance to let us send and receive packets over UDP
	EthernetUDP Udp;

	// 8-channel relay driver shield
	const bool relayOn = true;		// Set relay state = on
	const bool relayOff = false;	// Set relay state = off

	uint8_t relayBankA = 0;	// Relay Bank A bit settings. Contains a bit for each relay latch.

	char csvFlagByte[] = {"0,0,0,0,0,0,0,0"};	// byte flags as csv string.

	#define I2C_ADDR  0x20  // 0x20 is the address with all jumpers removed
	

	//Temperature Sensors

	bool sensorsOnline = false;	// true = sensors scan running at least once.
	bool sensorsOK = true;	// Sensors OK

	// Define DHT pin mappings.
	const int DHT_1 = 22;	// Theatre
	const int DHT_2 = 24;	// Duct
	const int DHT_3 = 26;	// Foyer
	const int DHT_4 = 30;	// Outside
	const int DHT_5 = 32;	// Stage
	const int DHT_6 = 28;	// Dressing Room
	
	
	const int g_numberOfZones = 6;  // Number of A/C sensor zones.
	const int scheduleHours = 24;	// Number of entries in schedule day arrays.
	const int numSchedulePressButtons = 2;	// Size of press buttons array.

	const char *zoneName[] = {"Theatre", "A/C Duct", "Foyer", "Outside", "Dressing Room", "Stage"};

	// Index zone data arrays by name.
	const int zoneTheatre = 0;
	const int zoneDuct = 1;
	const int zoneFoyer = 2;
	const int zoneOutside = 3;
	const int zoneDressingRoom = 4;
	const int zoneStage = 5;

	// Relay latch bit mapping.

	const int relayTheatreHeat = 1;
	const int relayFoyerHeat = 2;
	const int relayExternalFan = 4;
	const int relayCoolPump = 8;
	const int relayTheatreRunFan = 16;
	const int relayFanSpeedController = 32;
	const int relayFoyerRunFan = 64;
	const int relayTheatrePowerController = 128;
	
	// Zone switch on control panel
	byte zoneSwitchState[] = {0, 0, 0, 0, 0, 0};
	
	const byte zoneSwitchOff = 0;	// Zone is Off
	const byte zoneSwitchRun = 1;	// Zone is under control from switches on control panel
	const byte zoneSwitchAuto = 2;	// Zone is under computer control
	
	
	byte zoneState[] = {0, 0, 0, 0, 0, 0};

/*  0 = Manual Off
	1 = Manual Run
	2 = Auto Schedule Off
	3 = Auto Schedule On
	4 = Auto Run
	5 = Auto Off
	6 = Flameout restart
	7 = Ventilate check enabled
	8 = Ventilate fan running
	*/
	const byte zoneStateOff = 0;
	const byte zoneStateRun = 1;
	const byte zoneStateScheduleOff = 2;
	const byte zoneStateScheduleOn = 3;
	const byte zoneStateManualOff = 4;
	const byte zoneStateManualOn = 5;
	const byte zoneStateFlameout = 6;
	const byte zoneStateVentilateCheck = 7;
	const byte zoneStateVentilateFanRunning = 8;
	
	byte zoneRestartState[] = {0, 0, 0, 0, 0, 0};	// Holds zoneStateManualOn or zoneStateScheduleOn for flameout re-start mode.
	
	byte weeklyScheduleState = 0;	// Weekly schedule state 
	
	const byte scheduleOff = 0;	// Weekly Schedule is Off
	const byte scheduleOn = 1;	// Weekly Schedule is On
	
	const byte fanOff = 0;
	const byte fanDelayedStop = 1;
	const byte fanLowCoolStart = 2;
	const byte fanLowCool = 3;
	const byte fanLowRun = 4;
	const byte fanMediumCoolStart = 5;
	const byte fanMediumCool = 6;
	const byte fanMediumRun = 7;
	const byte fanHighCoolStart = 8;
	const byte fanHighCool = 9;
	const byte fanHighRun = 10;
	
	byte zoneFanState[] = {0, 0, 0, 0, 0, 0};	// zone Fan state

	byte zoneHeatState[] = {0, 0, 0, 0, 0, 0};	// zone Heat control state
	
	const byte heatOff = 0;
	const byte heatLow = 1;
	const byte heatHigh = 2;
	const byte heatFlameout = 3;

	byte zoneCoolPumpState[] = {0, 0, 0, 0, 0, 0};	// zone Cool Pump control state
		
	const byte coolPumpOff = 0;
	const byte coolPumpOn = 1;
	
	const byte powerOff = 0;
	const byte powerOn = 1;

	const byte statusVentilateOff = 0;	// Ventilate this zone Off
	const byte statusVentilateOn = 1;	// Ventilate this zone On

	bool zoneHumidityAvailable[] = {true, true, true, true, true, true};	// Humidity sensor is available in this zone
	bool zoneHighHumidity[] = {false, false, false, false, false, false};	// Zone high humidity flag

	// Arrays for DHT temperature and humidity readings.

	float zoneTemperature[] = {0, 0, 0, 0, 0, 0};	// Zone Temperature
	float zoneHumidity[] = {0, 0, 0, 0, 0, 0};	// Zone Humidity
	
	time_t systemStartTime;	// System start time
	
	byte displayPageLCD = 1;	// LCD display page number
	
	// System error counters
	
	int zoneSensorFaults[] = {0, 0, 0, 0, 0, 0};	// Count of sensor errors since system start
	int zoneFlameouts[] = {0, 0, 0, 0, 0, 0};	// Count of flameouts since system start
	
	// ***** The following settings are restored from EEPROM     ****
	
	// Arrays for zone thermostat setting and hysteresis
	float zoneThermostatSetting[] = {20, 20, 20, 20, 20, 20};	// Set point On temperature
	float zoneThermostatHysteresis[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; //Difference in degrees between set temperature & on & off settings.
	
	float zoneCoolThermostatSetting[] = {23, 23, 23, 23, 23, 23};	// Cool On temperature
	float zoneCoolThermostatHysteresis[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; //Difference in degrees between set temperature & on & off settings.
	
	float zoneMaxHumidity[] = {60, 60, 60, 60, 60, 60};	// Maximum humidity in zone for evaporative cooling pump control.
	float zoneMaxHumidityHysteresis[] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; //Difference in % humidity between set on & off settings.
	
	float zoneCoolHighFanPoint[] = {27, 27, 27, 27, 27, 27};	// Run high fan speed if zone temperature above this point.
	float zoneCoolHighFanHysteresis[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; 

	// Weekly schedule arrays. Hour 0 - 23, true = on, false = off
	byte schSunday[24] = { 0 };
	byte schMonday[24] = { 0 };
	byte schTuesday[24] = { 0 };
	byte schWednesday[24] = { 0 };
	byte schThursday[24] = { 0 };
	byte schFriday[24] = { 0 };
	byte schSaturday[24] = { 0 };
	
	byte weeklyScheduleArray[] = {0, 0};	// Weekly Schedule enabled = 1
	
	byte zoneVentilateArray[] = {0, 2, 0, 2, 2, 2, 2};	// Ventilate Zone enable. 0 = Off, 1 = On, 2 = N/A (Disabled)

	//	*****	End of restored settings.
	
	// Arrays for state of Form input "Set". 0 = clear, 1 = user has changed input value.
	byte zoneThermostatSettingSet[] = {0,0,0,0,0,0,0};
	byte zoneThermostatHysteresisSet[] = {0,0,0,0,0,0,0};
	
	byte zoneCoolThermostatSettingSet[] = {0,0,0,0,0,0,0};
	byte zoneCoolThermostatHysteresisSet[] = {0,0,0,0,0,0,0};
	
	byte zoneCoolHighFanPointSet[] = {0,0,0,0,0,0,0};
	byte zoneCoolHighFanHysteresisSet[] = {0,0,0,0,0,0,0};
	
	byte zoneMaxHumiditySet[] = {0,0,0,0,0,0,0};
	byte zoneMaxHumidityHysteresisSet[] = {0,0,0,0,0,0,0};
		
	
	
	// schedule array changed record. true = entry changed. Used by auto update.
	bool schSundayChanged = false;
	bool schMondayChanged = false;
	bool schTuesdayChanged = false;
	bool schWednesdayChanged = false;
	bool schThursdayChanged = false;
	bool schFridayChanged = false;
	bool schSaturdayChanged = false;
	bool weeklyScheduleArrayChanged = false;
	
	// Setup change record. Used by auto update. True = entry changed
	bool zoneVentilateSettingChanged = false;
	bool zoneThermostatSettingChanged = false;
	bool zoneThermostatHysteresisChanged = false;
	
	bool zoneCoolThermostatSettingChanged = false;
	bool zoneCoolThermostatHysteresisChanged = false;	
	
	bool zoneCoolHighFanPointChanged = false;
	bool zoneCoolHighFanHysteresisChanged = false;
	
	bool zoneMaxHumidityChanged = false;
	bool zoneMaxHumidityHysteresisChanged = false;
	
	const char *g_dayTag;	// Ajax changed schedule Id prefix
	int g_dayTagNumber;	// Ajax Id number
	byte *g_schDay;	// Pointer to schedule day

	int g_tagNum = 0;
	const char *g_tagName;
	int g_index = 0;
	int g_nextLogMinute = 0;	// next minute in hour to run logging.

	int address = 0;	// base EEPROM address.

	// Construct copy of local timezone to get size in bytes for EEPROM address allocation.
	//Australia Eastern Time Zone (Sydney, Melbourne)
	TimeChangeRule aEDT = {"AEDT", First, Sun, Oct, 2, 660};    //UTC + 11 hours
	TimeChangeRule aEST = {"AEST", First, Sun, Apr, 3, 600};    //UTC + 10 hours
	Timezone ausET(aEDT, aEST);
	
	// EEPROM address definitions
	
	int addressAusET = EEPROM.getAddress(sizeof(aEDT) + sizeof(aEST));
	int addressLastModifiedTime = EEPROM.getAddress(sizeof(time_t));
	
    int addressZoneThermostatSetting = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
    int addressZoneThermostatHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
	
    int addressZoneCoolThermostatSetting = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
    int addressZoneCoolThermostatHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
	
    int addressZoneMaxHumidity = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
    int addressZoneMaxHumidityHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);	
	
	int addressZoneCoolHighFanPoint = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
	int addressZoneCoolHighFanHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);	

	// Weekly Schedule day / hour settings arrays
	int addressSchSunday = EEPROM.getAddress(sizeof schSunday / sizeof *schSunday);
	int addressSchMonday = EEPROM.getAddress(sizeof schMonday / sizeof *schMonday);
	int addressSchTuesday = EEPROM.getAddress(sizeof schTuesday / sizeof *schTuesday);
	int addressSchWednesday = EEPROM.getAddress(sizeof schWednesday / sizeof *schWednesday);
	int addressSchThursday = EEPROM.getAddress(sizeof schThursday / sizeof *schThursday);
	int addressSchFriday = EEPROM.getAddress(sizeof schFriday / sizeof *schFriday);
	int addressSchSaturday = EEPROM.getAddress(sizeof schSaturday / sizeof *schSaturday);
	
	int addressWeeklyScheduleArray = EEPROM.getAddress(sizeof weeklyScheduleArray / sizeof *weeklyScheduleArray);
	int addressZoneVentilateArray = EEPROM.getAddress(sizeof zoneVentilateArray / sizeof *zoneVentilateArray);
 
	
	// Uncomment whatever type you're using!
	//#define DHTTYPE DHT11   // DHT 11
	#define DHTTYPE DHT22   // DHT 22  (AM2302)
	//#define DHTTYPE DHT21   // DHT 21 (AM2301)

	// Create DHT object for each DHT sensor
	// DHT dht(DHTPIN, DHTTYPE);
	DHT dht1(DHT_1, DHTTYPE);
	DHT dht2(DHT_2, DHTTYPE);
	DHT dht3(DHT_3, DHTTYPE);
	DHT dht4(DHT_4, DHTTYPE);
	DHT dht5(DHT_5, DHTTYPE);
	DHT dht6(DHT_6, DHTTYPE);
	
	/*
	// DS18x20 Temperature Sensor
	// Data wire is plugged into port 28 on the Arduino
	#define ONE_WIRE_BUS 28
	#define TEMPERATURE_PRECISION 9	// Conversion time = 94 ms
	//#define TEMPERATURE_PRECISION 10	// Conversion time = 188 ms
	//#define TEMPERATURE_PRECISION 11	// Conversion time = 375 ms
	//#define TEMPERATURE_PRECISION 12	// Conversion time = 750 ms

	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	OneWire oneWire(ONE_WIRE_BUS);

	// Pass our oneWire reference to Dallas Temperature.
	DallasTemperature sensors(&oneWire);

	// arrays to hold device addresses. 
	DeviceAddress insideThermometer, outsideThermometer;

	*/

	// web server
	
	// ROM-based messages used by the application
	// These are needed to avoid having the strings use up our limited
	//    amount of RAM.

P(Page_start) = "<html><head><title>Web_Parms_1 Version " VERSION_STRING "</title></head><body>\n";
P(Page_end) = "</body></html>";
P(Get_head) = "<h1>GET from ";
P(Post_head) = "<h1>POST to ";
P(Unknown_head) = "<h1>UNKNOWN request for ";
P(Default_head) = "unidentified URL requested.</h1><br>\n";
P(Raw_head) = "raw.html requested.</h1><br>\n";
P(Parsed_head) = "parsed.html requested.</h1><br>\n";
P(Good_tail_begin) = "URL tail = '";
P(Bad_tail_begin) = "INCOMPLETE URL tail = '";
P(Tail_end) = "'<br>\n";
P(Parsed_tail_begin) = "URL parameters:<br>\n";
P(Parsed_item_separator) = " = '";
P(Params_end) = "End of parameters<br>\n";
P(Post_params_begin) = "Parameters sent by POST:<br>\n";
P(Line_break) = "<br>\n";


	/* This creates an instance of the webserver.  By specifying a prefix
	* of "", all pages will be at the root of the server. */
	#define PREFIX ""
	WebServer webserver(PREFIX, 80);
	
/* commands are functions that get called by the webserver framework
 * they can read any posted data from client, and they output to the
 * server to send data back to the web browser. */
void helloCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

  server.printP(Page_start);
  switch (type)
    {
    case WebServer::GET:
        server.printP(Get_head);
        break;
    case WebServer::POST:
        server.printP(Post_head);
        break;
    default:
        server.printP(Unknown_head);
    }

    server.printP(Default_head);
    server.printP(tail_complete ? Good_tail_begin : Bad_tail_begin);
    server.print(url_tail);
    server.printP(Tail_end);
    server.printP(Page_end);

}

void rkjsCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
	getFileNamed(server, type, "rk.js", "text/html");	// GET the file
}
	
void scheduleCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {

	getFileNamed(server, type, "control.htm", "text/html");	// GET the file
}
void indexCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
	getFileNamed(server, type, "monitor.htm", "text/html");	// GET the file
}

void settingsCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {

	getFileNamed(server, type, "settings.htm", "text/html");	// GET the file
}

void setupCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
	getFileNamed(server, type, "setup.htm", "text/html");	// GET the file
}

void bgroundjpgCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
	getFileNamed(server, type, "bground.jpg", "text/html");	// GET the file
}

void wltcssCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
	getFileNamed(server, type, "wlt.css", "text/css");	// GET the file
}

// Purpose:	Respond to a GET "file name" request.

void getFileNamed(WebServer &server, WebServer::ConnectionType type, const char *getFileName, const char *contentType) {

	int c;
	
	if (contentType == "text/html") {
		/* this line sends the standard "we're all OK" headers back to the
			browser */
		server.httpSuccess();
		
	} else if (contentType == "text/css" ) {
		// This line sends the Content type text/css headers back to the browser 
		server.print("HTTP/1.1 200 OK\r\nContent-Type: text/css\r\nConnection: keep-alive\r\n\r\n");
	}

	/* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
	if (type == !WebServer::GET)
    return;

	// Change current directory to root.
	if (!sd.chdir()) {
		printDateTime();
		Serial.println(F("getFileNamed chdir to root failed"));
		return;
	}
	// open file in current working directory
	ifstream file(getFileName);
	
	if (!file.is_open()) {
		printDateTime();
		Serial.print(F("GET "));Serial.print(getFileName); Serial.println(F(" file open failed."));
		return;
		}
	while ((c = file.get()) >= 0) {
	server.print((char)c);
	}
	file.close();
	
}

// Purpose:	Respond to a GET "log file" request.
void logCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {

	int c;
	char* tokensArray[6];	// Split string tokens.
	char * pch;
	
  /* this line sends the standard "we're all OK" headers back to the
     browser */
	server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
	if (type == !WebServer::GET)
    return;
	// Change current directory to root.
	if (!sd.chdir()) {
		printDateTime();
		Serial.println(F("logCmd chdir to root failed"));
		return;
	}
	// Change to Logs directory
	if (!sd.chdir("/LOGS")) {
		printDateTime();
		Serial.println(F("logCmd: chdir failed for LOGS"));
		return;
	}
	Serial.print("Url Tail = ");
	Serial.println(url_tail);
	
	// open file in current working directory
	ifstream file(url_tail);

	if (!file.is_open()) {
		printDateTime();
		Serial.print("Failed to open log file ");
		Serial.println(url_tail);
		
		server.httpFail();
		return;
	}

	while ((c = file.get()) >= 0) {
	server.print((char)c);
	// debug Serial.print((char)c);
	}
	file.close();
	
	// Check if file has .htm extension

	// Split FileName into file name and extension.
	int tokensIndex = 0;
	pch = strtok (url_tail,".");	// String to search, tokens for string "."

	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, ".");
	}

	Serial.print("File extension = ");
	Serial.println(tokensArray[1]);
	
	if (tokensArray[1] == "htm" || tokensArray[1] == "HTM") {
		// Close HTML page & Table
		server.print("</tr></table></body></html>");
	}
	if (!sd.chdir()) {
		Serial.println(F("logCmd exit chdir to root failed"));
	}
}
// User has clicked on a ventilate button
void setupPressButtonCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}

	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");
	/*
	Serial.println();
	Serial.println(url_tail);
	*/
	checkButtonClick(url_tail);	// Check for any setup click button.
	XML_setup_response(webserver);	// Send XML response
	zoneVentilateSettingChanged = true;	// Mark change
	// Start/restart auto save timeout.
	setupAutoSaveTask.startDelayed();	// Delayed start of Setup autosave.	
}
void setupXmlCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}

	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");

	Serial.println();
	Serial.println(url_tail);
	
	checkSetupButtons(url_tail);	// Check for any setup button or input changes.
	
	XML_setup_response(webserver);	// Send XML response
	
}
// User clicked on Form input
void formInputClickedXmlCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
	const char *one = "1";

	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}
	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");

	updateOnClick(url_tail); // On click from setup.

	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Init
	xmlDataFormat(strXmlData, g_tagName, g_tagNum, one, server);
	strXmlData = String(strXmlData + (F("</responce>")));
	server.print(strXmlData);
	
}
// User has clicked on a Setup page Form Set button
void formInputSetXmlCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
	const char *one = "1";
	const char *zero = "0";
	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}
	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");

	checkSetupButtons(url_tail);	// Check for any setup button or input changes.

	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Response
	xmlDataFormat(strXmlData, g_tagName, g_tagNum, zero, server);
	strXmlData = String(strXmlData +  (F("</responce>")));
	server.print(strXmlData);

	// Start/restart auto save timeout.
	setupAutoSaveTask.startDelayed();	// Delayed start of Setup autosave.

}

void monitorXmlCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
 	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}
	server.print((F("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n")));

	// Check heading for any button settings
	if (strlen(url_tail) > 0) SetButtons(url_tail);
	XML_response(webserver);	// Send XML response
 }
void controlXmlCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
 	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}
	server.print((F("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n")));

	// Check heading for any button settings
	if (strlen(url_tail) > 0) SetButtons(url_tail);
	controlXmlResponse(webserver);	// Send XML response
 }

 // send the XML file containing analog value
void controlXmlResponse(WebServer &server) {

	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Init
	
	// Schedule On/Off Radio buttons
	xmlDataFormat(strXmlData, "pressButton_", 0, byte2str(weeklyScheduleArray[0]), server);

	// Theatre State
	xmlDataFormat(strXmlData, "theatreState", 9999, byte2str(zoneState[zoneTheatre]), server);

	// Foyer State
	xmlDataFormat(strXmlData, "foyerState", 9999, byte2str(zoneState[zoneFoyer]), server);
	
	strXmlData = String(strXmlData +  (F("</responce>")));
	server.print(strXmlData);
}
 
// send the XML file containing digital & analog values
void XML_response(WebServer &server) {

	char caLocalTime[9];
	char caLocalDate[11];
	byte xmlData;
	char XmlContentText[10];	// dtostrf  double/float to converted string conversion buffer.

	localDate(strLocalDate);	// Update local date string
	localTime(strLocalTime);	// Update local time string
	
	strLocalTime.toCharArray(caLocalTime, 9);
	strLocalDate.toCharArray(caLocalDate, 11);

	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Init
	
	xmlDataFormat(strXmlData, "pressButton_", 0, byte2str(weeklyScheduleArray[0]), server);
	
	xmlDataFormat(strXmlData, "theatreState", 9999, byte2str(zoneState[zoneTheatre]), server);

	xmlDataFormat(strXmlData, "foyerState", 9999, byte2str(zoneState[zoneFoyer]), server);

	xmlDataFormat(strXmlData, "lastUpdateDate", 9999, caLocalDate, server);

	xmlDataFormat(strXmlData, "lastUpdateTime", 9999, caLocalTime, server);
	
	xmlDataFormat(strXmlData, "version", 9999, sketchVersion, server);
	
	xmlDataFormat(strXmlData, "theatreHeat", 9999, byte2str(zoneHeatState[zoneTheatre]), server);
	
	xmlDataFormat(strXmlData, "fanSpeed", 9999, byte2str(zoneFanState[zoneTheatre]), server);

	xmlDataFormat(strXmlData, "coolPump", 9999, byte2str(zoneCoolPumpState[zoneTheatre]), server);
	
	xmlDataFormat(strXmlData, "foyerHeat", 9999, byte2str(zoneHeatState[zoneFoyer]), server);
	
	xmlDataFormat(strXmlData, "foyerFanSpeed", 9999, byte2str(zoneFanState[zoneFoyer]), server);

	xmlDataFormat(strXmlData, "analog1", 9999, dtostrf(zoneTemperature[zoneTheatre],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog2", 9999, dtostrf(zoneTemperature[zoneFoyer],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog3", 9999, dtostrf(zoneTemperature[zoneOutside],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog4", 9999, dtostrf(zoneTemperature[zoneStage],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog5", 9999, dtostrf(zoneTemperature[zoneDressingRoom],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog6", 9999, dtostrf(zoneTemperature[zoneDuct],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog7", 9999, dtostrf(zoneHumidity[zoneTheatre],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog8", 9999, dtostrf(zoneHumidity[zoneFoyer],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog9", 9999, dtostrf(zoneHumidity[zoneOutside],4 ,2 , XmlContentText), server);

	xmlDataFormat(strXmlData, "analog10", 9999, dtostrf(zoneHumidity[zoneStage],4 ,2 , XmlContentText), server);
	
	xmlDataFormat(strXmlData, "analog11", 9999, dtostrf(zoneHumidity[zoneDressingRoom],4 ,2 , XmlContentText), server);
		
	xmlDataFormat(strXmlData, "analog12", 9999, dtostrf(zoneHumidity[zoneDuct],4 ,2 , XmlContentText), server);
	
	strXmlData = String(strXmlData +  (F("</responce>")));
	server.print(strXmlData);
}

void scheduleXmlCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  
	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}

	server.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n"));

	XML_schedule_response(webserver);	// Send XML response
 }

void scheduleButtonCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  
	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}
	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");
	// Check heading for any button settings
	checkScheduleButtons(url_tail);	// Scan and change schedule button. Also set global g_dayTag and g_dayTagNumber
	XML_schedule_button_response(webserver);	// Send XML response
	
	// Start/restart auto save timeout.
	schedulerAutoSaveTask.startDelayed();	// Delayed start of Weekly Schedule autosave.	
 } 
	// Radio press buttons
 void schedulePressButtonCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  
	if (type == !WebServer::GET) {
		server.httpFail();
		return;
	}
	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");
	// Check heading for any button settings
	checkSchedulePressButtons(url_tail);	// Scan and change schedule press buttons.
	XML_schedule_pressButton_response(webserver);	// Send XML response
	
	// Start/restart auto save timeout.
	schedulerAutoSaveTask.startDelayed();	// Delayed start of Weekly Schedule autosave.
 } 
 
void tableCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  
	if (type == !WebServer::GET) {

	server.httpFail();
	return;
	}

	server.print("HTTP/1.1 200 OK\r\nContent-Type: text/xml\r\nConnection: keep-alive\r\n\r\n");

	XML_table_response(webserver);	// Send XML table response
}
/* Purpose: Converts byte to int in range of 0 - 254
	Returns -1 if 255
*/	
int byteToInt(byte number) {
	byte byteNumber = 0;
	for (int returnInt = 0; returnInt <= 254; returnInt++) {
		if (number == byteNumber)	{
			return returnInt;
		}
		byteNumber++;
	}
	return -1;
}	

/*	Purpose: Convert int (signed 16-bit integer) into string.
	Accepts: Signed 16-bit integer.
	Returns: Signed number string in global _int2str
*/

char _int2str[7];	// Global return string

const char* int2str(int i)	{

	// Convert signer 16 bit integer to string

	//	char _int2str[7];	// This return buffer must be declared globally outside int2str

	char* int2str(register int i ); 
	register unsigned char L = 1;
	register char c;
	register boolean m = false;
	register char b;  // lower-byte of i
	// negative
	if ( i < 0 ) {
	_int2str[ 0 ] = '-';
	i = -i;
	}
	else L = 0;
	// ten-thousands
	if( i > 9999 ) {
	c = i < 20000 ? 1
	  : i < 30000 ? 2
	  : 3;
	_int2str[ L++ ] = c + 48;
	i -= c * 10000;
	m = true;
	}
	// thousands
	if( i > 999 ) {
	c = i < 5000
	  ? ( i < 3000
		  ? ( i < 2000 ? 1 : 2 )
		  :   i < 4000 ? 3 : 4
		)
	  : i < 8000
		? ( i < 6000
			? 5
			: i < 7000 ? 6 : 7
		  )
		: i < 9000 ? 8 : 9;
	_int2str[ L++ ] = c + 48;
	i -= c * 1000;
	m = true;
	}
	else if( m ) _int2str[ L++ ] = '0';
	// hundreds
	if( i > 99 ) {
	c = i < 500
	  ? ( i < 300
		  ? ( i < 200 ? 1 : 2 )
		  :   i < 400 ? 3 : 4
		)
	  : i < 800
		? ( i < 600
			? 5
			: i < 700 ? 6 : 7
		  )
		: i < 900 ? 8 : 9;
	_int2str[ L++ ] = c + 48;
	i -= c * 100;
	m = true;
	}
	else if( m ) _int2str[ L++ ] = '0';
	// decades (check on lower byte to optimize code)
	b = char( i );
	if( b > 9 ) {
	c = b < 50
	  ? ( b < 30
		  ? ( b < 20 ? 1 : 2 )
		  :   b < 40 ? 3 : 4
		)
	  : b < 80
		? ( i < 60
			? 5
			: i < 70 ? 6 : 7
		  )
		: i < 90 ? 8 : 9;
	_int2str[ L++ ] = c + 48;
	b -= c * 10;
	m = true;
	}
	else if( m ) _int2str[ L++ ] = '0';
	// last digit
	_int2str[ L++ ] = b + 48;
	// null terminator
	_int2str[ L ] = 0;  

/*	
	Serial.print("int2str() returns:");
	Serial.println(_int2str);
*/	
	return _int2str;
	
}

/*	Purpose: Converts an unsigned byte to char string.
	Accepts: byte with an unsigned number, from 0 to 255.
	Returns: Global char array  _byte2str[4] contains number string.
*/

char _byte2str[4];	// Global return string

const char* byte2str(register byte i)	{

	// Convert Arduino unsigned byte number to string

	//	char _int2str[4];	// This return buffer must be declared globally outside byte2str

	char* int2str(byte i ); 
	register  unsigned char L = 0;
	register char c;
	register boolean m = false;
	register char b;  // lower-byte of i
	
/*
	// negative
	if ( i < 0 ) {
	_byte2str[ 0 ] = '-';
	i = -i;
	}
	else L = 0;
	// ten-thousands
	if( i > 9999 ) {
	c = i < 20000 ? 1
	  : i < 30000 ? 2
	  : 3;
	_byte2str[ L++ ] = c + 48;
	i -= c * 10000;
	m = true;
	}
	// thousands
	if( i > 999 ) {
	c = i < 5000
	  ? ( i < 3000
		  ? ( i < 2000 ? 1 : 2 )
		  :   i < 4000 ? 3 : 4
		)
	  : i < 8000
		? ( i < 6000
			? 5
			: i < 7000 ? 6 : 7
		  )
		: i < 9000 ? 8 : 9;
	_byte2str[ L++ ] = c + 48;
	i -= c * 1000;
	m = true;
	}
	else if( m ) _byte2str[ L++ ] = '0';
*/
	L = 0;
	// hundreds
	if( i > 99 ) {
	c = i < 500
	  ? ( i < 300
		  ? ( i < 200 ? 1 : 2 )
		  :   i < 400 ? 3 : 4
		)
	  : i < 800
		? ( i < 600
			? 5
			: i < 700 ? 6 : 7
		  )
		: i < 900 ? 8 : 9;
	_byte2str[ L++ ] = c + 48;
	i -= c * 100;
	m = true;
	}
	else if( m ) _byte2str[ L++ ] = '0';
	// decades (check on lower byte to optimize code)
	b = char( i );
	if( b > 9 ) {
	c = b < 50
	  ? ( b < 30
		  ? ( b < 20 ? 1 : 2 )
		  :   b < 40 ? 3 : 4
		)
	  : b < 80
		? ( i < 60
			? 5
			: i < 70 ? 6 : 7
		  )
		: i < 90 ? 8 : 9;
	_byte2str[ L++ ] = c + 48;
	b -= c * 10;
	m = true;
	}
	else if( m ) _byte2str[ L++ ] = '0';
	// last digit
	_byte2str[ L++ ] = b + 48;
	// null terminator
	_byte2str[ L ] = 0;

/*
	Serial.print("byte2str() returns:");
	Serial.println(_byte2str);
*/	
	
	return _byte2str;
}



const char* bool2str(bool boolean) {
	// Convert boolean to True or False.
	if (boolean == true)
		return "True";
	else
		return "False";
	}
const char* boolToNum(bool boolean) {
	// Convert boolean True to 1 or False to 0.
	if (boolean == true)
		return "1";
	else
		return "0";
}
/* Purpose: Searches URL tail for setup ventilate button clicks.
		Accepts: url_tail = Item Id and button state e.g. zoneVentilate_3=1
*/	
void checkButtonClick(char *url_tail) {
	
	char* tokensArray[6];	// Split string tokens.
	char * pch;
	byte iDNumber;
	// Split url_tail into tag number and value.
	int tokensIndex = 0;
	pch = strtok (url_tail,"_=&");	// String to search, tokens for string like zoneVentilate_6=1&ran=7642

	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, "_=&");
	}	
	if (strcmp(tokensArray[0], "zoneVentilate") == 0) {
		iDNumber = atoi(tokensArray[1]);	// Convert to index 0
		iDNumber--;
		if (iDNumber >= 0 && iDNumber < g_numberOfZones) {
			if (atoi(tokensArray[2]) == 0)	{
				zoneVentilateArray[iDNumber] =  0;
			} 
			else if (atoi(tokensArray[2]) == 1)	{
				zoneVentilateArray[iDNumber] =  1;
			}
			else if (atoi(tokensArray[2]) == 2)	{
				zoneVentilateArray[iDNumber] =  2;
			}
		}
	}		
}	

/* Purpose: Searches URL tail for setting change and updates system setting.
	Accepts: url_tail = Item Id and setting e.g. zoneThermostatSetting_3=21
*/
void checkSetupButtons(char *url_tail) {

	const int MaxTagNum = 6;	// Number of Tag data rows

	// Array of possible responce prefixes.
	const char* setupResponceArray[] = {"zoneThermostatSetting","zoneThermostatHysteresis","zoneCoolThermostatSetting","zoneCoolThermostatHysteresis","zoneCoolHighFanPoint","zoneCoolHighFanHysteresis","zoneMaxHumidity","zoneMaxHumidityHysteresis"};
	const int setupResponceArraySize = 7;	// Number of items in array - 1
	char* tokensArray[6];	// Split string tokens.
	char * pch;

	String strValue = "";	// String to hold value for string to float conversion
	float valueFloat;

	// Split url_tail into tag number and value.
	int tokensIndex = 0;
	pch = strtok (url_tail,"_=&");	// String to search, tokens for string like zoneThermostatSetting_6=21.5&ran=7642

	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, "_=&");
	}
/*	--tokensIndex;
	Serial.println(F("tokensArray"));
	
	for (int i = 0; i <= tokensIndex; i++)	{
		Serial.println(tokensArray[i]);
	}
*/
	// validate token Id

	for (int index = 0; index <= setupResponceArraySize; index++)	{
		if (strcmp(tokensArray[0], setupResponceArray[index]) == 0) {
			
			for (int tagNum = 1; tagNum <= MaxTagNum; tagNum++)	{
				if (atoi(tokensArray[1]) == tagNum)	{
		
					// update the value array
					String strValue = tokensArray[2];
					valueFloat = (strValue.toFloat());	// Convert string to float

					updateValue(index, atoi(tokensArray[1]), valueFloat);
					updateValueInt(index, atoi(tokensArray[1]), 0);	// also update corresponding "on Click" state
				}
			}
		break;
		}
	}	
}	
/*	Purpose: Converts index to address of value array and updates value.
	Also sets Changed flag for autosave
*/
void updateValue(int index, int tagNum, float value)	{

	/*
	Serial.print(F("index = ");
	Serial.println(index);
	
	Serial.print(F("tagNum = ");
	Serial.println(tagNum);
	Serial.println(value);
	*/
	int tagIndex = tagNum - 1;	// Array base index starts at 0
	
	switch (index) {
		case 0:
		zoneThermostatSetting[tagIndex] = value;
		zoneThermostatSettingChanged = true;
		break;
		case 1:
		zoneThermostatHysteresis[tagIndex] = value;
		zoneThermostatHysteresisChanged = true;
		break;
		
		case 2:
		zoneCoolThermostatSetting[tagIndex] = value;
		zoneCoolThermostatSettingChanged = true;
		break;
		case 3:
		zoneCoolThermostatHysteresis[tagIndex] = value;
		zoneCoolThermostatHysteresisChanged = true;
		break;		
		
		case 4:
		zoneCoolHighFanPoint[tagIndex] = value;
		zoneCoolHighFanPointChanged = true;
		break;
		case 5:
		zoneCoolHighFanHysteresis[tagIndex] = value;
		zoneCoolHighFanHysteresisChanged = true;
		break;		
		
		case 6:
		zoneMaxHumidity[tagIndex] = value;
		zoneMaxHumidityChanged = true;
		break;
		case 7:
		zoneMaxHumidityHysteresis[tagIndex] = value;
		zoneMaxHumidityHysteresisChanged = true;
			
		
	}
}

/* Purpose: Searches URL tail for setup Form input On click state change and updates setting.
	Accepts: url_tail = Item Id and setting e.g. zoneThermostatSettingSet_3=1
*/
void updateOnClick(char *url_tail) {

	const int MaxTagNum = 6;	// Number of Tag data rows

	// Array of possible responce prefixes.
	const char* setupResponceArray[] = {"zoneThermostatSettingSet","zoneThermostatHysteresisSet","zoneCoolThermostatSettingSet","zoneCoolThermostatHysteresisSet","zoneCoolHighFanPointSet","zoneCoolHighFanHysteresisSet","zoneMaxHumiditySet","zoneMaxHumidityHysteresisSet"};
	const int setupResponceArraySize = 7;	//Number of items in array
	char* tokensArray[6];	// Split string tokens.
	char * pch;

	String strValue = "";	// String to hold value for string to float conversion
	float valueFloat;
 
	// Split url_tail into tag number and value.
	int tokensIndex = 0;
	pch = strtok (url_tail,"_=&");	// String to search, tokens for string like zoneThermostatSettingSet_6=1&ran=7642

	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, "_=&");
	}
/*	--tokensIndex;
	Serial.println(F("tokensArray"));
	
	for (int i = 0; i <= tokensIndex; i++)	{
		Serial.println(tokensArray[i]);
	}
*/
	// validate token Id

	for (int index = 0; index <= setupResponceArraySize; index++)	{
		if (strcmp(tokensArray[0], setupResponceArray[index]) == 0) {
			
			for (int tagNum = 1; tagNum <= MaxTagNum; tagNum++)	{
				if (atoi(tokensArray[1]) == tagNum)	{
					updateValueInt(index, atoi(tokensArray[1]), atoi(tokensArray[2]));
				}
			}
		break;
		}
	}	
}	
/*	Purpose: Converts index to address of value array and updates value
*/
void updateValueInt(int index, int tagNum, int value)	{

	/*
	Serial.println(F("updateValueInt"));
	Serial.print(F("index = "));
	Serial.println(index);*/
	g_index = index;	// Set global
	/*Serial.print(F("tagNum = "));
	Serial.println(tagNum); */
	g_tagNum = tagNum;	// Set global
	
	/*Serial.print(F("value = "));
	Serial.println(value); */
	
	switch (index) {
		
		case 0:
		zoneThermostatSettingSet[tagNum] = value;
		g_tagName = "zoneThermostatSettingSet_";	// Set Global
		break;
		case 1:
		zoneThermostatHysteresisSet[tagNum] = value;
		g_tagName = "zoneThermostatHysteresisSet_";	// Set Global
		break;
		
		case 2:
		zoneCoolThermostatSettingSet[tagNum] = value;
		g_tagName = "zoneCoolThermostatSettingSet_";	// Set Global
		break;
		case 3:
		zoneCoolThermostatHysteresisSet[tagNum] = value;
		g_tagName = "zoneCoolThermostatHysteresisSet_";	// Set Global
		break;		
		
		case 4:
		zoneCoolHighFanPointSet[tagNum] = value;
		g_tagName = "zoneCoolHighFanPointSet_";	// Set Global
		break;
		case 5:
		zoneCoolHighFanHysteresisSet[tagNum] = value;
		g_tagName = "zoneCoolHighFanHysteresisSet_";	// Set Global
		break;		
		
		case 6:
		zoneMaxHumiditySet[tagNum] = value;
		g_tagName = "zoneMaxHumiditySet_";	// Set Global
		break;
		case 7:
		zoneMaxHumidityHysteresisSet[tagNum] = value;
		g_tagName = "zoneMaxHumidityHysteresisSet_";	// Set Global
		break;		
		
		
		// Serial.print(F("g_tagName = "));
		// Serial.println(g_tagName);
	
	}
}

// send the XML file containing analog value
void XML_table_response(WebServer &server)
{
    char XmlContentText[10];	// dtostrf  double/float to converted string conversion buffer.
	int index = 0;

//	server.print("<?xml version = \"1.0\" ?>\r\n");
	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Init
	
	xmlDataFormat(strXmlData, "version", 9999, sketchVersion, server);	// Sketch version

	for (int num = 1; num <= 6; num++){
		
		xmlDataFormat(strXmlData, "zoneName_", num, zoneName[index], server);

		xmlDataFormat(strXmlData, "zoneStatus_", num, byte2str(zoneState[index]), server);

		xmlDataFormat(strXmlData, "zoneTemperature_", num, dtostrf(zoneTemperature[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneHumidity_", num, dtostrf(zoneHumidity[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneThermostatSetting_", num, dtostrf(zoneThermostatSetting[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneThermostatHysteresis_", num, dtostrf(zoneThermostatHysteresis[index],4 ,2 , XmlContentText), server);
		
		xmlDataFormat(strXmlData, "zoneCoolThermostatSetting_", num, dtostrf(zoneCoolThermostatSetting[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolThermostatHysteresis_", num, dtostrf(zoneCoolThermostatHysteresis[index],4 ,2 , XmlContentText), server);
		
		xmlDataFormat(strXmlData, "zoneCoolHighFanPoint_", num, dtostrf(zoneCoolHighFanPoint[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolHighFanHysteresis_", num, dtostrf(zoneCoolHighFanHysteresis[index],4 ,2 , XmlContentText), server);
		
		xmlDataFormat(strXmlData, "zoneMaxHumidity_", num, dtostrf(zoneMaxHumidity[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneMaxHumidityHysteresis_", num, dtostrf(zoneMaxHumidityHysteresis[index],4 ,2 , XmlContentText), server);
		
		xmlDataFormat(strXmlData, "zoneSensorFaults_", num, int2str(zoneSensorFaults[index]), server);
		
		xmlDataFormat(strXmlData, "zoneFlameouts_", num, int2str(zoneFlameouts[index]), server);
		
		index++;
	}
	strXmlData = String(strXmlData +  (F("</responce>")));
	server.print(strXmlData);

}

// send the XML file containing setup settings
void XML_setup_response(WebServer &server)
{
    char XmlContentText[10];	// dtostrf  double/float to converted string conversion buffer.
	int index = 0;

//	server.print("<?xml version = \"1.0\" ?>\r\n");
    strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Init
	
	xmlDataFormat(strXmlData, "version", 9999, sketchVersion, server);	// Sketch version

	for (int num = 1; num <= g_numberOfZones; num++){
		
		xmlDataFormat(strXmlData, "zoneName_", num, zoneName[index], server);
		
		xmlDataFormat(strXmlData, "zoneVentilate_", num, byte2str(zoneVentilateArray[index]), server);	// 0 base array offset

		xmlDataFormat(strXmlData, "zoneStatus_", num, byte2str(zoneState[index]), server);

/*		xmlDataFormat(strXmlData, "zoneTemperature_", num, dtostrf(zoneTemperature[index],4 ,2 , XmlContentText));
		server.print(strXmlData);
		xmlDataFormat(strXmlData, "zoneHumidity_", num, dtostrf(zoneHumidity[index],4 ,2 , XmlContentText));
		server.print(strXmlData);
*/		xmlDataFormat(strXmlData, "zoneThermostatSetting_", num, dtostrf(zoneThermostatSetting[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneThermostatSettingSet_", num, byte2str(zoneThermostatSettingSet[index]), server);

		xmlDataFormat(strXmlData, "zoneThermostatHysteresis_", num, dtostrf(zoneThermostatHysteresis[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneThermostatHysteresisSet_", num, byte2str(zoneThermostatHysteresisSet[index]), server);

		xmlDataFormat(strXmlData, "zoneCoolThermostatSetting_", num, dtostrf(zoneCoolThermostatSetting[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolThermostatSettingSet_", num, byte2str(zoneCoolThermostatSettingSet[index]), server);

		xmlDataFormat(strXmlData, "zoneCoolThermostatHysteresis_", num, dtostrf(zoneCoolThermostatHysteresis[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolThermostatHysteresisSet_", num, dtostrf(zoneCoolThermostatHysteresisSet[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolHighFanPoint_", num, dtostrf(zoneCoolHighFanPoint[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolHighFanPointSet_", num, byte2str(zoneCoolHighFanPointSet[index]), server);

		xmlDataFormat(strXmlData, "zoneCoolHighFanHysteresis_", num, dtostrf(zoneCoolHighFanHysteresis[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneCoolHighFanHysteresisSet_", num, dtostrf(zoneCoolHighFanHysteresisSet[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneMaxHumidity_", num, dtostrf(zoneMaxHumidity[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneMaxHumiditySet_", num, byte2str(zoneMaxHumiditySet[index]), server);

		xmlDataFormat(strXmlData, "zoneMaxHumidityHysteresis_", num, dtostrf(zoneMaxHumidityHysteresis[index],4 ,2 , XmlContentText), server);

		xmlDataFormat(strXmlData, "zoneMaxHumidityHysteresisSet_", num, dtostrf(zoneMaxHumidityHysteresisSet[index],4 ,2 , XmlContentText), server);

/*		xmlDataFormat(strXmlData, "zoneHumidityAvailable_", num, bool2str(zoneHumidityAvailable[index]));
		server.print(strXmlData);
*/		
		index++;
	}
	strXmlData = String(strXmlData +  "</responce>");
	server.print(strXmlData);
}

// send the XML file containing schedule settings
void XML_schedule_response(WebServer &server)
{ 
	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));	// Init
	
	xmlDataFormat(strXmlData, "version", 9999, sketchVersion, server);	// Sketch version

	// Sunday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "Su_", num, byte2str(schSunday[num]), server);
	}
	// Monday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "Mo_", num, byte2str(schMonday[num]), server);
	}
	server.print(strXmlData);
	// Tuesday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "Tu_", num, byte2str(schTuesday[num]), server);
	}
	// Wednesday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "We_", num, byte2str(schWednesday[num]), server);
	}
	// Thursday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "Th_", num, byte2str(schThursday[num]), server);
	}
	// Friday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "Fr_", num, byte2str(schFriday[num]), server);
	}
	// Saturday
	for (int num = 0; num <= 23; num++){
		xmlDataFormat(strXmlData, "Sa_", num, byte2str(schSaturday[num]), server);
	}
	strXmlData = String(strXmlData +  "</responce>");
	server.print(strXmlData);	
}

// send the XML file containing changed schedule button.
void XML_schedule_button_response(WebServer &server)
{ 
	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>\r\n"));	// Xml string.
	xmlDataFormat(strXmlData, g_dayTag, g_dayTagNumber, byte2str(g_schDay[g_dayTagNumber]), server);
	strXmlData = (strXmlData + "</responce>");
	server.print(strXmlData);
}

// send the XML file containing changed schedule button.
void XML_schedule_pressButton_response(WebServer &server)
{ 
	strXmlData = (F("<?xml version = \"1.0\" ?>\r\n<responce>"));
 	xmlDataFormat(strXmlData, "pressButton_", 0, byte2str(weeklyScheduleArray[0]), server);
	strXmlData = (strXmlData + "</responce>");
	server.print(strXmlData);
}

// checks if received HTTP request has button press.
	void SetButtons(char *url_tail) {
	
	char* tokensArray[6];	// Split string tokens.
	char * pch;

	// Split url_tail into tag number and value.
	int tokensIndex = 0;
	pch = strtok (url_tail,"_=&");	// String to search, tokens for string like pressButton_1=1&ran=3158
	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, "_=&");
	}
/*	
	--tokensIndex;

	Serial.println(F("tokensArray"));
	
	for (int i = 0; i <= tokensIndex; i++)	{
		Serial.println(tokensArray[i]);
	}
*/
	if (strcmp(tokensArray[0], "pressButton") == 0) {
		if (atoi(tokensArray[1]) == 0) {
			if (atoi(tokensArray[2]) == 0) {
				zoneControl(zoneTheatre, zoneStateManualOff);
				Serial.println(F("zoneStateManualOff"));
			}
			else if (atoi(tokensArray[2]) == 1) {
					zoneControl(zoneTheatre, zoneStateManualOn);
					Serial.println(F("zoneStateManualOn"));
			}	
		}
		if (atoi(tokensArray[1]) == 1) {
			if (atoi(tokensArray[2]) == 0) {
				zoneControl(zoneFoyer, zoneStateManualOff);
			}
			else if (atoi(tokensArray[2]) == 1) {
					zoneControl(zoneFoyer, zoneStateManualOn);
			}	
		}
	}
}	

/* Purpose: Searches URL tail for Schedule On click state change and updates setting.
	Accepts: url_tail = Item Id and setting e.g. Sa_13=1&ran=3158
*/
void checkScheduleButtons(char *url_tail) {

	const int MaxTagNum = 23;	// Hours in day

	// Array of possible responce prefixes.
	const char* scheduleResponceArray[] = {"Su","Mo","Tu","We","Th","Fr","Sa"};
	const int scheduleResponceArraySize = 6;	//Number of items in array - 1
	char* tokensArray[6];	// Split string tokens.
	char * pch;

	String strValue = "";	// String to hold value for string to float conversion
	float valueFloat;
 
	// Split url_tail into tag number and value.
	int tokensIndex = 0;
	pch = strtok (url_tail,"_=&");	// String to search, tokens for string like Sa_13=1&ran=3158

	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, "_=&");
	}
/*	--tokensIndex;

	Serial.println(F("tokensArray"));
	
	for (int i = 0; i <= tokensIndex; i++)	{
		Serial.println(tokensArray[i]);
	}
*/
	// validate token Id

	for (int index = 0; index <= scheduleResponceArraySize; index++)	{
		if (strcmp(tokensArray[0], scheduleResponceArray[index]) == 0) {
			
			for (int tagNum = 0; tagNum <= MaxTagNum; tagNum++)	{
				if (atoi(tokensArray[1]) == tagNum)	{
					updateValueIntSchedule(index, atoi(tokensArray[1]), atoi(tokensArray[2]));
					
				}
			}
		break;
		}
	}	
}	
/*	Purpose: Converts index to address of schedule array and updates value
	Also sets data changed flag.
*/
void updateValueIntSchedule(int index, int tagNum, int value)	{

	/*
	Serial.println(F("updateValueIntSchedule"));
	Serial.print(F("index = "));
	Serial.println(index);*/

	/*Serial.print(F("tagNum = "));
	Serial.println(tagNum); */
	
	/*Serial.print(F("value = "));
	Serial.println(value); */
	// Update globals
	
	g_dayTagNumber = tagNum;	// Global schedule number.		
	
	switch (index) {
		
		case 0:
		schSunday[tagNum] = value;
		schSundayChanged = true;
		g_schDay = schSunday;	// Global return of pointer to day schedule array
		g_dayTag = "Su_";	//// Global return day Id prefix. eg Su_
		break;
		case 1:
		schMonday[tagNum] = value;
		schMondayChanged = true;
		g_schDay = schMonday;
		g_dayTag = "Mo_";
		break;
		case 2:
		schTuesday[tagNum] = value;
		schTuesdayChanged = true;
		g_schDay = schTuesday;
		g_dayTag = "Tu_";
		break;
		case 3:
		schWednesday[tagNum] = value;
		schWednesdayChanged = true;
		g_schDay = schWednesday;
		g_dayTag = "We_";
		break;
		case 4:
		schThursday[tagNum] = value;
		schThursdayChanged = true;
		g_schDay = schThursday;
		g_dayTag = "Th_";
		break;
		case 5:
		schFriday[tagNum] = value;
		schFridayChanged = true;
		g_schDay = schFriday;
		g_dayTag = "Fr_";
		break;
		case 6:
		schSaturday[tagNum] = value;
		schSaturdayChanged = true;
		g_schDay = schSaturday;
		g_dayTag = "Sa_";
		break;
						
		// Serial.print(F("g_tagName = "));
		// Serial.println(g_tagName);
	
	}
}

void checkSchedulePressButtons(char *url_tail) {

	const int MaxTagNum = 1;	// Number of press buttons in page
	char* tokensArray[6];	// Split string tokens.
	char * pch;

	// Split url_tail into tag number and value.
	int tokensIndex = 0;
	pch = strtok (url_tail,"_=&");	// String to search, tokens for string like pressButton_1=1&ran=3158

	while (pch != NULL)
	{
	tokensArray[tokensIndex++] = pch;
	pch = strtok (NULL, "_=&");
	}
/*	--tokensIndex;
	Serial.println(F("tokensArray"));
	
	for (int i = 0; i <= tokensIndex; i++)	{
		Serial.println(tokensArray[i]);
	}
*/
	if (strcmp(tokensArray[0], "pressButton") == 0) {
		
		for (int tagNum = 0; tagNum <= MaxTagNum; tagNum++)	{
			if (atoi(tokensArray[1]) == tagNum)	{
				weeklyScheduleArray[tagNum] = atoi(tokensArray[2]);	// 1 or 0
				weeklyScheduleArrayChanged = true;	// Flag for autosave
			}
		}
	}
}

void my_failCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  /* this line sends the "HTTP 400 - Bad Request" headers back to the
     browser */
  server.httpFail();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

  server.printP(Page_start);
  switch (type)
    {
    case WebServer::GET:
        server.printP(Get_head);
        break;
    case WebServer::POST:
        server.printP(Post_head);
        break;
    default:
        server.printP(Unknown_head);
    }

    server.printP(Default_head);
    server.printP(tail_complete ? Good_tail_begin : Bad_tail_begin);
    server.print(url_tail);
    server.printP(Tail_end);
    server.printP(Page_end);

}


//         ========================= setup ===================================

void setup() 
{
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only

  }
	Serial.println();
	Serial.println(F(SKETCH_DESCRIPTION));
	
	strXmlData.reserve(256);	// Reserve 256 bytes for string manipulation
	
	// Setup the Chip Select lines.

	pinMode(4, OUTPUT);      // sets the digital pin as output
	digitalWrite(4, HIGH);   // SD Card SPI SS

	pinMode(10, OUTPUT);      // sets the digital pin as output
	digitalWrite(10, HIGH);   //  Ethernet SPI SS 

	// pinMode(53, OUTPUT); digitalWrite(53, HIGH);  // atmega2560 SS line

	// Digital inputs
	// Configure pin as an input and enable the internal pull-up resistor
	pinMode(zoneTheatreSwitchPin0, INPUT_PULLUP);	
	pinMode(zoneTheatreSwitchPin1, INPUT_PULLUP);	
	pinMode(zoneFoyerSwitchPin0, INPUT_PULLUP);	
	pinMode(zoneFoyerSwitchPin1, INPUT_PULLUP);	

	// Configure EEPROM

	const int maxAllowedWrites = 300;
	const int memBase          = 0;
	
	// start reading from position memBase (address 0) of the EEPROM. Set maximumSize to EEPROMSizeMega
    // Writes before membase or beyond EEPROMSizeMega will only give errors when _EEPROMex_DEBUG is set
    EEPROM.setMemPool(memBase, EEPROMSizeMega);
	
    // Set maximum allowed writes to maxAllowedWrites.
    // More writes will only give errors when _EEPROMex_DEBUG is set
    EEPROM.setMaxAllowedWrites(maxAllowedWrites);

	restoreSettings();	// Restore the system settings from EEPROM
 
	lcd.begin(16, 2);

	digitalWrite(4, HIGH);   // Restore SD Card SPI SS after LCD use

  //  SetTCPIPMode();
  

	// Start Ethernet and UDP
	if (Ethernet.begin(mac) == 0) {
	
		Serial.println(F("Failed to configure Ethernet using DHCP"));

	} else {

		Serial.print(F("I/P address :  "));
		Serial.println(Ethernet.localIP());
		Serial.print(F("Subnet Mask : "));
		Serial.println(Ethernet.subnetMask());
		Serial.print(F("Default Gateway : "));
		Serial.println(Ethernet.gatewayIP());
		Serial.print(F("DNS Server : "));
		Serial.println(Ethernet.dnsServerIP()); 

		Udp.begin(localPort);
		
		if (GetNTPTime() >0 ) {
			Serial.println(F("NTP Time sync OK"));
		}
		else {
			Serial.println(F("NTP Time sync failed"));
		}
	}
  
	// Init system time.
	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if(timeStatus()!= timeSet) {
		Serial.println(F("Unable to sync with the RTC"));
	} else {
		Serial.println(F("RTC has set the system time"));
	}

	printDateTime();
	Serial.println(F("Started"));
	systemStartTime = now();	// Store system systemStartTime
	
	// Ensure that all relays are off.
	foyerFanControl(fanOff);	// Init foyerFanControl
	theatreFanControl(fanOff);	// Init theatreFanControl
	zoneHighHumidity[zoneTheatre] = false;	// Clear high humidity flag
	
	// Init the next LOG_INTERVAL minute time in hour for logging. EG if minute now is 6, nextLogMinute is set to 10
	time_t timeNow = now(); // Store the current time in time variable timeNow 

	g_nextLogMinute = roundUp(minute(timeNow), LOG_INTERVAL);	// Set next log time

	if (g_nextLogMinute >= 60) g_nextLogMinute = 0;	// Cycle nextLogMinute 

//	g_nextLogMinute = 5 * (( minute(t) + 5 - 1) / 5);
	
	Serial.print(F("Next logging minute = "));
	Serial.println(g_nextLogMinute);
		
	// Set run time schedule. This must be after the system time is set.
 
 	// const int alarmIdFlameout = 0;	//	Alarm ID
	// const int alarmIdResetDelay = 1;	// Flameout reset timeout


	// initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
	// breadboards.  use SPI_FULL_SPEED for better performance.
	if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
		printDateTime();
		Serial.println(F("Initialize SD card failed"));
	} else {
		printDateTime();
		Serial.println(F("Initialize SD card OK"));;
	}

	// Initialise DHT sensors
	dht1.begin();
	dht2.begin();
	dht3.begin();
	dht4.begin();
	dht5.begin();
	dht6.begin();
	
	/* Remove DS18B20 support

	// Start up the library
	sensors.begin();

	// locate devices on the bus
	Serial.print(F("Locating devices..."));
	Serial.print(F("Found "));
	Serial.print(sensors.getDeviceCount(), DEC);
	Serial.println(F(" devices."));

	// report parasite power requirements
	Serial.print(F("Parasite power is: "));
	if (sensors.isParasitePowerMode()) Serial.println(F("ON"));
	else Serial.println(F("OFF"));
	
	// assign address manually.  the addresses below will beed to be changed
	// to valid device addresses on your bus.  device address can be retrieved
	// by using either oneWire.search(deviceAddress) or individually via
	// sensors.getAddress(deviceAddress, index)
	//insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
	//outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

	// search for devices on the bus and assign based on an index.  ideally,
	// you would do this to initially discover addresses on the bus and then
	// use those addresses and manually assign them (see above) once you know
	// the devices on your bus (and assuming they don't change).
	//
	// method 1: by index
	
	if (!sensors.getAddress(insideThermometer, 0)) Serial.println(F("Unable to find address for Device 0"));
	if (!sensors.getAddress(outsideThermometer, 1)) Serial.println(F("Unable to find address for Device 1"));

	// method 2: search()
	// search() looks for the next device. Returns 1 if a new address has been
	// returned. A zero might mean that the bus is shorted, there are no devices,
	// or you have already retrieved all of them.  It might be a good idea to
	// check the CRC to make sure you didn't get garbage.  The order is
	// deterministic. You will always get the same devices in the same order
	//
	// Must be called before search()
	//oneWire.reset_search();
	// assigns the first address found to insideThermometer
	//if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
	// assigns the seconds address found to outsideThermometer
	//if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

	// show the addresses we found on the bus
	/*
	Serial.print("Device 0 Address: ");
	printAddress(insideThermometer);
	Serial.println();

	Serial.print("Device 1 Address: "));
	printAddress(outsideThermometer);
	Serial.println();
	*/
	/*
	// set the resolution to 9 bit
	sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
	sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

	/*
	Serial.print("Device 0 Resolution: "));
	Serial.print(sensors.getResolution(insideThermometer), DEC);
	Serial.println();

	Serial.print("Device 1 Resolution: "));
	Serial.print(sensors.getResolution(outsideThermometer), DEC);
	Serial.println();
	*/
	
	// Setup Freetronics 8-Channel Relay Driver shield.
	Wire.begin(); // Wake up I2C bus

	// Set I/O bank A to outputs
	Wire.beginTransmission(I2C_ADDR);
	Wire.write(0x00); // IODIRA register
	Wire.write(0x00); // Set all of bank A to outputs
	Wire.endTransmission();
	
	
	// Configure and start web server

	/* setup our default command that will be run when the user accesses
	* the root page on the server */
	webserver.setDefaultCommand(&indexCmd);

	/* setup our default command that will be run when the user accesses
	* a page NOT on the server */
	webserver.setFailureCommand(&my_failCmd);

	/* run the same command if you try to load /index.html, a common
	* default page name */
	
	webserver.addCommand("index.html", &indexCmd);
	
	// These commands are from the Monitor page.
	webserver.addCommand("monitor.htm", &indexCmd);	
	webserver.addCommand("ajax_monitor", &monitorXmlCmd);

	// These commands are from the Settings page.
	webserver.addCommand("settings.htm", &settingsCmd);
	webserver.addCommand("table.xml", &tableCmd); 	

	// These commands are from the Control Schedule page.
	webserver.addCommand("control.htm", &scheduleCmd);
	webserver.addCommand("ajax_schedule", &scheduleXmlCmd); 
	webserver.addCommand("control.xml", &controlXmlCmd);
	webserver.addCommand("schedule_Button", &scheduleButtonCmd);
	webserver.addCommand("schedule_PressButton", &schedulePressButtonCmd);
	
	// These commands are from the Setup page.
	webserver.addCommand("setup.htm", &setupCmd);	// Setup page
	webserver.addCommand("setup.xml", &setupXmlCmd);	// Table update 
	webserver.addCommand("forminputclicked.xml", &formInputClickedXmlCmd);	// User has clicked on Form input field
	webserver.addCommand("forminputset.xml", &formInputSetXmlCmd);	// User has clicked on Form Set
	webserver.addCommand("setup_PressButton", &setupPressButtonCmd);	// User has clicked on a dehumidity button.
	
	// These commands come from a page load.	
	webserver.addCommand("wlt.css", &wltcssCmd);
	webserver.addCommand("rk.js", &rkjsCmd);
	webserver.addCommand("bground.jpg", &bgroundjpgCmd);
	
	// Access logs
	webserver.addCommand("logs", &logCmd);

	/* start the webserver */
	webserver.begin();



	// Setup and run timed tasks

	// -- Register the tasks to the timer manager. Tasks will start immediately.

	SoftTimer.add(&taskTempHumidity);
	SoftTimer.add(&taskTemperatureControl);
	SoftTimer.add(&taskDisplayLCD);
	SoftTimer.add(&taskWebServer);
	SoftTimer.add(&taskSensorLogger);
	SoftTimer.add(&taskScheduler);

	}
	
/**
 * Poll temp & humidity sensors
 */
void tempHumidity(Task* me) {

	printDateTime();
	Serial.println(F("tempHumidity()"));
	
	pollTempHumidity();	// Read temp & humidity and store the results into zone arrays.

	/*
	
	// Send command for all devices on the bus to perform a temperature conversion.
	// This can take up to 
	pollTemperatures(); 

	// Start the delayed read of the temperature conversions delayed task.
	getTempTask.startDelayed();
	
	*/
}

/*
// Delayed get DS18B20 temperatures task.
boolean getDsTemperatures(Task* task) {

	getTemp(insideThermometer, 4);	// Get the temp and store it
	getTemp(outsideThermometer, 5);

	sensorsOnline = true;	// Set sensors on line flag. Readings are valid.

	return false;	// Return false to disable the "followedBy" task.
}

/* Delayed start of Foyer fan task.
	Will only start medium or high fan if Fan status is in start state after timeount.
	NB Dummy code - this zone does not have evaporative cooling
*/
boolean runFoyerFanSpeed(Task* task) {

	if (zoneFanState[zoneFoyer] == fanMediumCoolStart) {
		
		fanControl(zoneFoyer, fanMediumCool);
		printDateTime();
		Serial.println(F("Delayed start: Foyer Medium cool fan"));
	}
	else if (zoneFanState[zoneFoyer] == fanHighCoolStart) {
		
		fanControl(zoneFoyer, fanHighCool);
		printDateTime();
		Serial.println(F("Delayed start: Foyer High fan"));
	}
}

/* Delayed start of Theatre fan task.
	Will only start medium or high fan if Fan status is in start state after timeount.
*/
boolean runTheatreFanSpeed(Task* task) {

	if (zoneFanState[zoneTheatre] == fanMediumCoolStart) {
		
		fanControl(zoneTheatre, fanMediumCool);
		printDateTime();
		Serial.println(F("Delayed start: Medium cool fan"));
	}
	else if (zoneFanState[zoneTheatre] == fanHighCoolStart) {
		
		fanControl(zoneTheatre, fanHighCool);
		printDateTime();
		Serial.println(F("Delayed start: High cool fan"));
	}
}

/* Purpose: Delayed stop of Theatre fan task.
	Delayed stop of gas heater fan
*/
boolean delayedFoyerFanStop(Task* task) {

	printDateTime();
	Serial.println(F("Delayed Stop Foyer fan"));
	fanControl(zoneFoyer, fanOff);	//	Stop fan
	
}
/* Purpose: Delayed stop of Theatre fan task.
	Delayed stop of gas heater fan
*/
boolean delayedTheatreFanStop(Task* task) {

	printDateTime();
	Serial.println(F("Delayed Stop Theatre fan"));
	fanControl(zoneTheatre, fanOff);	//	Stop fan	
}


/* Purpose: Runs every flameout check timeout when heating is on. 
	Detects flameount by comparing duct temperature with Duct Heat setting.
	If flameout is detected, shuts down heating and stops fan for 5 minutes and restarts.
*/

boolean flameoutCheck(Task* task) {

	if (isnan(zoneTemperature[zoneDuct])) {
		
		printDateTime();
		Serial.println(F("Warning - Flameout detector Duct sensor failed."));
		
	} else if (zoneTemperature[zoneDuct] < zoneThermostatSetting[zoneDuct]
		&& zoneHeatState[zoneTheatre] != heatOff)	{
		
		printDateTime();
		Serial.println(F("Warning - Theatre heating flameout detected"));
		zoneFlameouts[zoneTheatre]++;	// Increment flameout count
		zoneState[zoneTheatre] = zoneStateFlameout;	// Set Theatre zone status to Flameout.
		heatControl(zoneTheatre, heatFlameout);	// Shutdown zone heater and fan with no delay
		theatrePowerControl(powerOff);	// Shutdown zone gas heater to reset flameout.
		theatreFlameoutTimerTask.startDelayed();	// Start flameout reset timer


	}		
	else if (zoneHeatState[zoneTheatre] != heatOff)	{
		// Continue running flameout check if still heating
		delayedFlameoutTask.startDelayed();	// Restart flameout timeout
		printDateTime();
		Serial.println(F("Theatre heating flameout check O.K."));

	}
}
	
boolean theatreFlameoutTimer(Task* task)	{

	printDateTime();
	Serial.println(F("Warning - Theatre heating restarted"));
	zoneState[zoneTheatre] = zoneRestartState[zoneTheatre];	// Start Theatre zone in last run mode - manual or scheduled. 
	theatrePowerControl(powerOn);	// Restore power to Theatre gas heater
	
}

/*
	Purpose: Autosave Weekly Schedule.
			 Runs as a delayed start task to save any changes in Setup to EEPROM
	Accepts: scans sch<Day>Changed for day changes

*/
boolean runSchedulerAutoSave(Task* task) {

	/*
	http://thijs.elenbaas.net/2012/07/extended-eeprom-library-for-arduino
	
	The update functions are different from the write functions, in that
	they will check per byte if the current value differs and only update
	the the cell with a different value. This will not only reduce wear,
	and can also significantly reduce write time.
	*/
	int writtenBytes = 0;	// Count of written bytes.
	
	printDateTime();
	Serial.println(F("Autosave Weekly Schedule"));
	
	// If any changes to each days schedule, update in EEPROM.
	
	if (schSundayChanged) {
	
		printByteArray(schSunday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchSunday, schSunday, scheduleHours);
		schSundayChanged = false;
		printWrittenBytes("Sunday", writtenBytes);
		}
	if (schMondayChanged) {
		printByteArray(schMonday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchMonday, schMonday, scheduleHours);
		schMondayChanged = false;
		printWrittenBytes("Monday", writtenBytes);
		}
	if (schTuesdayChanged) 	{
		printByteArray(schTuesday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchTuesday, schTuesday, scheduleHours);
		schTuesdayChanged = false;
		printWrittenBytes("Tuesday", writtenBytes);
		}
	if (schWednesdayChanged) 	{
		printByteArray(schWednesday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchWednesday, schWednesday, scheduleHours);
		schWednesdayChanged = false;
		printWrittenBytes("Wednesday", writtenBytes);
		}
	if (schThursdayChanged) 	{
		printByteArray(schThursday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchThursday, schThursday, scheduleHours);
		schThursdayChanged = false;
		printWrittenBytes("Thursday", writtenBytes);
		}
	if (schFridayChanged) 	{
		printByteArray(schFriday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchFriday, schFriday, scheduleHours);
		schFridayChanged = false;
		printWrittenBytes("Friday", writtenBytes);
		}
	if (schSaturdayChanged) {
		printByteArray(schSaturday, 24);
		writtenBytes = EEPROM.updateBlock<byte>(addressSchSaturday, schSaturday, scheduleHours);
		schSaturdayChanged = false;
		printWrittenBytes("Saturday", writtenBytes);
		}
	if (weeklyScheduleArrayChanged) {
		printByteArray(weeklyScheduleArray, 2);
		writtenBytes = EEPROM.updateBlock<byte>(addressWeeklyScheduleArray, weeklyScheduleArray, numSchedulePressButtons);
		weeklyScheduleArrayChanged = false;
		printWrittenBytes("Schedule", writtenBytes);
		}
}

/**
	Autosave Setup
	Purpose: Runs as a delayed start task to save any changes in Setup to EEPROM
	Accepts: Scans Setup xxxxChanged flags
*/
boolean runSetupAutoSave(Task* task) {

	/*
	http://thijs.elenbaas.net/2012/07/extended-eeprom-library-for-arduino
	
	The update functions are different from the write functions, in that
	they will check per byte if the current value differs and only update
	the the cell with a different value. This will not only reduce wear,
	and can also significantly reduce write time.
	*/
	int writtenBytes = 0;	// Count of written bytes.
	printDateTime();
	Serial.println(F("Autosave Setup"));
	
	// If any changes to setup, update in EEPROM.

	if (zoneVentilateSettingChanged) {
		writtenBytes = EEPROM.updateBlock<byte>(addressZoneVentilateArray, zoneVentilateArray, g_numberOfZones);
		printWrittenBytes("VentilateSetting", writtenBytes);
		zoneVentilateSettingChanged = false;
		}	
	if (zoneThermostatSettingChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneThermostatSetting, zoneThermostatSetting, g_numberOfZones);
		printWrittenBytes("ThermostatSetting", writtenBytes);
		zoneThermostatSettingChanged = false;
		}
	if (zoneThermostatHysteresisChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneThermostatHysteresis, zoneThermostatHysteresis, g_numberOfZones);
		printWrittenBytes("ThermostatHysteresis", writtenBytes);
		zoneThermostatHysteresisChanged = false;
		}
	if (zoneCoolThermostatSettingChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneCoolThermostatSetting, zoneCoolThermostatSetting, g_numberOfZones);
		printWrittenBytes("CoolThermostatSetting", writtenBytes);
		zoneCoolThermostatSettingChanged = false;
		}
	if (zoneCoolThermostatHysteresisChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneCoolThermostatHysteresis, zoneCoolThermostatHysteresis, g_numberOfZones);
		printWrittenBytes("CoolThermostatHysteresis", writtenBytes);
		zoneCoolThermostatHysteresisChanged = false;
		}		
	if (zoneCoolHighFanPointChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneCoolHighFanPoint, zoneCoolHighFanPoint, g_numberOfZones);
		printWrittenBytes("CoolHighFanPoint", writtenBytes);
		zoneCoolHighFanPointChanged = false;
		}
	if (zoneCoolHighFanHysteresisChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneCoolHighFanHysteresis, zoneCoolHighFanHysteresis, g_numberOfZones);
		printWrittenBytes("CoolHighFanHysteresis", writtenBytes);
		zoneCoolHighFanHysteresisChanged = false;
		}		
	if (zoneMaxHumidityChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneMaxHumidity, zoneMaxHumidity, g_numberOfZones);
		printWrittenBytes("MaxHumidity", writtenBytes);
		zoneMaxHumidityChanged = false;
		}
	if (zoneMaxHumidityHysteresisChanged) {
		writtenBytes = EEPROM.updateBlock<float>(addressZoneMaxHumidityHysteresis, zoneMaxHumidityHysteresis, g_numberOfZones);
		printWrittenBytes("MaxHumidityHysteresis", writtenBytes);
		zoneMaxHumidityHysteresisChanged = false;
		}		
}

/*	
	Purpose: Prints number of bytes written to EEPROM
	Accepts: Text name of data, number of bytes written.
*/
void printWrittenBytes(const char* name, int writtenBytes) {

	Serial.print(name); Serial.print(F(" = ")); Serial.println(writtenBytes);
}

/*
	Purpose: Prints contents of byte array in HEX format
	Accepts: pointer to byte array, array size
*/
void printByteArray(byte *byteArray, int arraySize) {

	Serial.print(F("Array size = "));
	Serial.println(arraySize);
	
	for (int index=0; index < arraySize; index++) {
//		byte arrayByte = byteArray[index];
		Serial.print(index);
		Serial.print(F("\t"));	// Print a tab
		Serial.print(byteArray[index], HEX);
		Serial.println();
	}
}


/*************************** Main Heating, Ventilating and Cooling Task *********************************

*/


/**
	Purpose: Runs heating, cooling and ventilation controls in each zone
 
 */
void controlHVAC(Task* me) {

	if (!sensorsOnline) return;	// Skip until first sensor scan.
	
	printDateTime();
	Serial.println(F("controlHVAC()"));	
	
	// Check zone sensors and update fault counters
	
	if (!checkSensors(zoneTheatre)) zoneSensorFaults[zoneTheatre]++;	//Increment zone sensor fault count
	if (!checkSensors(zoneDuct)) zoneSensorFaults[zoneDuct]++;
	if (!checkSensors(zoneFoyer)) zoneSensorFaults[zoneFoyer]++;
	if (!checkSensors(zoneOutside)) zoneSensorFaults[zoneOutside]++;
	if (!checkSensors(zoneDressingRoom)) zoneSensorFaults[zoneDressingRoom]++;
	if (!checkSensors(zoneStage)) zoneSensorFaults[zoneStage]++;

	//	Check zone humidity and set/clear maxium humidity flag, no fan control
	checkZoneHumidity(zoneOutside, noMaxHumidityControlFan);
	checkZoneHumidity(zoneDressingRoom, noMaxHumidityControlFan);
	checkZoneHumidity(zoneStage, noMaxHumidityControlFan);
	
	// Status display
	printTempHum(zoneTheatre);
	printTempHum(zoneFoyer);
	printTempHum(zoneOutside);	
	
	// Control execution is controlled by zone state
	
	controlZoneHeating(zoneTheatre);	// Theatre zone heat control

	controlZoneHeating(zoneFoyer);	// Foyer zone heat control

	controlZoneCooling(zoneTheatre);	// Control Theatre Cooling

	controlZoneCooling(zoneFoyer);	// Control Foyer Cooling

	controlZoneVentilation(zoneTheatre);	// Ventilate Theatre

	controlZoneVentilation(zoneFoyer);	// Ventilate Foyer

}

/*	controlZoneHeating

	Purpose: If zone is on, start heating, fan and flameout watchdog timer if temperature is below set point.
		If temperature is below low heat band, use high fan, otherwise use medium fan.
		If in heating mode, stop heating, fan and flameout watchdog timer if temperature has dropped below set point. 
		Hysteries is applied to on and off set points.
	Accepts:	zoneIndex = zone number (base 0).
*/
void controlZoneHeating(int zoneIndex) {

	//Check if zone is "On".
	if (!(zoneState[zoneIndex] == zoneStateScheduleOn || zoneState[zoneIndex] == zoneStateManualOn)) {
	
		if (zoneHeatState[zoneIndex] != heatOff)	{
			heatControl(zoneIndex, heatOff);	// Turn zone heating off. Delayed fan stop if required
		}
		if (zoneIndex == zoneTheatre)	{
			Alarm.disable(alarmIdFlameout);	// Stop flameout watchdog timer.
		}
		return;
	}
	// Control zone heating
	
	printDateTime();
	Serial.print(F("controlZoneHeating("));
	Serial.print(zoneName[zoneIndex]);
	Serial.println(F(")"));

	// Stop heating if zone temperature is above set point + hysteresis
	if (!isnan(zoneTemperature[zoneIndex])
		&& zoneHeatState[zoneIndex] != heatOff
		&& zoneTemperature[zoneIndex] > (zoneThermostatSetting[zoneIndex] + zoneThermostatHysteresis[zoneIndex]) ) {
			
		heatControl(zoneIndex, heatOff);	// Turn zone heating off. Delayed fan stop if required

	}
	// Check if zone temperature is below low heat band and not already high heat
	else if (!isnan(zoneTemperature[zoneIndex])
		&& zoneTemperature[zoneIndex] < (zoneThermostatSetting[zoneIndex] - zoneThermostatHysteresis[zoneIndex] - LOW_HEAT_BAND)
		&& zoneHeatState[zoneIndex] != heatHigh) {
			
		heatControl(zoneIndex, heatHigh);	// Start high heat and fan

		if (zoneIndex == zoneTheatre) {
			Alarm.enable(alarmIdFlameout);	// Start heater flameout watchdog timer.
			zoneRestartState[zoneTheatre] = zoneState[zoneTheatre];	// Save current mode for restart.
		}
	}
			
	// Check if zone temperature is in low heat band
	else if (!isnan(zoneTemperature[zoneIndex])
		&& zoneTemperature[zoneIndex] < (zoneThermostatSetting[zoneIndex] - zoneThermostatHysteresis[zoneIndex])
		&& (!(zoneHeatState[zoneIndex] == heatLow || zoneHeatState[zoneIndex] == heatHigh))) {
		
		heatControl(zoneIndex, heatLow);	// Start low heat and fan
		if (zoneIndex == zoneTheatre) {
			Alarm.enable(alarmIdFlameout);	// Start heater flameout watchdog timer.
			zoneRestartState[zoneTheatre] = zoneState[zoneTheatre];	// Save current mode for restart.
		}
	}				
}

/*
	Purpose: Maps controlZoneCooling to zone cooling handler.
	Accepts: zoneIndex
	
*/

void controlZoneCooling(byte zoneIndex)	{

	switch (zoneIndex)	{
		case zoneTheatre:
			controlTheatreCooling();
			break;
			
		case zoneFoyer:
			controlFoyerCooling();
			break;
			
		default:
			printDateTime();
			Serial.println(F("Unsupported cooling control request"));
		}
}

/*
	Theatre Cooling Control
	Purpose: Runs Theatre cooling if zone is on.
*/

void controlTheatreCooling()	{

	//Check if zone is "On".
	if (!(zoneState[zoneTheatre] == zoneStateScheduleOn || zoneState[zoneTheatre] == zoneStateManualOn)) {
		return;
	}
	printDateTime();
	Serial.println(F("controlTheatreCooling()"));
	
	sensorsOK = true;	// Prime
	if (isnan(zoneTemperature[zoneOutside])) {
		
		printDateTime();
		Serial.println(F("zoneOutside sensor Read Fail"));
		sensorsOK = false;
	}
	if (isnan(zoneTemperature[zoneTheatre])) {
		
		printDateTime();
		Serial.println(F("zoneTheatre sensor Read Fail"));
		sensorsOK = false;
	}
	if (!sensorsOK)	{
		return;
	}
	// Check if zone is not heating
	if (!(zoneHeatState[zoneTheatre] == heatOff))	{
		
		checkZoneHumidity(zoneTheatre, noMaxHumidityControlFan);	//	Check zone humidity - no fan control
		return;
	}	

	checkZoneHumidity(zoneTheatre, maxHumidityControlFan);	//	Check zone humidity and control fan/cool fan state
	
	// Check if not heating or not fan delayed start
	if (!(zoneFanState[zoneTheatre] == fanHighCoolStart) || (zoneFanState[zoneTheatre] == fanMediumCoolStart)) {

		if (zoneTemperature[zoneTheatre] > zoneCoolHighFanPoint[zoneTheatre]
			&& !(zoneFanState[zoneTheatre] == fanHighRun || zoneFanState[zoneTheatre] == fanHighCoolStart)) {
			
			// Check if outside air temp is cool enough to reduce inside temp without increasing humidity.
			if (!isnan(zoneTemperature[zoneOutside])
				&& zoneTemperature[zoneOutside] < zoneThermostatSetting[zoneOutside]) {
				theatreFanControl(fanHighRun);	//Start High Fan - Low outside temperature

			} else {

				// Check zone humidity
				if (!zoneHighHumidity[zoneTheatre]) {
						
					theatreFanControl(fanHighCoolStart);	//	Start cool pump and delay start fan unless cool pump is already running.
															//	If cool pump was already running, run high cool fan.

				} else {
					theatreFanControl(fanHighRun);	//	Start High Fan - High Humidity
				}
			}

		} else if (zoneTemperature[zoneTheatre] < zoneCoolHighFanPoint[zoneTheatre] 
			&& zoneFanState[zoneTheatre] == fanHighRun) {
			
			// If temperature has dropped below high fan set point and high fan is operating, run fan medium speed.			
			theatreFanControl(fanMediumRun);	//	Drop to medium fan speed

		} else if (zoneTemperature[zoneTheatre] < zoneCoolHighFanPoint[zoneTheatre] 
			&& zoneFanState[zoneTheatre] == fanHighCool) {
			// If temperature has dropped below high fan set point and high cool fan is operating, run fan medium cool speed.			

			theatreFanControl(fanMediumCool);	//	Drop to medium fan speed

			
		/* If temperature is above cool set point and fan is not running, start medium fan. 
			Start cool pump if outside temp is above outside cool setpoint and inside humidity is below max setpoint
		*/
		} else if (zoneTemperature[zoneTheatre] > zoneCoolThermostatSetting[zoneTheatre] 
				&& !(zoneFanState[zoneTheatre] == fanMediumRun || zoneFanState[zoneTheatre] == fanHighRun
					|| zoneFanState[zoneTheatre] == fanMediumCoolStart || zoneFanState[zoneTheatre] == fanHighCoolStart)) {
		
			// Check if outside air temp is cool enough to reduce inside temp without increasing humidity.
			if (!isnan(zoneTemperature[zoneOutside])
				&& zoneTemperature[zoneOutside] < zoneThermostatSetting[zoneOutside]) {
				
				theatreFanControl(fanMediumRun);	//	Run medium fan speed
	
			} else {
				
				// Check current zone humidity
				if (!zoneHighHumidity[zoneTheatre]) {
				
					theatreFanControl(fanMediumCoolStart);	//	Delayed start medium cool fan
				} else {

					theatreFanControl(fanMediumRun);	//	Run medium fan
				}	
			} 
		}
		// If temperature is below cool setpoint - hysterisis and fan is running, stop fan and cool pump.
		else if (zoneTemperature[zoneTheatre] < (zoneCoolThermostatSetting[zoneTheatre] - zoneCoolThermostatHysteresis[zoneTheatre])
				&& (zoneFanState[zoneTheatre] == fanHighCool || zoneFanState[zoneTheatre] == fanHighRun
					|| zoneFanState[zoneTheatre] == fanMediumCool || zoneFanState[zoneTheatre] == fanMediumRun
					|| zoneFanState[zoneTheatre] == fanMediumCoolStart || zoneFanState[zoneTheatre] == fanHighCoolStart)) {
					
			theatreFanControl(fanOff);	//	Stop fan and cool pump

		}
		// Check if outside temperature has dropped below threshold and stop Cool Pump.
		else if (!isnan(zoneTemperature[zoneOutside])
				&& zoneTemperature[zoneOutside] < (zoneCoolThermostatSetting[zoneOutside] - zoneCoolThermostatHysteresis[zoneOutside])
				&& (zoneFanState[zoneTheatre] == fanMediumCool || zoneFanState[zoneTheatre] == fanHighCool
					|| zoneFanState[zoneTheatre] == fanMediumCoolStart || zoneFanState[zoneTheatre] == fanHighCoolStart)) {
					
			if (zoneFanState[zoneTheatre] == fanMediumCool) {
				theatreFanControl(fanMediumRun);	//	Run medium fan speed
			}
			else if (zoneFanState[zoneTheatre] == fanHighCool) {
				theatreFanControl(fanHighRun);	//	Run high fan speed
			}
		}
		// If fan is opeating without cool pump and not zone High Humidity and outside temperature has risen above set point, start cool pump

		else if (!isnan(zoneTemperature[zoneOutside])
				&& zoneTemperature[zoneOutside] > zoneCoolThermostatSetting[zoneOutside]
				&& !zoneHighHumidity[zoneTheatre]
				&& (zoneFanState[zoneTheatre] == fanHighRun || zoneFanState[zoneTheatre] == fanMediumRun)) {
/*	
			printDateTime();
			Serial.print(F("Outside temp over "));Serial.print(zoneCoolThermostatSetting[zoneOutside],1);
			Serial.println(F(" and not high humidity - start cool fan"));
*/				
			// Run Cool Pump and delayed fan start.
			if (zoneFanState[zoneTheatre] == fanMediumRun) {
				theatreFanControl(fanMediumCoolStart);	//	Run medium cool fan
			}
			else if (zoneFanState[zoneTheatre] == fanHighRun) {
				theatreFanControl(fanHighCoolStart);	//	Run high cool fan
			}
		}
	}
}

/*
	Foyer Cooling Control
	Purpose: Runs Foyer cooling if zone is on.
*/

void controlFoyerCooling()	{

	// Foyer Cooling
	 
	 //Check if Foyer zone is "On".
	if (!(zoneState[zoneFoyer] == zoneStateScheduleOn || zoneState[zoneFoyer] == zoneStateManualOn)) {
		return;
	}
	
	printDateTime();
	Serial.println(F("controlFoyerCooling()"));
	
	sensorsOK = true;	// Prime
	if (isnan(zoneTemperature[zoneOutside])) {
		
		printDateTime();
		Serial.println(F("zoneOutside sensor Read Fail"));
		sensorsOK = false;
	}
	if (isnan(zoneTemperature[zoneFoyer])) {
		
		printDateTime();
		Serial.println(F("zoneFoyer sensor Read Fail"));
		sensorsOK = false;
	}
	if (!sensorsOK)	{
		foyerFanControl(fanOff);	// Sensor fail shutdown
		return;
	}
	// Check if zone is not in heating mode
	if (!(zoneHeatState[zoneFoyer] == heatOff))	{
		
		checkZoneHumidity(zoneFoyer, noMaxHumidityControlFan);	//	Check zone humidity - no fan control
		return;
	}	

	checkZoneHumidity(zoneFoyer, maxHumidityControlFan);	//	Check zone humidity and control fan/cool fan state

	// If Foyer temperature is higher than outside and Foyer temperature is greater than cool set point
	// and fan is not operating, start fan
	if (zoneTemperature[zoneFoyer] > zoneTemperature[zoneOutside]
		&& zoneTemperature[zoneFoyer] > zoneCoolThermostatSetting[zoneFoyer]
		&& zoneFanState[zoneFoyer] == fanOff) {
	
		foyerFanControl(fanHighRun);
	}
	// If temperature is below cool setpoint - hysterisis, and fan is running,  stop fan.
	// Exception is fanDelayedStop
	else if (zoneTemperature[zoneFoyer] < (zoneCoolThermostatSetting[zoneFoyer] - zoneCoolThermostatHysteresis[zoneFoyer])
			&& (zoneFanState[zoneFoyer] == fanHighCool || zoneFanState[zoneFoyer] == fanHighRun
					|| zoneFanState[zoneFoyer] == fanMediumCool || zoneFanState[zoneFoyer] == fanMediumRun
					|| zoneFanState[zoneFoyer] == fanMediumCoolStart || zoneFanState[zoneFoyer] == fanHighCoolStart)) {
					
		foyerFanControl(fanOff);
	}
	// If foyer temperature is lower than outside temperature, heat is off and fan is running, stop fan
	// Exception is fanDelayedStop
	else if (zoneTemperature[zoneFoyer] < (zoneTemperature[zoneOutside] - zoneCoolThermostatHysteresis[zoneFoyer])
			&& (zoneFanState[zoneFoyer] == fanHighCool || zoneFanState[zoneFoyer] == fanHighRun
					|| zoneFanState[zoneFoyer] == fanMediumCool || zoneFanState[zoneFoyer] == fanMediumRun
					|| zoneFanState[zoneFoyer] == fanMediumCoolStart || zoneFanState[zoneFoyer] == fanHighCoolStart)) {
			
		foyerFanControl(fanOff);
	}
} 


/*
	Purpose: Run ventilation checks in zone. If outside temperature plus differential is below inside zone temperature, run ventilation fan.
				Stop ventilation if inside temperature is below ventilation set point. 
				Stop ventilation fan if outside temperature rises above inside temperature.
	Accepts:	zoneIndex = zone number
*/
void controlZoneVentilation(int zoneIndex)	{

	/* Uses settings from Outside zone to control ventilation. These settings apply to all ventilated zones.
	
	zoneCoolThermostatSetting[zoneOutside] = Ventilation temperature set point (minium).
	zoneCoolThermostatHysteresis[zoneIndex] = Hysteresis applied to Ventilation set point.
	zoneCoolHighFanHysteresis[zoneOutside] = Difference required between Outside and Inside zone temperature to run ventilation fan.
	*/

	if (!(zoneState[zoneIndex] == zoneStateVentilateFanRunning || zoneState[zoneIndex] == zoneStateVentilateCheck ))	{
		return;	// Exit if zone not in ventilate state
	}
	printDateTime();
	Serial.print(F("controlZoneVentilation("));
	Serial.print(zoneName[zoneIndex]);
	Serial.println(F(")"));
	
	
	/* Check if temperature in zone is above ventilation cool set point and there is a negative differental between inside and outside temperature
		Differental is set by zoneCoolHighFanHysteresis[zoneOutside]
	*/
	if (zoneTemperature[zoneIndex] > (zoneCoolThermostatSetting[zoneOutside] + zoneCoolThermostatHysteresis[zoneIndex])
		&& (zoneTemperature[zoneOutside] + zoneCoolHighFanHysteresis[zoneOutside]) < zoneTemperature[zoneIndex]
		&& zoneState[zoneIndex] == zoneStateVentilateCheck) {

		// Start ventilation fan
		fanControl(zoneIndex, fanHighRun);	//	Run high fan speed
		zoneState[zoneIndex] = zoneStateVentilateFanRunning;
		printDateTime();
		Serial.print(F("Start Ventilation Fan\tOutside Temp = "));
		Serial.println(zoneTemperature[zoneOutside]);

	}

	// Stop ventilation fan if zone temperature is below set point. 
	else if (zoneState[zoneIndex] == zoneStateVentilateFanRunning
		&& zoneTemperature[zoneIndex] < zoneCoolThermostatSetting[zoneOutside])	{
		
		// Stop ventilation fan
		fanControl(zoneIndex, fanOff);
		printDateTime();
		Serial.println(F("Stop Ventilation Fan"));
		zoneState[zoneIndex] = zoneStateVentilateCheck;		// Resume ventilation checking
	}
	// Check if Outside temp/Inside temp differential less than zoneCoolHighFanHysteresis[zoneOutside]
	else if (zoneState[zoneIndex] == zoneStateVentilateFanRunning
		&& (zoneTemperature[zoneOutside] + zoneCoolHighFanHysteresis[zoneOutside]) > zoneTemperature[zoneIndex]) {
		// Outside temp/Inside temp differential less than zoneCoolHighFanHysteresis[zoneOutside] - stop ventilation fan
		fanControl(zoneIndex, fanOff);
		printDateTime();
		Serial.println(F("Inside/Outside temperature differential below threshold - stop Ventilation Fan"));
		zoneState[zoneIndex] = zoneStateVentilateCheck;	// Resume ventilation checking
	}
}


/*
	Purpose:	Check Zone humidity and maintain zoneHighHumidity flag

			If zoneHighHumidity is > zoneMaxHumidity, sets zoneHighHumidity flag and if coolFanControl true, changes any cooling fan status to fan only.
			If zoneHumidity drops below zoneMaxHumidity - Hysteresis, clear zoneHighHumidity flag and if coolFanControl = true, change any run fan mode to cool fan mode.
			
	Accepts: zoneIndex = zone number.
			 coolFanControl true = control fans on humidity change
			 
	Returns:	zoneHighHumidity[zoneIndex] = true if high humidity

*/

bool checkZoneHumidity(int zoneIndex, bool coolFanControl)	{

	if (zoneHumidity[zoneIndex] > zoneMaxHumidity[zoneIndex]
			&& !zoneHighHumidity[zoneIndex] )	{
			
		zoneHighHumidity[zoneIndex] = true;	// Transition to high humidity detected
	
		// If cool fan control and cool fan is running, change to fan-only mode
		
		if (coolFanControl && (zoneFanState[zoneIndex] == fanLowCool || zoneFanState[zoneIndex] == fanMediumCool
			|| zoneFanState[zoneIndex] == fanHighCool || zoneFanState[zoneIndex] == fanLowCoolStart
			|| zoneFanState[zoneIndex] == fanMediumCoolStart || zoneFanState[zoneIndex] == fanHighCoolStart))	{

			printDateTime();
			Serial.println(F("Humidity control: Run fan only"));

			if (zoneFanState[zoneIndex] == fanLowCool) {
				fanControl(zoneIndex, fanLowRun);	//	Run low fan speed
			}
			else if (zoneFanState[zoneIndex] == fanMediumCool) {
				fanControl(zoneIndex, fanMediumRun);	//	Run medium fan speed
			}
			else if (zoneFanState[zoneIndex] == fanHighCool) {
				fanControl(zoneIndex, fanHighRun);	//	Run high fan speed
			}
			return true;
		}
	}

	//	If humidity has dropped below zoneMaxHumidity - hysterisis and fan is operating, change to cool fan mode
	
	else if (zoneHumidity[zoneIndex] < (zoneMaxHumidity[zoneIndex] - zoneMaxHumidityHysteresis[zoneIndex])
			&& zoneHighHumidity[zoneIndex] == true)	{
			
		zoneHighHumidity[zoneIndex] = false;	
	
		if (coolFanControl && (zoneFanState[zoneIndex] == fanLowRun || zoneFanState[zoneIndex] == fanMediumRun
			|| zoneFanState[zoneIndex] == fanHighRun))	{ 
				
			printDateTime();
			Serial.println(F("Humidity control: Resume cool fan"));
			
			// Run Cool Pump and fan - no delayed fan start.
			if (zoneFanState[zoneIndex] == fanLowRun) {
				fanControl(zoneIndex, fanLowCool);	//	Run low cool fan
			}
			else if (zoneFanState[zoneIndex] == fanMediumRun) {
				fanControl(zoneIndex, fanMediumCool);	//	Run medium cool fan
			}
			else if (zoneFanState[zoneIndex] == fanHighRun) {
				fanControl(zoneIndex, fanHighCool);	//	Run high cool fan
			}
		}
	}
	return zoneHighHumidity[zoneIndex];
}

/*
	Purpose: Checks zone temperature for nan (Not a Number) to check if sensor is OK.
	Accepts: zoneIndex
	Returns: true if valid temperature reading, false if nan.
*/

bool checkSensors(int zoneIndex)	{

	if (!isnan(zoneTemperature[zoneIndex]))	{
		return true;
	}
	else	{
		return false;
	}
}





/*	************************ Zone Device Controls **********************************************

*/

/*	Fan modes
	const byte fanOff = 0;
	const byte fanDelayedStop = 1;
	const byte fanLowCoolStart = 2;
	const byte fanLowCool = 3;
	const byte fanLowRun = 4;
	const byte fanMediumCoolStart = 5;
	const byte fanMediumCool = 6;
	const byte fanMediumRun = 7;
	const byte fanHighCoolStart = 8;
	const byte fanHighCool = 9;
	const byte fanHighRun = 10;
*/

/*
	Purpose: Map fanState request to zone-based device handler
	Accepts: zoneIndex = zone number
			 fanState = Required new fan state
*/

void fanControl(int zoneIndex, byte fanState)	{

	switch (zoneIndex)	{
		case zoneTheatre:
			theatreFanControl(fanState);
			break;
			
		case zoneFoyer:
			foyerFanControl(fanState);
			break;
			
		default:
			printDateTime();
			Serial.println(F("Unsupported fan control request"));
		}
}

/*
	Purpose: Theatre Fan control driver. Maps logical fan controls to hardware.
	Accepts: fanState = new fan state
*/
void theatreFanControl(byte fanState) {

	switch (fanState)	{
		case fanOff:
			relayCommand(relayBankA, relayTheatreRunFan, relayOff);	// Stop Fan.
			relayCommand(relayBankA, relayFanSpeedController, relayOff);	// Stop Fan speed controller.
			theatreCoolPumpControl(coolPumpOff);	// Stop Cool water pump.
			SoftTimer.remove(&runTheatreFanTask);	// Stop any delayed fan start.
			zoneFanState[zoneTheatre] = fanOff;
			printDateTime();
			Serial.println(F("Stop Theatre Fan"));
			break;
		
		case fanDelayedStop:
			// Delayed fan shutdown to cool gas heater
			delayedTheatreFanTask.startDelayed();	// Fan stop delay	
			zoneFanState[zoneTheatre] = fanDelayedStop;
			printDateTime();
			Serial.println(F("Delayed Stop Theatre Fan"));
			break;
		
		case fanLowCoolStart:
		case fanLowCool:
		case fanLowRun:
			printDateTime();
			Serial.println(F("Unsupported Theatre fan speed"));
		break;
		
		case fanMediumCoolStart:
			// Check if cool pump is already running
			if (zoneCoolPumpState[zoneTheatre] == coolPumpOn) {
				relayCommand(relayBankA, relayFanSpeedController, relayOn);	// Select fan speed controller.
				relayCommand(relayBankA, relayTheatreRunFan, relayOn);	// Run Fan.
				zoneFanState[zoneTheatre] = fanMediumCool;	// 
				printDateTime();
				Serial.println(F("Theatre Medium Cool Fan"));
			}
			else {
				theatreCoolPumpControl(coolPumpOn);	// Run Cool water pump.
				zoneFanState[zoneTheatre] = fanMediumCoolStart;	// Run Medium speed after delay
				runTheatreFanTask.startDelayed();	// Fan start delay
				printDateTime();
				Serial.println(F("Theatre Start Medium Cool Fan"));
			}
			break;
			
		case fanMediumCool:
			// 
			theatreCoolPumpControl(coolPumpOn);	// Run Cool water pump.
			relayCommand(relayBankA, relayFanSpeedController, relayOn);	// Select fan speed controller.
			relayCommand(relayBankA, relayTheatreRunFan, relayOn);	// Run Fan.
			zoneFanState[zoneTheatre] = fanMediumCool;
			printDateTime();
			Serial.println(F("Theatre Medium Cool Fan"));
			break;
			
		case fanMediumRun:

			relayCommand(relayBankA, relayFanSpeedController, relayOn);	// Select fan speed controller.
			relayCommand(relayBankA, relayTheatreRunFan, relayOn);	// Run Fan.
			theatreCoolPumpControl(coolPumpOff);	// Stop Cool water pump.
			zoneFanState[zoneTheatre] = fanMediumRun;
			printDateTime();
			Serial.println(F("Theatre Medium Fan"));
			break;
			
		case fanHighCoolStart:
			// Check if cool pump is already running
			if (zoneCoolPumpState[zoneTheatre] == coolPumpOn) {
				relayCommand(relayBankA, relayFanSpeedController, relayOff);	// Stop Fan speed controller.
				relayCommand(relayBankA, relayTheatreRunFan, relayOn);	// Run Fan.
				zoneFanState[zoneTheatre] = fanHighCool;
				printDateTime();
				Serial.println(F("Theatre High Cool Fan"));
			}
			else {
				theatreCoolPumpControl(coolPumpOn);	// Run Cool water pump.
				relayCommand(relayBankA, relayFanSpeedController, relayOff);	// Stop fan speed controller.
				zoneFanState[zoneTheatre] = fanHighCoolStart;	// Run High fan speed after delay
				runTheatreFanTask.startDelayed();	// Fan start delay
				printDateTime();
				Serial.println(F("Theatre Start High Cool Fan"));
			}
			break;
	
		case fanHighCool:
			
			theatreCoolPumpControl(coolPumpOn);	// Run Cool water pump.
			relayCommand(relayBankA, relayFanSpeedController, relayOff);	// Stop Fan speed controller.
			relayCommand(relayBankA, relayTheatreRunFan, relayOn);	// Run Fan.
			zoneFanState[zoneTheatre] = fanHighCool;
			printDateTime();
			Serial.println(F("Theatre High Cool"));
			break;
	
		case fanHighRun:
			relayCommand(relayBankA, relayFanSpeedController, relayOff);	// Stop Fan speed controller.
			theatreCoolPumpControl(coolPumpOff);	// Stop Cool water pump.
			relayCommand(relayBankA, relayTheatreRunFan, relayOn);	// Run Fan.
			zoneFanState[zoneTheatre] = fanHighRun;
			printDateTime();
			Serial.println(F("Theatre High Fan"));
			break;
		}
}

/*
	Purpose: Foyer Fan control driver. Maps logical fan controls to hardware.
	Accepts: fanState = required fan state
*/
void foyerFanControl(byte fanState) {

	switch (fanState) {
	
		case fanOff:
			relayCommand(relayBankA, relayFoyerRunFan, relayOff);	// Stop Fan.
			
			zoneFanState[zoneFoyer] = fanOff;
			printDateTime();
			Serial.println(F("Stop Foyer Fan"));
			break;
			
		case fanDelayedStop:
			// Delayed fan shutdown to cool gas heater
			delayedFoyerFanTask.startDelayed();	// Fan stop delay	
			zoneFanState[zoneFoyer] = fanDelayedStop;
			printDateTime();
			Serial.println(F("Delayed Stop Foyer Fan"));
			break;
		
		case fanLowCoolStart:
		case fanLowCool:
		case fanLowRun:
		case fanMediumCoolStart:
		case fanMediumCool:
		case fanMediumRun:
		case fanHighCoolStart:
		case fanHighCool:
		case fanHighRun:
			relayCommand(relayBankA, relayFoyerRunFan, relayOn);	// Start Fan.
			zoneFanState[zoneFoyer] = fanState;	//Set current fan status
			printDateTime();
			Serial.println(F("Start Foyer Fan"));
			break;
		
		default:
			relayCommand(relayBankA, relayFoyerRunFan, relayOff);	// Stop Fan.
			zoneFanState[zoneFoyer] = fanOff;
		}
}

void theatreCoolPumpControl(byte coolPumpStatus)	{

	if (coolPumpStatus == coolPumpOn)	{
		relayCommand(relayBankA, relayCoolPump, relayOn);	// Run Cool water pump.
		zoneCoolPumpState[zoneTheatre] = coolPumpOn;
		printDateTime();
		Serial.println(F("Theatre Cool Pump on"));
	}
	else if (coolPumpStatus == coolPumpOff)	{
		relayCommand(relayBankA, relayCoolPump, relayOff);	// Stop Cool water pump.
		zoneCoolPumpState[zoneTheatre] = coolPumpOff;
		printDateTime();
		Serial.println(F("Theatre Cool Pump off"));
	}
}


/*	Theatre Gas Heater Power Control
	Purpose: Control power to Theatre gas heater. Used to shutdown power during flameout reset.
			Power relay is wired using NC (Normally ON) contacts. Activating relay removes power from Theatre gas heater.
	Accepts: powerOn = Relay off
			powerOff = Relay on
*/

void theatrePowerControl(byte powerStatus)	{

	if (powerStatus == powerOn)	{
		relayCommand(relayBankA, relayTheatrePowerController, relayOff);	// A/C power on
		printDateTime();
		Serial.println(F("Theatre Gas Heater A/C power on"));
	}
	else if (powerStatus == powerOff)	{
		relayCommand(relayBankA, relayTheatrePowerController, relayOn);	// A/C power off
		printDateTime();
		Serial.println(F("Theatre Gas Heater A/C power off"));
	}
}



/*
	Purpose: Controls zone heat
	Accepts: Required  zone index, zone heat status
	
*/
	
void heatControl(int zoneIndex, byte heatStatus)	{

	switch (zoneIndex)	{
		case zoneTheatre:
			theatreHeatControl(heatStatus);
			break;
			
		case zoneFoyer:
			foyerHeatControl(heatStatus);
			break;
			
		default:
			printDateTime();
			Serial.println(F("Unsupported heat control request"));
		}
}

/*
	Purpose: Theatre Heat control driver. Maps logical heat controls to hardware.
	Accepts: heatStatus = required heat state
*/

void theatreHeatControl(byte heatStatus)	{

	switch (heatStatus)	{
		case heatOff:
			relayCommand(relayBankA, relayTheatreHeat, relayOff);	// Turn zone heating off
			SoftTimer.remove(&delayedFlameoutTask);	// Stop flameout timeout
			zoneHeatState[zoneTheatre] = heatOff;	// Set new heat status
			printDateTime();
			Serial.println(F("Stop Theatre Heat"));
			fanControl(zoneTheatre, fanDelayedStop);	// Delayed fan shutdown to cool gas heater
			break;
			
		case heatLow:
			relayCommand(relayBankA, relayTheatreHeat, relayOn);	// Turn zone heating on.
			delayedFlameoutTask.startDelayed();	// Start flameout timeout
			zoneHeatState[zoneTheatre] = heatLow;	// Set new heat status
			printDateTime();
			Serial.println(F("Start Theatre Low Heat"));
			theatreFanControl(fanMediumRun);	// Run medium fan
			break;
			
		case heatHigh:
			relayCommand(relayBankA, relayTheatreHeat, relayOn);	// Turn zone heating on.
			delayedFlameoutTask.startDelayed();	// Start flameout timeout
			zoneHeatState[zoneTheatre] = heatHigh;	// Set new heat status
			printDateTime();
			Serial.println(F("Start Theatre High Heat"));
			theatreFanControl(fanHighRun);	// Run high fan
			break;
			
		case heatFlameout:
			relayCommand(relayBankA, relayTheatreHeat, relayOff);	// Turn zone heating off
			zoneHeatState[zoneTheatre] = heatOff;	// Set new heat status
			theatreFanControl(fanOff);	// Stop Theatre fan
			printDateTime();
			Serial.println(F("Theatre Flameout Stop"));
			break;
			
		default:
			printDateTime();
			Serial.println(F("Unsupported Theatre heat status request"));
		}
}		

/*
	Purpose: Foyer Heat control driver. Maps logical heat controls to hardware.
	Accepts: heatStatus = required heat state
*/

void foyerHeatControl(byte heatStatus)	{

	/* NB Foyer Gas heater has automatic fan control
		Override with fan control so that fan status is displayed on web page.
	*/
	
	switch (heatStatus)	{
		case heatOff:
			relayCommand(relayBankA, relayFoyerHeat, relayOff);	// Turn zone heating off
			zoneHeatState[zoneFoyer] = heatOff;	// Set new heat status
			printDateTime();
			Serial.println(F("Stop Foyer Heat"));
			fanControl(zoneFoyer, fanDelayedStop);	// Delayed fan shutdown to cool gas heater. May overide/extend hardware automatic fan control
			break;
			
		case heatLow:
			relayCommand(relayBankA, relayFoyerHeat, relayOn);	// Turn zone heating on.
			zoneHeatState[zoneFoyer] = heatLow;	// Set new heat status
			printDateTime();
			Serial.println(F("Start Foyer High Heat"));
			foyerFanControl(fanHighRun);	// Override automatic control - run high fan
			break;
		case heatHigh:
			relayCommand(relayBankA, relayFoyerHeat, relayOn);	// Turn zone heating on.
			zoneHeatState[zoneFoyer] = heatHigh;	// Set new heat status
			printDateTime();
			Serial.println(F("Start Foyer High Heat"));
			foyerFanControl(fanHighRun);	// Override automatic control - run high fan
			break;
		default:
			printDateTime();
			Serial.println(F("Unsupported Foyer heat status request"));
		}
}		


/*
	Purpose: Rolling display on LCD
	Accepts: Uses global displayPageLCD page number
 */
void displayLCD(Task* me) {

	if (!sensorsOnline) return;	// Skip until first sensor scan.
	
	switch (displayPageLCD)	{
	
		case 0:
			displayPageLCD = 1;	// Reset page counter
			break;
		case 1:
			DisplayDateTimeLCD();
			displayPageLCD++;
			break;
		case 2:
			DisplayTempHum(zoneTheatre);
			displayPageLCD++;
			break;
		case 3:
			DisplayTempHum(zoneFoyer);
			displayPageLCD++;
			break;
		case 4:
			DisplayTempHum(zoneStage);
			displayPageLCD++;
			break;
		case 5:
			DisplayTempHum(zoneDressingRoom);
			displayPageLCD++;
			break;
		case 6:
			DisplayTempHum(zoneOutside);
			displayPageLCD++;
			break;
		case 7:
			DisplayTempHum(zoneDuct);
			displayPageLCD = 1;	// Reset page counter
			break;

		default:
			displayPageLCD = 1;	// Reset page counter
		}
}				

/*
	Purpose: Task runs web server

*/

void runWebServer(Task* me) {
	char buff[256];
	int len = 256;

	for (int i = 0; i <= 2; i++){
 

	/* process incoming connections one at a time forever */
	webserver.processConnection(buff, &len);
	}

}

/*
	Purpose: Scans control panel mode switch, web controls and weekly schedule to set zone state.
*/
void runScheduler(Task* me) {

		
	checkZoneSwitch(zoneTheatre);	// Theatre
	
	checkZoneSwitch(zoneFoyer);	// Foyer
}


/* Purpose: Manages zone state changes
	Accepts: Required zone state

	const byte zoneStateOff = 0;
	const byte zoneStateRun = 1;
	const byte zoneStateScheduleOff = 2;
	const byte zoneStateScheduleOn = 3;
	const byte zoneStateManualOff = 4;
	const byte zoneStateManualOn = 5;
	const byte zoneStateFlameout = 6;
	const byte zoneStateVentilateCheck = 7;
	const byte zoneStateVentilateFanRunning = 8;
	
*/


void zoneControl(byte zoneIndex, byte newZoneState)	{

	switch (newZoneState)	{
	
		case zoneStateOff:
			zoneShutdown(zoneIndex);	// Shutdown any zone heat, fan or cooling
			zoneState[zoneIndex] = zoneStateOff;
			break;
			
		case zoneStateRun:
			zoneState[zoneIndex] = zoneStateRun;
			break;
			
		case zoneStateScheduleOff:
			// Check current state
			if (zoneState[zoneIndex] != zoneStateScheduleOff)	{
				zoneShutdown(zoneIndex);	// Shutdown any zone heat, fan or cooling
			}
			zoneState[zoneIndex] = zoneStateScheduleOff;
			break;
		
		case zoneStateScheduleOn:
		
			// Check current state
			if (zoneState[zoneIndex] != zoneStateScheduleOn)	{
				zoneShutdown(zoneIndex);	// Shutdown any zone heat, fan or cooling
			}
			zoneState[zoneIndex] = zoneStateScheduleOn;
			break;
		
		case zoneStateManualOff:
			// Check current state
			if (zoneState[zoneIndex] != zoneStateManualOff)	{
				zoneShutdown(zoneIndex);	// Shutdown any zone heat, fan or cooling
			}
			zoneState[zoneIndex] = zoneStateManualOff;
			break;
		
		case zoneStateManualOn:
			zoneState[zoneIndex] = zoneStateManualOn;
			break;
		
		case zoneStateFlameout:
			zoneState[zoneIndex] = zoneStateFlameout;
			break;
		
		case zoneStateVentilateCheck:
			// Check current state
			if (zoneState[zoneIndex] != zoneStateVentilateCheck)	{
				zoneShutdown(zoneIndex);	// Shutdown any zone heat, fan or cooling
			}
			zoneState[zoneIndex] = zoneStateVentilateCheck;
			break;
		
		case zoneStateVentilateFanRunning:
			zoneState[zoneIndex] = zoneStateVentilateFanRunning;
			break;
		
		default:
			zoneShutdown(zoneIndex);	// Shutdown any zone heat, fan or cooling
			zoneState[zoneIndex] = zoneStateOff;
		}
}

/*
	Purpose: Reads zone control panel switch state and controls zone state
	Accepts: zoneIndex

*/

void checkZoneSwitch(byte zoneIndex)	{

	// Check zone control panel switch
	
	byte zoneControlSwitch = readZoneSwitch(zoneIndex);	// Get control panel switch setting
	
	switch (zoneControlSwitch)	{
	
		// Control panel switch: Off
		case zoneSwitchOff:
			zoneControl(zoneIndex, zoneStateOff);
			break;
			
		// Control panel switch: On
		case zoneSwitchRun:
			zoneControl(zoneIndex, zoneStateRun);
			break;
			
		// Control panel switch: Auto	
		case zoneSwitchAuto:
		
			zoneSwitchState[zoneIndex] = zoneSwitchAuto;	// Set new control panel zone switch state
			
			if (zoneState[zoneIndex] != zoneStateFlameout) {

				// Check if web Weekly Schedule is Off.
				if (weeklyScheduleArray[0] == 0 ) {

					if (zoneState[zoneIndex] == zoneStateManualOff || zoneState[zoneIndex] == zoneStateManualOn) {
					}
					else {
						zoneControl(zoneIndex, zoneStateManualOff);	// Shutdown any zone heat, fan or cooling
					}
				}
				else if (weeklyScheduleArray[0] == 1 ) {
				
					/* Check current day/Hour against weekly schedule. If time is start of scheduled hour, set zoneState[zoneIndex] = zoneStateScheduleOn
						If end of scheduled hour is found and zone ventilate is enabled, set zoneState[zoneIndex] =  zoneStateVentilateCheck, otherwise 
						set zoneStateScheduleOff
					*/
					checkCurrentSchedule(zoneIndex);	
				}
			}
			break;
		}
}
			

	
/*
	Purpose: Shutdown a zone heating or cooling
	Accepts: zoneIndex
*/

void zoneShutdown(byte zoneIndex)	{

	if (zoneHeatState[zoneIndex] != heatOff)	{
		heatControl(zoneIndex, heatOff);	// Stop zone heating with delayed fan shutdown
		return;
	}
	
	if (zoneFanState[zoneIndex] != fanOff)	{
		fanControl(zoneIndex, fanOff);	//Stop any fan and cool pump
	}
}

/*
	Purpose: Checks schedule and sets zone state to zoneStateScheduleOn if hour is scheduled.
		If hour is unscheduled and zone ventilation is enabled and zone is not in zoneStateVentilateCheck or 
		zoneStateVentilateFanRunning state, sets zone to zoneStateVentilateCheck.
		If zone ventilation is not enabled, sets zone state to zoneStateScheduleOff.
	
	Accepts: zoneIndex, pointer to current day hourly schedule array.

*/

void checkScheduleHour(byte zoneIndex, byte *daySchedule)	{


	if (daySchedule[hour()] == 1) {
		zoneControl(zoneIndex, zoneStateScheduleOn);	// Set state to schedule On time
	}
	else if (daySchedule[hour()] == 0 )	{
	
		// Check if ventilate is enabled in this zone
		if (zoneVentilateArray[zoneIndex] == statusVentilateOn)	{

			if (!(zoneState[zoneIndex] == zoneStateVentilateCheck || zoneState[zoneIndex] == zoneStateVentilateFanRunning))	{
				zoneControl(zoneIndex, zoneStateVentilateCheck);	// Shutdown any running systems in zone and enable zone ventilation mode
			}
		}	
		else {
			zoneControl(zoneIndex, zoneStateScheduleOff);	// Shutdown any running systems in zone and wait for next scheduled run
		}
	}	
}
	
/*
	Purpose: Calls checkScheduleHour with address of current day's hourly schedule array.
	Accepts: zoneIndex
	
*/
void checkCurrentSchedule(byte zoneIndex)	{

	byte * daySchedulePointer;	// Pointer to array
	
	switch (weekday()) {
		case 1:
		daySchedulePointer = schSunday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;
		case 2:
		daySchedulePointer = schMonday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;
		case 3:
		daySchedulePointer = schTuesday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;
		case 4:
		daySchedulePointer = schWednesday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;
		case 5:
		daySchedulePointer = schThursday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;
		case 6:
		daySchedulePointer = schFriday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;
		case 7:
		daySchedulePointer = schSaturday;
		checkScheduleHour(zoneIndex, daySchedulePointer);
		break;		
	}
}

	
/*
	Purpose: Reads the control switch property of a zone
	Accepts: zoneIndex
	Returns: 0 = Off, 1 = Run, 2 = Auto, 3 = invalid
*/

byte readZoneSwitch(byte zoneIndex)	{

		switch (zoneIndex)	{
		
		case zoneTheatre:
			return switchTheatre();
			break;
			
		case zoneFoyer:
			return switchFoyer();
			break;
			
		default:
			return 3;
		}
}

 /*
	Purpose:	Runs every second to check for start of each 5 minutes interval in the hour.
 				Calls logSensors every 5 minutes.
*/
 
void sensorLogger(Task* me) {

	
	time_t t = now(); // Store the current time in time variable t 
	
	if (minute(t) == g_nextLogMinute) {
		
		g_nextLogMinute = g_nextLogMinute + 5;	// Set next run minute.
		logSensors();
		
		if (g_nextLogMinute == 60) g_nextLogMinute = 0;	// Reset minutes run
	}
}

/*
	Purpose: Log zone temperature, humidity, relay settings and zone state to csv and html monthly
		log files on SD card.
	
*/

 void logSensors()	{
 	// Run task every 5 minutes, synced to the hour.
	// Log data to SD card.
	if (sensorsOnline) {
		// Log if sensors on line.
		if(!logTemperatureHumidity()); {
			// Error in logTemperatureHumidity - change current directory to root.
			if (!sd.chdir()) {
				printDateTime();
				Serial.println(F("Error in logTemperatureHumidity"));
			}
		}
		if(!htmlTemperatureHumidity()); {
			// Error in htmlTemperatureHumidity - change current directory to root.
			if (!sd.chdir()) {
				printDateTime();
				Serial.println(F("Error in htmlTemperatureHumidity"));
			}
		}
	}		
}
	
 

//  Functions *************************************************************************************************************

/*
	Purpose:	Restore system settings from EEPROM.
	
*/
	
void restoreSettings() {

/*
	int address = 0;	// base EEPROM address.

	// Construct copy of local timezone to get size in bytes for EEPROM address allocation.
	//Australia Eastern Time Zone (Sydney, Melbourne)
	TimeChangeRule aEDT = {"AEDT", First, Sun, Oct, 2, 660};    //UTC + 11 hours
	TimeChangeRule aEST = {"AEST", First, Sun, Apr, 3, 600};    //UTC + 10 hours
	Timezone ausET(aEDT, aEST);
	
	// EEPROM Memory allocation
	// Always get the addresses first and in the same order
	
	int addressAusET = EEPROM.getAddress(sizeof(aEDT) + sizeof(aEST));
	int addressLastModifiedTime = EEPROM.getAddress(sizeof(time_t));

    int addressZoneThermostatSetting = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
    int addressZoneThermostatHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
	
    int addressZoneCoolThermostatSetting = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
    int addressZoneCoolThermostatHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
	
    int addressZoneMaxHumidity = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
    int addressZoneMaxHumidityHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);	
	
	int addressZoneCoolHighFanPoint = EEPROM.getAddress(sizeof(float)* g_numberOfZones);
	int addressZoneCoolHighFanHysteresis = EEPROM.getAddress(sizeof(float)* g_numberOfZones);	

	// Weekly Schedule day / hour settings arrays
	int addressSchSunday = EEPROM.getAddress(sizeof schSunday / sizeof *schSunday);
	int addressSchMonday = EEPROM.getAddress(sizeof schMonday / sizeof *schMonday);
	int addressSchTuesday = EEPROM.getAddress(sizeof schTuesday / sizeof *schTuesday);
	int addressSchWednesday = EEPROM.getAddress(sizeof schWednesday / sizeof *schWednesday);
	int addressSchThursday = EEPROM.getAddress(sizeof schThursday / sizeof *schThursday);
	int addressSchFriday = EEPROM.getAddress(sizeof schFriday / sizeof *schFriday);
	int addressSchSaturday = EEPROM.getAddress(sizeof schSaturday / sizeof *schSaturday);
	
	int addressWeeklyScheduleArray = EEPROM.getAddress(sizeof weeklyScheduleArray / sizeof *weeklyScheduleArray);
	int addressZoneVentilateArray = EEPROM.getAddress(sizeof zoneVentilateArray / sizeof *zoneVentilateArray);
 
*/
 	
	// Read the stored values in volitile storage
	
//	EEPROM.readBlock<byte>(addressAusET, *tcr, sizeof(aEDT) + sizeof(aEST));

	EEPROM.readBlock<float>(addressZoneThermostatSetting, zoneThermostatSetting, g_numberOfZones);
	EEPROM.readBlock<float>(addressZoneThermostatHysteresis, zoneThermostatHysteresis, g_numberOfZones);
	
	EEPROM.readBlock<float>(addressZoneCoolThermostatSetting, zoneCoolThermostatSetting, g_numberOfZones);
	EEPROM.readBlock<float>(addressZoneCoolThermostatHysteresis, zoneCoolThermostatHysteresis, g_numberOfZones);
	
	EEPROM.readBlock<float>(addressZoneMaxHumidity, zoneMaxHumidity, g_numberOfZones);
	EEPROM.readBlock<float>(addressZoneMaxHumidityHysteresis, zoneMaxHumidityHysteresis, g_numberOfZones);
	
	EEPROM.readBlock<float>(addressZoneCoolHighFanPoint, zoneCoolHighFanPoint, g_numberOfZones);
	EEPROM.readBlock<float>(addressZoneCoolHighFanHysteresis, zoneCoolHighFanHysteresis, g_numberOfZones);

	EEPROM.readBlock<byte>(addressSchSunday, schSunday, scheduleHours);	
	EEPROM.readBlock<byte>(addressSchMonday, schMonday, scheduleHours);
	EEPROM.readBlock<byte>(addressSchTuesday, schTuesday, scheduleHours);
	EEPROM.readBlock<byte>(addressSchWednesday, schWednesday, scheduleHours);
	EEPROM.readBlock<byte>(addressSchThursday, schThursday, scheduleHours);
	EEPROM.readBlock<byte>(addressSchFriday, schFriday, scheduleHours);
	EEPROM.readBlock<byte>(addressSchSaturday, schSaturday, scheduleHours);

	EEPROM.readBlock<byte>(addressWeeklyScheduleArray, weeklyScheduleArray, numSchedulePressButtons);
	EEPROM.readBlock<byte>(addressZoneVentilateArray, zoneVentilateArray, g_numberOfZones);
	
}

/*
	Purpose:	Round up positive numbers to the nearest multiple
	Accepts:	Number to round, Multiple
	Returns: 	Number rounded up to nearst multiple. 
	Example:	roundup(32,5) = 35
*/

int roundUp(int numToRound, int multiple)	{

  return multiple * ((numToRound + multiple - 1) / multiple);
}







/*
	Purpose: Reads digital inputs and converts to code
	Returns:	0 = Off, 1 = Run, 2 = Auto, 3 = invalid
	
	Control Panel zone mode switch digital input pins.
	
	Pin 1	Pin 0			Return	
		1 		0 = Off			0
		1		1 = Run			1
		0		1 = Auto		2	

*/

byte switchTheatre()	{

/*
	Pin 1	Pin 0	
		1 		0 = Off
		1		1 = Run
		0		1 = Auto
*/
	if (digitalRead(zoneTheatreSwitchPin1) && !digitalRead(zoneTheatreSwitchPin0) ) {
		return 0;
	} else if (digitalRead(zoneTheatreSwitchPin1) && digitalRead(zoneTheatreSwitchPin0)) {
		return 1;
	} else if (digitalRead(zoneTheatreSwitchPin0) && !digitalRead(zoneTheatreSwitchPin1) ) {
		return 2;
	} 
	return 3;
}

/* Purpose: Reads digital inputs and converts to code
	Returns:	0 = Off, 1 = Run, 2 = Auto, 3 = invalid

*/

byte switchFoyer()	{
	int switchval;
	if (digitalRead(zoneFoyerSwitchPin1) && !digitalRead(zoneFoyerSwitchPin0) ) {
		return 0;
	} else if (digitalRead(zoneFoyerSwitchPin1) && digitalRead(zoneFoyerSwitchPin0)) {
		return 1;
	} else if (digitalRead(zoneFoyerSwitchPin0) || !digitalRead(zoneFoyerSwitchPin1) ) {
		return 2;
	} 
	return 3;
}


/*
	Purpose:	Compares state of latch bits and flag bit.
	Accepts:	latchState = 8 bit flags to be tested.
				flagBit = bit to be tested.
	Returns:	true if corresponding flag bit is 1.
*/

bool flagBit(uint8_t &latchState, uint8_t bitFlag) {

	return (latchState & bitFlag);
	
}

/*
	Purpose: Create or open file for append on SD
	Accepts: file name
	
*/

void fileOpen(char fileName[]) {

	// create or open a file for append
	ofstream sdData(fileName, ios::out | ios::app);

	// check for errors
	if (!sdData) sd.errorHalt("Log file init failed");
  
}


/*

 Udp NTP Client
 
 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket 
 For more on NTP time servers and the messages needed to communicate with them, 
 see http://en.wikipedia.org/wiki/Network_Time_Protocol
 
 Warning: NTP Servers are subject to temporary failure or IP address change.
 Plese check 
 
 http://tf.nist.gov/tf-cgi/servers.cgi
 
 if the time server used in the example didn't work.
 
 created 4 Sep 2010 
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe
 
 This code is in the public domain.
 
 */

/*
	Purpose: Send an NTP request to the time server at the given address
	Accepts: address of IP address object
	
*/

unsigned long sendNTPpacket(IPAddress& address) {

	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE); 
	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)
	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	packetBuffer[1] = 0;     // Stratum, or type of clock
	packetBuffer[2] = 6;     // Polling Interval
	packetBuffer[3] = 0xEC;  // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12]  = 49; 
	packetBuffer[13]  = 0x4E;
	packetBuffer[14]  = 49;
	packetBuffer[15]  = 52;

	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp: 		   
	Udp.beginPacket(address, 123); //NTP requests are to port 123
	Udp.write(packetBuffer,NTP_PACKET_SIZE);
	Udp.endPacket(); 
}

/*
	Purpose: Trys to get NTP time. Convert UTC time to local time. Set RTC and system time
	Returns: Local time_t if successful, 0 if not available.
  
 */
 
 
 
time_t GetNTPTime() {

	int i;
	unsigned long epoch;
	unsigned long localTime;

	// Serial.println(F("Starting NTP get loop"));
	// NTP time try count.
	for (int i = 1; i <= 3; i++){
		sendNTPpacket(timeServer); // send an NTP packet to a time server

		// wait to see if a reply is available
		delay(1000);  
		if ( Udp.parsePacket() ) {  
			// We've received a packet, read the data from it
			Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer

			//the timestamp starts at byte 40 of the received packet and is four bytes,
			// or two words, long. First, extract the two words:

			unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
			unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);  
			// combine the four bytes (two words) into a long integer
			// this is NTP time (seconds since Jan 1 1900):
			unsigned long secsSince1900 = highWord << 16 | lowWord;  
			// Serial.print(F("Seconds since Jan 1 1900 = " );
			// Serial.println(secsSince1900);               

			// now convert NTP time into everyday time:
			// Serial.print(F("Unix time = "));
			// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
			const unsigned long seventyYears = 2208988800UL;     
			// subtract seventy years:
			unsigned long epoch = secsSince1900 - seventyYears;  
			// print Unix time:
			Serial.println(epoch);                               

			// Setting The RTC time 

			if(epoch >0) {
				
				Serial.println(F("NTP set RTC & System time"));
				RTC.set(ausET.toLocal(epoch, &tcr));   // set the RTC and the system time to the received value
				setTime(ausET.toLocal(epoch, &tcr));
				return ausET.toLocal(epoch, &tcr); //			break;	// exit the loop
			}
			else {
				return 0;
			}
		}
 		Serial.println(F("Retry NTP"));
		// wait ten seconds before asking for the time again
		//delay(10000); 
	}

	return 0;
}

/*
	Purpose: Digital clock display of system time.
	
*/

void digitalClockDisplay(){
	// digital clock display of the time
	Serial.print(hour());
	printDigits(minute(),":");
	printDigits(second(),":");
	Serial.print(F(" "));
	Serial.print(day());
	Serial.print(F(" "));
	Serial.print(month());
	Serial.print(F(" "));
	Serial.print(year()); 
	Serial.println(); 
}


void printDigits(int digits, const char* separatorChar){
	// utility function for digital clock display: prints preceding separator Char and leading 0
	Serial.print(separatorChar);
	if(digits < 10)
	Serial.print('0');
	Serial.print(digits);
}

void printDigitsLCD(int digits, const char* separatorChar){
	// utility function for digital clock display on LCD: prints preceding separator Char and leading 0
	lcd.print(separatorChar);
	if(digits < 10)
	lcd.print('0');
	lcd.print(digits);
}


void printDec2(int value) {
	Serial.print((char)('0' + (value / 10)));
	Serial.print((char)('0' + (value % 10)));
}

// From HardwareRTC

//Function to print time with time zone
void printTime(time_t t, const char *tz) {
	sPrintI00(hour(t));
	sPrintDigits(minute(t));
	sPrintDigits(second(t));
	Serial.print(' ');
	Serial.print(dayShortStr(weekday(t)));
	Serial.print(' ');
	sPrintI00(day(t));
	Serial.print(' ');
	Serial.print(monthShortStr(month(t)));
	Serial.print(' ');
	Serial.print(year(t));
	Serial.print(' ');
	Serial.print(tz);
	Serial.println();
}

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int val) {
	if (val < 10) Serial.print('0');
	Serial.print(val, DEC);
	return;
}

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int val) {
	Serial.print(':');
	if(val < 10) Serial.print('0');
	Serial.print(val, DEC);
}


// Print Date Time in dd/mm/yyyy 24 hour:mm:ss format

void printDateTime()
{
	Serial.print(day());
	printDigits(month(),"/");
	printDigits(year(),"/"); 
	Serial.print(F(" ")); 
	Serial.print(hour());
	printDigits(minute(),":");
	printDigits(second(),":");
	Serial.print(F(" "));
	
}

// Return  local date/time string in format: yy-mm-dd 24hour-mm-ss 
void dateTime() {

	//get the current local time
//	Timezone ausET(addressAuMelbourneTZ);       // Rules stored at EEPROM address.
	TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
	time_t utc, local;


	Serial.println();
	utc = now();
	printTime(utc, "UTC");
	//  local = myTZ.toLocal(utc, &tcr);
	local = ausET.toLocal(utc, &tcr);
	printTime(local, tcr -> abbrev);

  
	// Construct the date/time string.
	
	String stringDateTime = String(year(local));
	
	stringDateTime = String(stringDateTime + "-");
	stringDateTime = String(stringDateTime + month(local));
	stringDateTime = String(stringDateTime + "-");
	stringDateTime = String(stringDateTime + day(local));
	stringDateTime = String(stringDateTime + " ");
	
	stringDateTime = String(stringDateTime + hourFormat12(local));
	stringDateTime = String(stringDateTime + ".");
	
	stringDateTime = String(stringDateTime + minute(local));
	stringDateTime = String(stringDateTime + ".");
	
	stringDateTime = String(stringDateTime + second(local));
	stringDateTime = String(stringDateTime + " ");
	
	if (isAM(local))
		{ stringDateTime = String(stringDateTime + "AM"); }
	else
		{ stringDateTime = String(stringDateTime + "PM"); }
	
	
	Serial.println();
	Serial.println(stringDateTime);
	
}

/*
	Purpose: Create local date string in format: d/mm/yyyy
	Accepts: Reference to a String object for local date.

*/

void localDate(String &stringDate) {

	// Construct the date string.
	
	stringDate = String(day());
	stringDate = String(stringDate + "/");
	if (month() < 10)
		{ stringDate = String(stringDate + "0"); }
		
	stringDate = String(stringDate + month());
	stringDate = String(stringDate + "/");
	stringDate = String(stringDate + year());
	stringDate = String(stringDate + " ");
	
}

/*
	Purpose: Create local time string in format: hh:mm:ss
	Accepts: Reference to a String object for local time.

*/

void localTime(String &stringTime) {

	// Construct the date string.
	
	stringTime = String(hour());
	stringTime = String(stringTime + ":");
	if (minute() < 10)
		{ stringTime = String(stringTime + "0"); }
		
	stringTime = String(stringTime + minute());
	stringTime = String(stringTime + ":");
	
	if (second() < 10)
		{ stringTime = String(stringTime + "0"); }
	
	stringTime = String(stringTime + second());
	stringTime = String(stringTime + " ");
	
}

/*
	Purpose: Create a log file name date string in format: Logyyyymm
	Accepts: Reference to a String object for log file name in form "LogYYYYMM" 

*/

void logYearMonth(String &stringLogYearMonth) {

	// Construct the date string.
	stringLogYearMonth = "Log";
	stringLogYearMonth = String(stringLogYearMonth + year());
	if (month() < 10) {
		stringLogYearMonth = String(stringLogYearMonth + "0");	// Add leading zero
		stringLogYearMonth = String(stringLogYearMonth + month());
	}
	else {
		stringLogYearMonth = String(stringLogYearMonth + month());
	}
}

/*
 Purpose: Constructs test string in the form:
			Tag + number + = + text
			Example Su_0=0
			
 Accepts:	&stringBuffer Pointer to string buffer.
			idTag	Pointer to id string.
			number 	Appended to id string.
			state	append 0 or 1 char to string.
 */
 
 void ResponceMask(String &stringBuffer, char* idTag, int number, char* state) {
 
		stringBuffer = idTag;	// Load primary id string into string buffer.
		stringBuffer = String(stringBuffer + number);	// add number
		stringBuffer = String(stringBuffer + "=");	// add equels
		stringBuffer = String(stringBuffer + state);	//	add	state, 0 or 1
}

/*
 Purpose: Create XML data string in format "<Tag>Data</Tag>"
 Accepts: Reference to a String object for XML data string, Tag*, Tag number, xml Data text
		If number == 9999 then tag number is not appended to xmlTag.
		If string has reached limit, trasmit over TCIP packet.
*/

void xmlDataFormat(String &stringXmlData, const char* xmlTag, int number, const char* xmlData, WebServer &server ) {

	// Construct the XML string.
	
	stringXmlData = String(stringXmlData + "<");	// Append "<"
	stringXmlData = String(stringXmlData + xmlTag);
	if (number == 9999 ) {
		stringXmlData = String(stringXmlData);
	} 
	else {
		stringXmlData = String(stringXmlData + number);	// Add number to tag if number > 0
	}	
	stringXmlData = String(stringXmlData + ">");
	stringXmlData = String(stringXmlData + xmlData);
	stringXmlData = String(stringXmlData + "</");
	stringXmlData = String(stringXmlData + xmlTag);
	if (number == 9999 ) {
		stringXmlData = String(stringXmlData);
	} 
	else {
		stringXmlData = String(stringXmlData + number);	// Add number to tag if number > 0
	}	
	stringXmlData = String(stringXmlData + ">");
	
	// Check if string has reached it's size limit.
	if (strXmlData.length() > XML_DATA_FORMAT_BUFFER_LIMIT) {
		server.print(strXmlData);
		strXmlData = "";	// Clear string
	}
	
}

/*
 * Append a line to SYSLOG.TXT
 */
void logEvent(String &strLogLine) {

	digitalWrite(4, HIGH);   // SD Card SPI SS

  // create dir if needed
  sd.mkdir("LOGS");

   // create or open a file for append
  ofstream sdData("LOGS/SYSLOG.TXT", ios::out | ios::app);

  // check for errors
  if (!sdData) sd.errorHalt("Log file init failed");
  
  Serial.println(strLogLine);
  
  // append a line to the file
  sdData << strLogLine << endl;

  // check for errors
  if (!sdData) sd.errorHalt("Log file append failed");

  sdData.close();
}

/*
 * Purpose:	Append current time, date, temperature, humidity and relay state to log
 * Returns:	true = log succesful, false = fail.
 */
bool logTemperatureHumidity() {

	bool newMonthFile = true;
	char caLogYearMonth[14];	// Char array for log file name
	char caLocalTime[9];
	char caLocalDate[11];

	digitalWrite(4, HIGH);   // SD Card SPI SS

	// Change current directory to root.
	if (!sd.chdir()) {
		printDateTime();
		Serial.println(F("logTemperatureHumidity chdir to root failed"));
		return false;
	}
	sd.mkdir("LOGS");
	// Change volume working directory to Logs Folder
	if (!sd.chdir("LOGS")) {
		printDateTime();
		Serial.println(F("logTemperatureHumidity: chdir failed for LOGS folder."));
		return  false;
	}
	// Create log file name "Logyyyymm"
	logYearMonth(strLogYearMonth);	// Update log year month string
	strLogYearMonth = String(strLogYearMonth + ".csv");	// Add .csv	
	printDateTime();
	Serial.print("Log file name = ");
	Serial.println(strLogYearMonth);

	strLogYearMonth.toCharArray(caLogYearMonth, 14);	// Conver String to Char array
	// Check if log file exists
	if (!sd.exists(caLogYearMonth)) {
		// Create new log file in current directory.
		if (!file.open(caLogYearMonth, O_CREAT | O_WRITE)) {
			printDateTime();
			Serial.println(F("logTemperatureHumidity: Create log file failed"));
			return  false;
		}
		file.close();
		newMonthFile = true;
	} else {
		newMonthFile = false;
	}
	// create or open a file for append
	ofstream sdLogger(caLogYearMonth, ios::out | ios::app);
	// check for errors
	if (!sdLogger) {
		printDateTime();
		Serial.println(F("logTemperatureHumidity: Logger file init failed"));
		return  false;
	}
	if (newMonthFile) {
		// new file created - add headings.
		sdLogger << "Date,Time,Theatre,Duct,Foyer,Outside,Dressing Room,Stage,Theatre,Duct,Foyer,Outside,"
		<< "Dressing Room,Stage,Theatre Heat,Foyer Heat,Ext Fan,Cool Pump,Run Fan,Fan Controller,Foyer Fan,Flameout,Theatre State,Foyer State" << endl;
		newMonthFile = false;
	}

	localDate(strLocalDate);	// Update local date string
	localTime(strLocalTime);	// Update local time string

	strLocalTime.toCharArray(caLocalTime, 9);
	strLocalDate.toCharArray(caLocalDate, 11);
	csvFlags(relayBankA, csvFlagByte);
	
	cout << caLocalDate << " " << caLocalTime << " Relays " << csvFlagByte << endl;
	
	sdLogger << caLocalDate << "," << caLocalTime << "," 
	<< zoneTemperature[0] << "," << zoneTemperature[1] << "," << zoneTemperature[2] << "," << zoneTemperature[3] << ","
	<< zoneTemperature[4] << "," << zoneTemperature[5] << ","
	<< zoneHumidity[0] << ","  << zoneHumidity[1] << "," << zoneHumidity[2] << ","  << zoneHumidity[3] << ","
	<< zoneHumidity[4] << ","  << zoneHumidity[5] << ","
	<< csvFlagByte << "," << byte2str(zoneState[zoneTheatre])<< "," << byte2str(zoneState[zoneFoyer])
	<< endl;

	// check for errors
	if (!sdLogger) {
		printDateTime();
		Serial.println(F("logTemperatureHumidity: append failed"));
		sdLogger.close();
		return false;
	}
	sdLogger.close();

	// Change current directory to root.
	if (!sd.chdir()) {
		printDateTime();
		Serial.println(F("logTemperatureHumidity chdir to root failed"));
		return false;
	}
	return true;
}


/*
 * Purpose:	Append current time, date, temperature, humidity and relay state to html log
 * Returns:	true = log succesful, false = fail.
 */
 
bool htmlTemperatureHumidity() {

	bool newMonthFile = true;
	char caLogYearMonth[14];	// Char array for log file name
	char caLocalTime[9];
	char caLocalDate[11];

	digitalWrite(4, HIGH);   // SD Card SPI SS

	// Change current directory to root.
	if (!sd.chdir()) {
		printDateTime();
		Serial.println(F("htmlTemperatureHumidity chdir to root failed"));
		return false;
	}
	sd.mkdir("LOGS");
	// Change volume working directory to Logs Folder
	if (!sd.chdir("LOGS")) {
		printDateTime();
		Serial.println(F("htmlTemperatureHumidity: chdir failed for LOGS folder."));
		return  false;
	}
	// Create html file name "Logyyyymm"
	logYearMonth(strHtmlLogYearMonth);	// Update log year month string
	strHtmlLogYearMonth = String(strHtmlLogYearMonth + ".htm");	// Add .htm
	printDateTime();
	Serial.print("html Log file name = ");
	Serial.println(strHtmlLogYearMonth);

	strHtmlLogYearMonth.toCharArray(caLogYearMonth, 14);	// Conver String to Char array
	// Check if log file exists
	if (!sd.exists(caLogYearMonth)) {
		// Create new log file in current directory.
		if (!file.open(caLogYearMonth, O_CREAT | O_WRITE)) {
			printDateTime();
			Serial.println(F("htmlTemperatureHumidity: Create html file failed"));
			return  false;
		}
		file.close();
		newMonthFile = true;
	} else {
		newMonthFile = false;
	}
	// create or open a file for append
	ofstream sdHtmlLogger(caLogYearMonth, ios::out | ios::app);
	// check for errors
	if (!sdHtmlLogger) {
		printDateTime();
		Serial.println(F("logTemperatureHumidity: HTML Logger file init failed"));
		return  false;
	}
	if (newMonthFile) {
		// new file created - add HTML Table headings.
		sdHtmlLogger << (F("<!DOCTYPE html><html><head>"))
			<< (F("<style>table, th, td {border: 1px solid black; border-collapse: collapse;}th, td {padding: 5px;}th {text-align: left;}td {text-align: right;}</style>"))
			<< (F("</head><body>"))
			<< (F("<table style='width:100%'><thead><tr>"))
			<< (F("<th>Date</th><th>Time</th><th>Theatre</th><th>Duct</th><th>Foyer</th><th>Outside</th><th>Dressing Room</th><th>Stage</th>"))
			<< (F("<th>Theatre</th><th>Duct</th><th>Foyer</th><th>Outside</th><th>Dressing Room</th><th>Stage</th><th>Theatre Heat</th>"))
			<< (F("<th>Foyer Heat</th><th>Ext Fan</th><th>Cool Pump</th><th>Run Fan</th><th>Fan Controller</th><th>Foyer Fan</th><th>Flameout</th>"))
			<< (F("<th>Theatre State</th><th>Foyer State</th></tr>"))
			<< endl;
		newMonthFile = false;
	}

	localDate(strLocalDate);	// Update local date string
	localTime(strLocalTime);	// Update local time string

	strLocalTime.toCharArray(caLocalTime, 9);
	strLocalDate.toCharArray(caLocalDate, 11);
//	csvFlags(relayBankA, csvFlagByte);
	
	sdHtmlLogger << (F("<tr><td>")) << caLocalDate << (F("</td><td>")) << caLocalTime << (F("</td><td>"))
	<< zoneTemperature[0] << (F("</td><td>")) << zoneTemperature[1] << (F("</td><td>")) << zoneTemperature[2] << (F("</td><td>")) << zoneTemperature[3] << (F("</td><td>"))
	<< zoneTemperature[4] << (F("</td><td>")) << zoneTemperature[5] << (F("</td><td>"))
	<< zoneHumidity[0] << (F("</td><td>"))  << zoneHumidity[1] << (F("</td><td>")) << zoneHumidity[2] << (F("</td><td>"))  << zoneHumidity[3] << (F("</td><td>"))
	<< zoneHumidity[4] << (F("</td><td>"))  << zoneHumidity[5] << (F("</td>"));
//	<< endl;
	
	htmlFlags(relayBankA, sdHtmlLogger);	// Relay bits as 0 or 1 html table entries.
	
	sdHtmlLogger  << (F("<td>")) << byte2str(zoneState[zoneTheatre])<< (F("</td><td>")) << byte2str(zoneState[zoneFoyer]) << (F("</td></tr>"))
	<< endl;

	// check for errors
	if (!sdHtmlLogger) {
		printDateTime();
		Serial.println(F("htmlTemperatureHumidity: append failed"));
		sdHtmlLogger.close();
		return false;
	}
	sdHtmlLogger.close();

	// Change current directory to root.
	if (!sd.chdir()) {
		printDateTime();
		Serial.println(F("htmlTemperatureHumidity chdir to root failed"));
		return false;
	}
	return true;
}

/*
	Purpose: Convert byte flags to csv character array in form 0|1,0|1, ...
	Accepts: Flag byte, pointer to csv character array.
*/

void csvFlags(uint8_t flagsByte, char strArray[]) {
	int bitMask = 1;	// Bit mask

	for (uint8_t i = 0; i < 16; i++ ) {
	
		
		if (flagsByte & bitMask) {
			strArray[i] = '1';
		} else {
			strArray[i] = '0';
		}
		i++;
		if (i < 14) { 
			strArray[i] = ',';	// add comma.
		} else {
			strArray[i] = '\0';	// Terminate char string
		}
		bitMask = bitMask << 1;	// Shift bit mask 1 bit left.
	}
}	

/*
	Purpose: Convert byte flags to HTML Table character array in form <td> 0 | 1 </td>.....
	Accepts: Flag byte, pointer to HTML character array.
*/
void htmlFlags(uint8_t flagsByte, ofstream &sdFile) {
	int bitMask = 1;	// Bit mask

	for (uint8_t i = 0; i < 16; i++ ) {
	
		
		if (flagsByte & bitMask) {
			sdFile << "<td>1</td>"; 
		} else {
			sdFile << "<td>0</td>";
		}
		i++;
		bitMask = bitMask << 1;	// Shift bit mask 1 bit left.
	}
}

float ReadTempHum(int SensorChannel, float SensorData[]){;
  // Function: Reads DHT temperature and humidity sensor data into array.
  // Accepts:  SensorChannel = Sensor pin number.
  //            SensorData = pointer to arrray to return data.

	DHT dht(SensorChannel, DHTTYPE);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  SensorData[0] = dht.readHumidity();
  SensorData[1] = dht.readTemperature();

}

/*
	Function:  Check returned temp and humidity and serial print.
	Accepts:    zoneIndex = index to zone data arrays.
	Returns: Nil.
*/ 

void printTempHum(int zoneIndex) {

	printDateTime();	// Date & Time
	
	// check if returns are valid, if they are NaN (not a number) then something went wrong!
	if (isnan(zoneTemperature[zoneIndex]) || isnan(zoneHumidity[zoneIndex])) {
		
		Serial.print(zoneName[zoneIndex]);
		Serial.println(F("\tRead Failed"));
	} else {
		Serial.print(zoneName[zoneIndex]);

		Serial.print(F("\tT:"));
		Serial.print(zoneTemperature[zoneIndex],1);

		if (zoneHumidityAvailable[zoneIndex]) {
			Serial.print(F("\tH:"));
			Serial.print(zoneHumidity[zoneIndex],1);
		}
		
		Serial.print(F("\t"));	// Output tab
		PrintZoneHighHumidity(zoneIndex);	// Print status
		Serial.print(F("\t"));	// Output tab
		PrintFanStatus(zoneIndex);	// Print zone fan status

		if (zoneIndex == zoneTheatre &&  flagBit(relayBankA, relayTheatreHeat)) {
		
			Serial.print(F("\tHeat"));

		} else if (zoneIndex == zoneFoyer && (flagBit(relayBankA, relayFoyerHeat))) {

			Serial.print(F("\tHeat"));
		}
	Serial.println();	
	}
}

/*
	Purpose:	Print zone fan status in the form "Fan: <fan state>
	Accepts:	zone number
	Returns:	Nil
*/	
void PrintFanStatus(int zoneIndex)	{

	switch (zoneFanState[zoneIndex]) {
	
	case fanOff:
		Serial.print(F("Fan: Off"));
		break;
	
	case fanDelayedStop:
		Serial.print(F("Fan: Delay Stop"));
		break;
		
	case fanLowCoolStart:
		Serial.print(F("Fan: Low Cool Start"));
		break;
		
	case fanLowCool:
		Serial.print(F("Fan: Low Cool"));
		break;
		
	case fanLowRun:
		Serial.print(F("Fan: Low Run"));
		break;
		
	case fanMediumCoolStart:
		Serial.print(F("Fan: Medium Cool Start"));
		break;
		
	case fanMediumCool:
		Serial.print(F("Fan: Medium Cool"));
		break;
		
	case fanMediumRun:
		Serial.print(F("Fan: Medium Run"));
		break;
		
	case fanHighCoolStart:
		Serial.print(F("Fan: High Cool Start"));
		break;
		
	case fanHighCool:
		Serial.print(F("Fan: High Cool"));
		break;
		
	case fanHighRun:
		Serial.print(F("Fan: High Run"));
		break;	
	
	default:
		Serial.print(F("Invalid Fan Status"));
		break;
	}
}

/*
	Purpose: Prints zoneHighHumidity status
	Accepts: zone Number
*/

void PrintZoneHighHumidity(int zoneIndex)	{

	if (zoneHighHumidity[zoneIndex]) { 
		Serial.print(F("High Humidity"));
	} 
	else {
	Serial.print(F("Humidity OK"));
	}
}


/*
	Function:	Display date and time in LCD.
	Display Date Time in dd/mm/yyyy 24 hour:mm:ss format
*/
void DisplayDateTimeLCD() {

	lcd.clear();
	lcd.print("Date:");
	lcd.print(day());
	printDigitsLCD(month(),"/");
	printDigitsLCD(year(),"/"); 
	lcd.setCursor(0, 1);
	lcd.print("Time:");
	lcd.print(hour());
	printDigitsLCD(minute(),":");
	printDigitsLCD(second(),":");

	digitalWrite(4, HIGH);   // SD Card SPI SS
	
}

/*
	Function:	Display system start time and error counts on LCD.
	Display Date Time in dd/mm/yyyy 24 hour:mm:ss format
*/

void DisplaySystemLCD() {

	lcd.clear();
	lcd.setCursor(0, 0);

	lcd.print("T:");
	lcd.print(zoneSensorFaults[zoneTheatre]);
	lcd.setCursor(5, 0);
	lcd.print("S:");
	lcd.print(zoneSensorFaults[zoneStage]);	
	lcd.setCursor(10, 0);
	lcd.print("Du:");
	lcd.print(zoneSensorFaults[zoneDuct]);

	lcd.setCursor(0, 1);	
	lcd.print("F:");
	lcd.print(zoneSensorFaults[zoneFoyer]);	
	lcd.setCursor(5, 1);
	lcd.print("D:");
	lcd.print(zoneSensorFaults[zoneDressingRoom]);
	lcd.setCursor(10, 1);
	lcd.print("O:");
	lcd.print(zoneSensorFaults[zoneOutside]);


	digitalWrite(4, HIGH);   // SD Card SPI SS
	
}

/*
	Function:  Check returned temp and humidity and then display on LCD panel.
	Accepts:    zoneIndex = index to zone data arrays.
	Returns: Nil.
*/ 

void DisplayTempHum(int zoneIndex) {

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
	if (isnan(zoneTemperature[zoneIndex]) || isnan(zoneHumidity[zoneIndex])) {
		lcd.clear();
		lcd.print(zoneName[zoneIndex]);
		lcd.setCursor(0, 1);
		lcd.print("Read Failed");
	} else {
		lcd.clear();
		lcd.print(zoneName[zoneIndex]);
		lcd.setCursor(0, 1);
		lcd.print("T:");
		lcd.print(zoneTemperature[zoneIndex]);

		if (zoneHumidityAvailable[zoneIndex]) {
			lcd.setCursor(9, 1);
			lcd.print("H:");
			lcd.print(zoneHumidity[zoneIndex]);
		}
		
		if (zoneIndex == zoneTheatre || zoneIndex == zoneFoyer)	{
		
			// Display Heat|Fan|Cool
			lcd.setCursor(10,0);
			lcd.print(strZoneDisplay(zoneIndex));
		}

	}
  	digitalWrite(4, HIGH);   // SD Card SPI SS
}

/*	Fan modes
	const byte fanOff = 0;
	const byte fanDelayedStop = 1;
	const byte fanLowCoolStart = 2;
	const byte fanLowCool = 3;
	const byte fanLowRun = 4;
	const byte fanMediumCoolStart = 5;
	const byte fanMediumCool = 6;
	const byte fanMediumRun = 7;
	const byte fanHighCoolStart = 8;
	const byte fanHighCool = 9;
	const byte fanHighRun = 10;
*/

/* Purpose: Return abbreviated zone heat/fan/cool status string
	Accepts: zoneIndex
	Returns: Abbreviated zone status string.

*/

const char* strZoneDisplay(int zoneIndex)	{

	// Check if zone is in flameout reset state
	if (zoneState[zoneIndex] == zoneStateFlameout)	{
		return "Heat:F";
	}
	switch (zoneHeatState[zoneIndex])	{
	
		case heatLow:
			return "Heat:L";
			break;
		case heatHigh:
			return "Heat:H";
			break;
		case heatFlameout:	// This is a transient state change and unlikely to be displayed here
			return "Heat:F";
			break;
		default:
			break;
	}
	
	switch (zoneFanState[zoneIndex])	{
	
		case fanDelayedStop:
			return "Fan:DS";
			break;
		case fanLowCoolStart:
			return "Cool:LS";
			break;
		case fanLowCool:
			return "Cool:L";
			break;
		case fanLowRun:
			return "Fan:L";
			break;
		case fanMediumCoolStart:
			return "Cool:MS";
			break;
		case fanMediumCool:
			return "Cool:M";
			break;
		case fanMediumRun:
			return "Fan:M";
			break;
		case fanHighCoolStart:
			return "Cool:HS";
			break;
		case fanHighCool:
			return "Cool:H";
			break;
		case fanHighRun:
			return "Fan:H";
			break;
		default:
			return "";
			break;
	}
}			


/*

 // Display Temperature and Humidity

void DisplayTempC(DeviceAddress deviceAddress, char ZoneName[])
{
  float tempC = sensors.getTempC(deviceAddress);

  // Function:  Get returned temp and display on LCD panel.
  // Accepts:    ZoneName.
  // Returns: Nil.
  
  
  	Serial.print(F("DisplayTempC() Start SS 4 = "));
	Serial.println(digitalRead(4));	// SD Card SPI SS
	Serial.print(F("SS 10 = "));
	Serial.println(digitalRead(10));	// Ethernet SPI SS
  

    lcd.clear();
    lcd.print(ZoneName);
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(tempC);
	
	Serial.print(F("DisplayTempC() Exit SS 4 = "));
	Serial.println(digitalRead(4));	// SD Card SPI SS
	Serial.print(F("SS 10 = "));
	Serial.println(digitalRead(10));	// Ethernet SPI SS
		
	digitalWrite(4, HIGH);   // SD Card SPI SS

 }

 /*
// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
	for (uint8_t i = 0; i < 8; i++) {
		// zero pad the address if necessary
		if (deviceAddress[i] < 16) Serial.print(F("0"));
		Serial.print(deviceAddress[i], HEX);
	}
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print(F("Temp C: "));
  Serial.print(tempC);
  Serial.print(F(" Temp F: "));
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print(F("Resolution: "));
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print(F("Device Address: "));
  printAddress(deviceAddress);
  Serial.print(F(" "));
  printTemperature(deviceAddress);
  Serial.println();
}

  // Print Temperature of devices
 void PrintC(DeviceAddress deviceAddress)
{
 float tempC = sensors.getTempC(deviceAddress);
  Serial.print(F("Temp C: "));
  Serial.print(tempC);
  Serial.println();

}
*/

/*
	Purpose: Poll DHT sensors temperatures and humidity
	
*/

void pollTempHumidity() {

	// Note: readTemperature returns NaN (Not a number) if read from sensor fails.

	zoneTemperature[0] = dht1.readTemperature();
	zoneHumidity[0] = dht1.readHumidity();

	zoneTemperature[1] = dht2.readTemperature();
	zoneHumidity[1] = dht2.readHumidity();

	zoneTemperature[2] = dht3.readTemperature();
	zoneHumidity[2] = dht3.readHumidity();
	  
	zoneTemperature[3] = dht4.readTemperature();
	zoneHumidity[3] = dht4.readHumidity();
	  
	zoneTemperature[4] = dht5.readTemperature();
	zoneHumidity[4] = dht5.readHumidity();
	  
	zoneTemperature[5] = dht6.readTemperature();
	zoneHumidity[5] = dht6.readHumidity();
	
	sensorsOnline = true;	// Set sensors on line flag. Readings are valid.
  }


/*  
  // Poll DS18B20 temperature sensors
void pollTemperatures() {

	// Serial.print(F("Requesting temperatures..."));
	sensors.setWaitForConversion(false);	// Async read
	sensors.requestTemperatures();
	// Serial.println(F("DONE"));

}

void getTemp(DeviceAddress deviceAddress, int zoneIndex) {

	zoneTemperature[zoneIndex] = sensors.getTempC(deviceAddress);
}
 */	

// sets every element of str to 0 (clears array)
void StrClear(char *str, char length)
{
    for (int i = 0; i < length; i++) {
        str[i] = 0;
    }
}

// searches for the string sfind in the string str
// returns 1 if string found
// returns 0 if string not found
char StrContains(char *str, char *sfind)
{
    char found = 0;
    char index = 0;
    char len;

    len = strlen(str);
    
    if (strlen(sfind) > len) {
        return 0;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                return 1;
            }
        }
        else {
            found = 0;
        }
        index++;
    }

    return 0;
}

/*
	Purpose: Control relays on Freetronics RELAY8 8-Channel relay driver shield.
	Accepts:	uint8_t &latchState = Reference to byte holding relay latch state.
				uint8_t latchValue = latch bit/s to be set or cleared.
				bool relayState = required relay state. true = On, false = Off.
*/

void relayCommand(uint8_t &latchState, uint8_t latchBits, bool relayState)
{

   if (relayState == true) {	// Turn On
		latchState = latchState | latchBits;	// OR new bit/s into latch.
	} else {
		int latchMask = 0xff ^ latchBits;	// Create bit mask that contains a 1 for every bit except the bit/s being set to 0.
		latchState = latchState & latchMask;	// Remove bit/s 
	}

	Wire.beginTransmission(I2C_ADDR);
	Wire.write(0x12);        // Select GPIOA
	Wire.write(latchState);  // Send value to bank A
	//  Wire.write(latchBits);  // Send value to bank A
	Wire.endTransmission();
}
