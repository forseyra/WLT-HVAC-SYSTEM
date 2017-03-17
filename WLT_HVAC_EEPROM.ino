#define SKETCH_VERSION "WLT_HVAC_EEPROM 1.4.0"	
#define SKETCH_DESCRIPTION "WLT_HVAC_EEPROM.ino	Version 1.4.0	10 March 2017	Original rforsey@tpg.com.au"

/*
	Arduino-based multi-zone HVAC (Heating, Cooling, Ventilation and Air Conditioning) system with web-based control,
	configuration and monitoring user interface.
	
	Initialises data stored in Arduino EEPROM by WLT_HVAC.ino

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


/* WLT_HVAC_EEPROM.ino	Original Roger Forsey

	5 May 2014 Original
	V1.0 9 February 2015 added schedule arrays
	V1.1 1 March 2015 added additional hysteresis settings
	V1.2 30 January 2016 Added zoneVentilateArray
	V1.3 14 February 2017 Enabled Foyer Ventilate
	V1.4 10 March 2017 Updated inital settings and added run control dialogue

	Purpose: Initialises data stored in Arduino EEPROM by WLT_HVAC.ino
	
		Run this once to prep Arduino EEPROM before uploading WLT_HVAC.ino
*/	
	
	#include <Time.h>
	#include <TimeLib.h>
	#include <EEPROMex.h>

	#include <Time.h>        //http://www.arduino.cc/playground/Code/Time
	#include <Timezone.h>    //https://github.com/JChristensen/Timezone

	
	const int maxAllowedWrites = 400;
	String strInput;	// String for user keyboard responce
	
	//Australia Eastern Time Zone (Sydney, Melbourne)
	TimeChangeRule aEDT = {"AEDT", First, Sun, Oct, 2, 660};    //UTC + 11 hours
	TimeChangeRule aEST = {"AEST", First, Sun, Apr, 3, 600};    //UTC + 10 hours
	Timezone ausET(aEDT, aEST);


	int address = 0;	// Base address for data in EEPROM

	const int g_numberOfZones = 6;  // Number of A/C sensor zones.
	const int scheduleHours = 24; // size of each days array.
	const int numSchedulePressButtons = 2;	// Size of press buttons array.
	
	float zoneTemperature[] = {0, 0, 0, 0, 0, 0};	// Zone Temperature
	float zoneHumidity[] = {0, 0, 0, 0, 0, 0};	// Zone Humidity
	
	// Arrays for zone thermostat setting and hysteresis
	float zoneThermostatSetting[] = {20, 20, 20, 20, 20, 20};	// Set point On temperature
	float zoneThermostatHysteresis[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; //Difference in degrees between set temperature & on & off settings.
	
	float zoneCoolThermostatSetting[] = {23, 23, 23, 20, 23, 23};	// Cool On temperature
	float zoneCoolThermostatHysteresis[] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; //Difference in degrees between set temperature & on & off settings.
	
	float zoneMaxHumidity[] = {70, 70, 70, 70, 70, 70};	// Maximum humidity in zone for evaporative cooling pump control.
	float zoneMaxHumidityHysteresis[] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; //Difference in % humidity between set on & off settings.	
	
	float zoneCoolHighFanPoint[] = {26, 26, 26, 26, 26, 26};	// Run high fan speed if zone temperature above this point.
	float zoneCoolHighFanHysteresis[] = {0.5, 0.5, 0.5, 2, 0.5, 0.5}; 
		

	// Weekly schedule arrays. Hour 0 - 23, true = on, false = off
	byte schSunday[24] = { 0 };
	byte schMonday[24] = { 0 };
	byte schTuesday[24] = { 0 };
	byte schWednesday[24] = { 0 };
	byte schThursday[24] = { 0 };
	byte schFriday[24] = { 0 };
	byte schSaturday[24] = { 0 };
	
	byte weeklyScheduleArray[] = {0, 0};	// Weekly Schedule enabled = 1
	
	byte zoneVentilateArray[] = {0, 2, 0, 2, 2, 2};	// Dehumidity Zone. 0 = Off, 1 = On, 2 = N/A (Disabled)

	float readBuffer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 
void setup() {

byte kbResponce = 3;
	
	// Open serial communications and wait for port to open:
	Serial.begin(57600);
	Serial.println();
	Serial.println(SKETCH_DESCRIPTION);
	Serial.println("Purpose: Initialises data stored in Arduino EEPROM by WLT_HVAC.ino");
	Serial.println();
	Serial.println("Run this once to initialise default setup in Arduino EEPROM before uploading WLT_HVAC.ino");
	Serial.println();
	Serial.println ("Enter yes to continue or no to stop"); 
	Serial.flush(); 

	kbResponce = 3;
	
	// Wait loop for yes
	while (kbResponce > 0) {
		kbResponce = enterYesNo();	// Wait for a yes responce from user

	}
	
	Serial.println();
	Serial.print ("Writing to EEPROM starting at address = ");
	Serial.println(address);
	
 
	EEPROM.setMaxAllowedWrites(maxAllowedWrites);
	
    Serial.println("-----------------------------------------------------");    
    Serial.println("Setting EEPROM initial values");    
    Serial.println("-----------------------------------------------------"); 
	
	// EEPROM Memory allocation
	
	// Always get the adresses first and in the same order
    /* int addressByte      = EEPROM.getAddress(sizeof(byte));
    int addressInt       = EEPROM.getAddress(sizeof(int));
    int addressLong      = EEPROM.getAddress(sizeof(long));
    int addressFloat     = EEPROM.getAddress(sizeof(float));
    int addressDouble    = EEPROM.getAddress(sizeof(double));    
    int addressByteArray = EEPROM.getAddress(sizeof(byte)*7);  
    int addressCharArray = EEPROM.getAddress(sizeof(char)*7); 
		*/
		
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
 
	
	// Print starting address and size.
	
	Serial.println("begin address \t\t size");
	
	Serial.print(addressAusET); Serial.print(" \t\t\t "); Serial.print(sizeof(aEDT) + sizeof(aEST)); Serial.println(" (ausET)"); 
	Serial.print(addressLastModifiedTime); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (LastModifiedTime)"); 

	Serial.print(addressZoneThermostatSetting); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneThermostatSetting)"); 
	Serial.print(addressZoneThermostatHysteresis); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneThermostatHysteresis)");
	
	Serial.print(addressZoneCoolThermostatSetting); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneCoolThermostatSetting)"); 
	Serial.print(addressZoneCoolThermostatHysteresis); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneCoolThermostatHysteresis)"); 
		
	Serial.print(addressZoneMaxHumidity); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneMaxHumidity)"); 
	Serial.print(addressZoneMaxHumidityHysteresis); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneMaxHumidityHysteresis)"); 
	
	Serial.print(addressZoneCoolHighFanPoint); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneCoolHighFanPoint)");
	Serial.print(addressZoneCoolHighFanHysteresis); Serial.print(" \t\t\t "); Serial.print(sizeof(float)* g_numberOfZones); Serial.println(" (ZoneCoolHighFanHysteresis)");

	Serial.print(addressSchSunday); Serial.print(" \t\t\t "); Serial.print(sizeof schSunday / sizeof *schSunday); Serial.println(" (schSunday)"); 
	Serial.print(addressSchMonday); Serial.print(" \t\t\t "); Serial.print(sizeof schMonday / sizeof *schSunday); Serial.println(" (schMonday)"); 
	Serial.print(addressSchTuesday); Serial.print(" \t\t\t "); Serial.print(sizeof schTuesday / sizeof *schTuesday); Serial.println(" (schTuesday)"); 
	Serial.print(addressSchWednesday); Serial.print(" \t\t\t "); Serial.print(sizeof schWednesday / sizeof *schWednesday); Serial.println(" (schWednesday)"); 
	Serial.print(addressSchThursday); Serial.print(" \t\t\t "); Serial.print(sizeof schThursday / sizeof *schThursday); Serial.println(" (schThursday)"); 
	Serial.print(addressSchFriday); Serial.print(" \t\t\t "); Serial.print(sizeof schFriday / sizeof *schFriday); Serial.println(" (schFriday)"); 
	Serial.print(addressSchSaturday); Serial.print(" \t\t\t "); Serial.print(sizeof schSaturday / sizeof *schSaturday); Serial.println(" (schSaturday)"); 

	Serial.print(addressWeeklyScheduleArray); Serial.print(" \t\t\t "); Serial.print(sizeof weeklyScheduleArray / sizeof *weeklyScheduleArray); Serial.println(" (weeklyScheduleArray)"); 
	Serial.print(addressZoneVentilateArray); Serial.print(" \t\t\t "); Serial.print(sizeof zoneVentilateArray / sizeof *zoneVentilateArray); Serial.println(" (zoneVentilateArray)"); 
	
	
	// Write data to EEPROM.
	
	ausET.writeRules(0);    //Write timezone rules to EEPROM address 0
	
	EEPROM.writeLong(addressLastModifiedTime, 0) ;

	EEPROM.writeBlock<float>(addressZoneThermostatSetting, zoneThermostatSetting, g_numberOfZones);
	EEPROM.writeBlock<float>(addressZoneThermostatHysteresis, zoneThermostatHysteresis, g_numberOfZones);
	
	EEPROM.writeBlock<float>(addressZoneCoolThermostatSetting, zoneCoolThermostatSetting, g_numberOfZones);
	EEPROM.writeBlock<float>(addressZoneCoolThermostatHysteresis, zoneCoolThermostatHysteresis, g_numberOfZones);
	
	EEPROM.writeBlock<float>(addressZoneMaxHumidity, zoneMaxHumidity, g_numberOfZones);
	EEPROM.writeBlock<float>(addressZoneMaxHumidityHysteresis, zoneMaxHumidityHysteresis, g_numberOfZones);
	
	EEPROM.writeBlock<float>(addressZoneCoolHighFanPoint, zoneCoolHighFanPoint, g_numberOfZones);
	EEPROM.writeBlock<float>(addressZoneCoolHighFanHysteresis, zoneCoolHighFanHysteresis, g_numberOfZones);

	EEPROM.writeBlock<byte>(addressSchSunday, schSunday, scheduleHours);
	EEPROM.writeBlock<byte>(addressSchMonday, schMonday, scheduleHours);
	EEPROM.writeBlock<byte>(addressSchTuesday, schTuesday, scheduleHours);
	EEPROM.writeBlock<byte>(addressSchWednesday, schWednesday, scheduleHours);
	EEPROM.writeBlock<byte>(addressSchThursday, schThursday, scheduleHours);
	EEPROM.writeBlock<byte>(addressSchFriday, schFriday, scheduleHours);
	EEPROM.writeBlock<byte>(addressSchSaturday, schSaturday, scheduleHours);
	
	EEPROM.writeBlock<byte>(addressWeeklyScheduleArray, weeklyScheduleArray, numSchedulePressButtons);
	EEPROM.writeBlock<byte>(addressZoneVentilateArray, zoneVentilateArray, g_numberOfZones);
	

	Serial.println();
	Serial.print("Finished Writing");
	Serial.println();
	Serial.println("EEPROM Test read");
	
	EEPROM.readBlock<float>(addressZoneCoolThermostatSetting, readBuffer, g_numberOfZones);			
	
	Serial.println("Read ZoneCoolThermostatSetting");
	printFloatArray(readBuffer, g_numberOfZones);
	Serial.println();
	
}

void loop() { }

void printFloatArray(float floatArray[], int numberOfItems) {

	Serial.println();
 
	for (int i = 0; i < numberOfItems; i++) {
		Serial.print(floatArray[i]);
		Serial.print(" ");
	}
	Serial.println();
}	

byte enterYesNo()	{
 
	String strInput = Serial.readStringUntil('\n');

	// Check if null string returned. This seems to be an undocumented feature of readStringUntil()
	if (strInput != "")	{
	
		Serial.println(strInput);	// Echo input string.
		
		if (strInput == "yes")	{
			return 0;
			
		}
		else if (strInput == "no")	{
			return 1;
		}
		else {

			return 2;
		}
	}
}




	
	
	
