/*
	MySensors Weather Station
	https://github.com/soif/MySensors_WeatherStation
	Copyright 2014 François Déchery

	** Description **
	This Arduino ProMini (3.3V) based project is a MySensors node  solar powered node which reports external temperature, luminosity, humidity, barometric pressure and rain condition. 

	** Compilation **
		- needs MySensors version 2.1+
*/

// debug #################################################################################
#define OWN_DEBUG	// Comment/uncomment to remove/show debug (May overflow Arduino memory when set)
#define MY_DEBUG	// Comment/uncomment to remove/show MySensors debug messages (May overflow Arduino memory when set)

// Define ################################################################################
#define INFO_NAME "WeatherStation"
#define INFO_VERS "2.1.0"

// MySensors
#define MY_RADIO_NRF24
#define MY_NODE_ID 201
#define MY_TRANSPORT_WAIT_READY_MS 5000	//set how long to wait for transport ready. in milliseconds
//#define MY_TRANSPORT_SANITY_CHECK
//#define MY_REPEATER_FEATURE

#define CHILD_ID_HUM		0
#define CHILD_ID_TEMP		1
#define CHILD_ID_BARO		2
#define CHILD_ID_LIGHT		3
#define CHILD_ID_RAIN_D		4
#define CHILD_ID_RAIN_A		5

#define ALTITUDE			36.32	// meters above sealevel (see http://www.altitude.nu/)
#define SLEEP_TIME			75017	// read sensors every 75017 ms (every 1m 15s 17ms )
#define REPORT_ALL_COUNT	24		// force report every x cycles
#define BATTERY_MIN			800		// value when Battery is 3.3v

#define NUM_READS			100		// time to read analog values
#define RAIN_MIN			250		// minimum analog rain value (when dry)

// Pins ##################################################################################
#define PIN_RAIN_DIGITAL	3 
#define PIN_DHT				4
#define PIN_ONEWIRE			5 
#define PIN_BATTERY_SENSE	A0
#define PIN_RAIN_ANALOG		A1

// BMP085 & BH1750 wired to A4, A5

// includes ##############################################################################
#include "debug.h"
#include <SPI.h>
#include <MySensors.h>
#include <Wire.h> 
#include <DHT.h>	
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <DallasTemperature.h>
#include <OneWire.h>


// Variables #############################################################################
boolean	metric			= true;
float		lastHum				= -1;
float		lastTemp			= -100;
float		lastBmpTemp			= -100;
float		lastDallasTemp		= -100;
float		lastPressure		= -1;
int			lastDigitalRain		= -1;
int 		lastAnalogRainPcnt	= 0;
uint16_t	lastLux				= -1;
int 		lastBatteryPcnt		= 0;
int 		updateCount			= 0;

//float		batteryRatio	= 100.0 / (1023 - BATTERY_MIN);
float		batteryVal		=0;
int			batteryPcnt		=0;

float		analogRain		=0;
int			analogRainPcnt	=0;

// objects --------
DHT						dht;
Adafruit_BMP085			bmp = Adafruit_BMP085();
BH1750					lightSensor;
OneWire					oneWire(PIN_ONEWIRE);
DallasTemperature		dallas(&oneWire);

MyMessage msgHum(		CHILD_ID_HUM,	V_HUM);
MyMessage msgTemp(		CHILD_ID_TEMP,	V_TEMP);
MyMessage msgPressure(	CHILD_ID_BARO,	V_PRESSURE);
MyMessage msgLux(		CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgRainD(		CHILD_ID_RAIN_D,	V_TRIPPED);
MyMessage msgRainA(		CHILD_ID_RAIN_A,	V_RAIN);


// ###### Setup ##########################################################################
void setup(){
	DEBUG_PRINTLN("_setup START");

	Serial.begin(115000);
	bootLeds();


	dht.setup(PIN_DHT); 
	bmp.begin();
	lightSensor.begin();

	pinMode(PIN_RAIN_DIGITAL, INPUT);
	pinMode(PIN_RAIN_ANALOG, INPUT);
	pinMode(PIN_BATTERY_SENSE, INPUT);

	dallas.begin();
	dallas.setWaitForConversion(false);
	
	DEBUG_PRINTLN("_setup END");
	
}


// ###### main Loop ######################################################################
void loop(){
	updateCount += 1;

	if (updateCount >= REPORT_ALL_COUNT) {
		lastHum = -1;

		lastTemp			= -100;
		lastBmpTemp			= -100;
		lastDallasTemp		= -100;
	
		lastPressure		= -1;
		lastLux				= -1;
		lastDigitalRain		= -1;
		lastBatteryPcnt		= -1;
		lastAnalogRainPcnt	= -1;
		updateCount			= 0;
		DEBUG_PRINTLN("");
		DEBUG_PRINTLN("--> Forcing report... --------------------");
		DEBUG_PRINTLN("");
	}

	ReadBattery();

	
	// dht (Temps + Hum) sensor ------------------------------
	delay(dht.getMinimumSamplingPeriod());
	float temperature	= dht.getTemperature();
	float humidity		= dht.getHumidity();
	
	if (! isnan(temperature)) {
		temperature = ( (int) (temperature * 10 ) )/ 10.0 ; //rounded to 1 dec
		if (temperature != lastTemp) {
			lastTemp = temperature;
			if (!metric) {
				temperature = temperature * 1.8 + 32.0;
			}
			//send(msgTemp.set(temperature,1));		
		}
	} 
	
	if (! isnan(humidity)) {
		humidity= (int) (humidity + 0.5); // round to integer
		if (humidity != lastHum) {
			lastHum = humidity;
			send(msgHum.set(humidity,0));
		}
	}


	// Dallas 1820 Sensor ------------------------------------
	dallas.requestTemperatures(); // Send the command to get temperatures
	delay( dallas.millisToWaitForConversion(dallas.getResolution()) ); // make sure we get the latest temps

	float dallasTemp = dallas.getTempCByIndex(0);
	if (! isnan(dallasTemp)) {
		dallasTemp = ( (int) (dallasTemp * 10 ) )/ 10.0 ; //rounded to 1 dec
		if (dallasTemp != lastDallasTemp	&& dallasTemp != -127.00 && dallasTemp != 85.0) {
			if (!metric) {
				dallasTemp = dallasTemp * 1.8 + 32.0;
			}
			send(msgTemp.set(dallasTemp, 1));
			lastDallasTemp = dallasTemp;
		}
	}


	// BMP Pressure (& temp) Sensor ---------------------------
	float pressure = bmp.readSealevelPressure(ALTITUDE) * 0.01;
	if (! isnan(pressure)) {
		pressure= (int) (pressure + 0.5); // round to integer
		if (pressure != lastPressure) {
			send(msgPressure.set(pressure, 0));
			lastPressure = pressure;
		}
	}

	float bmptemp = bmp.readTemperature();	
	if (! isnan(bmptemp)) {
		bmptemp = ( (int) (bmptemp * 10 ) )/ 10.0 ; //rounded to 1 dec
		if (bmptemp != lastBmpTemp) {
			if (!metric) {
				bmptemp = bmptemp * 1.8 + 32.0;
			}
			//send(msgTemp.set(bmptemp,1));
			lastBmpTemp = bmptemp;
		}
	}

	
	// Light Sensor -----------------------------------------
	uint16_t lux = lightSensor.readLightLevel();
	if (lux != lastLux) {
		send(msgLux.set(lux));
		lastLux = lux;
	}
	
	
	// Rain Sensor -------------------------------------------
	int digitalRain = !(digitalRead(PIN_RAIN_DIGITAL));
	if (digitalRain != lastDigitalRain) {
		send(msgRainD.set(digitalRain));
		lastDigitalRain = digitalRain;
	}
	ReadAnalogRain();


	//debug ----------------------------------------------------
	DEBUG_PRINTLN("#########################################");
	DEBUG_PRINT("# DHT Humidity    : "); DEBUG_PRINTLN(humidity);
	DEBUG_PRINT("# DHT Temperature : "); DEBUG_PRINTLN(temperature);
	DEBUG_PRINT("# Dallas Temp     : "); DEBUG_PRINTLN(dallasTemp);
	DEBUG_PRINT("# Baro Temp       : "); DEBUG_PRINTLN(bmptemp);
	DEBUG_PRINT("# Baro Pressure   : "); DEBUG_PRINTLN(pressure);
	DEBUG_PRINT("# Lux             : "); DEBUG_PRINTLN(lux);
	DEBUG_PRINT("# Rain            : "); DEBUG_PRINTLN(digitalRain);
	DEBUG_PRINT("# Rain Analog     : "); DEBUG_PRINT(analogRain); DEBUG_PRINT(" (");DEBUG_PRINT(analogRainPcnt); ;DEBUG_PRINTLN(" %)");
	DEBUG_PRINT("# Battery         : "); DEBUG_PRINT(batteryVal); DEBUG_PRINT(" (");DEBUG_PRINT(batteryPcnt); ;DEBUG_PRINTLN(" %)");
	DEBUG_PRINTLN("#########################################");

	sleep(SLEEP_TIME);
}

// #######################################################################################

// --------------------------------------------------------------------
void ReadBattery(){
	analogReference(INTERNAL);
	analogRead(PIN_BATTERY_SENSE);
	wait(5);
	batteryVal=0;
	for(int i=0;i< NUM_READS;i++){
      batteryVal += analogRead(PIN_BATTERY_SENSE);    
      wait(2);
    }
	batteryVal = ceil(batteryVal / NUM_READS);
	//batteryPcnt = (batteryVal - BATTERY_MIN) * batteryRatio;
	batteryPcnt	=map(batteryVal, BATTERY_MIN,1023, 0,100);
	
	if (lastBatteryPcnt != batteryPcnt) {
		sendBatteryLevel(batteryPcnt);
		lastBatteryPcnt = batteryPcnt;
	}
}

// --------------------------------------------------------------------
void ReadAnalogRain(){
	analogReference(DEFAULT);
	analogRead(PIN_RAIN_ANALOG); 
	wait(5);
	analogRain=0;
	for(int i=0;i< NUM_READS;i++){
      analogRain += analogRead(PIN_RAIN_ANALOG);    
      wait(2);
    }
	analogRain 		= ceil(analogRain / NUM_READS);
	analogRainPcnt	= map(analogRain, RAIN_MIN,1023, 100,0);

	if (lastAnalogRainPcnt != analogRainPcnt) {
		send(msgRainA.set(analogRainPcnt));
		lastAnalogRainPcnt = analogRainPcnt;
	}

}


// --------------------------------------------------------------------
void presentation(){
	DEBUG_PRINTLN("_presentation START");
	sendSketchInfo(INFO_NAME , INFO_VERS );

	present(CHILD_ID_HUM,		S_HUM);
	present(CHILD_ID_TEMP,		S_TEMP);
	present(CHILD_ID_BARO,		S_BARO);
	present(CHILD_ID_LIGHT,		S_LIGHT_LEVEL);
	present(CHILD_ID_RAIN_D,	S_MOTION);
	present(CHILD_ID_RAIN_A,	S_RAIN);

	DEBUG_PRINTLN("_presentation END");
}

// --------------------------------------------------------------------
void receive(const MyMessage &msg){
}

// --------------------------------------------------------------------
void bootLeds(){
	DEBUG_PRINTLN("+++++++++++++");
	DEBUG_PRINTLN(" Booting...");
	DEBUG_PRINTLN("+++++++++++++");
	for (int i=0; i <=4; i++){
		digitalWrite(13, HIGH);
		delay(50);
		digitalWrite(13, LOW);
		delay(90);
	}
}

