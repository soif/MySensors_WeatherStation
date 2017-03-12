// includes ------------------------------------------------------------------------------
#include <SPI.h>
#include <MySensor.h>	 
#include <Wire.h> 
#include <DHT.h>	
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <DallasTemperature.h>
#include <OneWire.h>

// Define --------------------------------------------------------------------------------
#define SERIAL_PRINT_DEBUG false //do we print serial

#define INFO_NAME "WeatherStation"
#define INFO_VERS "1.1"

#define GW_NODE_ID 201		// 255 for Auto
#define GW_REPEATER false	// repeater mode


#define PIN_RAIN_DIGITAL	3 
#define PIN_DHT				4
#define PIN_ONEWIRE			5 
#define PIN_BATTERY_SENSE	A0
#define PIN_RAIN_ANALOG		A1

#define CHILD_ID_HUM		0
#define CHILD_ID_TEMP		1
#define CHILD_ID_BARO		2
#define CHILD_ID_LIGHT		3
#define CHILD_ID_RAIN		4
// BMP085 & BH1750 wired to A4, A5

#define ALTITUDE			36.32	// meters above sealevel (see http://www.altitude.nu/)
#define SLEEP_TIME			75017	//75017 (every 1m 15s 17ms )
#define REPORT_ALL_COUNT	24		// force report every x cycles
#define BATTERY_MIN			800		// value when Battery is 3.3v

// Variables -----------------------------------------------------------------------------
boolean	metric			= true;
float	lastHum			= -1;
float	lastTemp		= -100;
float	lastBmpTemp		= -100;
float	lastDallasTemp	= -100;
float	lastPressure	= -1;
int		lastRainValue	= -1;
uint16_t	lastLux		= -1;

int lastBatteryPcnt		= 0;
int updateCount			= 0;

float batteryRatio		= 100.0 / (1023 - BATTERY_MIN);

// objects -------------------------------------------------------------------------------
MySensor				gw;
DHT						dht;
Adafruit_BMP085			bmp = Adafruit_BMP085();
BH1750					lightSensor;
OneWire					oneWire(PIN_ONEWIRE);
DallasTemperature		dallas(&oneWire);

MyMessage msgHum(		CHILD_ID_HUM,	V_HUM);
MyMessage msgTemp(		CHILD_ID_TEMP,	V_TEMP);
MyMessage msgPressure(	CHILD_ID_BARO,	V_PRESSURE);
MyMessage msgLux(		CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgRain(		CHILD_ID_RAIN,	V_TRIPPED);


// ###### Boot ###########################################################################
void bootLeds(){
	Serial.println("+++++++++++++");
	Serial.println(" Booting...");
	Serial.println("+++++++++++++");
	for (int i=0; i <=4; i++){
		digitalWrite(13, HIGH);
		delay(50);
		digitalWrite(13, LOW);
		delay(90);
	}
}

// ###### ReceiveMessage #################################################################
void receiveMessage(const MyMessage &message){
}


// ###### Setup ##########################################################################
void setup(){

	analogReference(INTERNAL);

	Serial.begin(115000);
	bootLeds();
		
	gw.begin(receiveMessage, GW_NODE_ID, GW_REPEATER);

	dht.setup(PIN_DHT); 
	bmp.begin();
	lightSensor.begin();
	pinMode(PIN_RAIN_DIGITAL, INPUT);

	dallas.begin();
	dallas.setWaitForConversion(false);
	
	gw.sendSketchInfo(INFO_NAME, INFO_VERS);
	gw.present(CHILD_ID_HUM, S_HUM);
	gw.present(CHILD_ID_TEMP, S_TEMP);
	gw.present(CHILD_ID_BARO, S_BARO);
	gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
	gw.present(CHILD_ID_RAIN, S_MOTION);
	
	metric = gw.getConfig().isMetric;
}


// ###### main Loop ######################################################################
void loop(){
	updateCount += 1;
	
	if (updateCount >= REPORT_ALL_COUNT) {
		lastHum = -1;

		lastTemp		= -100;
		lastBmpTemp		= -100;
		lastDallasTemp	= -100;
	
		lastPressure	= -1;
		lastLux			= -1;
		lastRainValue	= -1;
		lastBatteryPcnt	= -1;
		updateCount		= 0;
	}
	
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
			//gw.send(msgTemp.set(temperature,1));		
		}
	} 
	
	if (! isnan(humidity)) {
		humidity= (int) (humidity + 0.5); // round to integer
		if (humidity != lastHum) {
			lastHum = humidity;
			gw.send(msgHum.set(humidity,0));
		}
	}


	// BMP Pressure (& temp) Sensor ---------------------------
	float pressure = bmp.readSealevelPressure(ALTITUDE) * 0.01;
	if (! isnan(pressure)) {
		pressure= (int) (pressure + 0.5); // round to integer
		if (pressure != lastPressure) {
			gw.send(msgPressure.set(pressure, 0));
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
			//gw.send(msgTemp.set(bmptemp,1));
			lastBmpTemp = bmptemp;
		}
	}

	
	// Light Sensor -----------------------------------------
	uint16_t lux = lightSensor.readLightLevel();
	if (lux != lastLux) {
		gw.send(msgLux.set(lux));
		lastLux = lux;
	}
	
	
	// Rain Sensor -------------------------------------------
	int rainValue = !(digitalRead(PIN_RAIN_DIGITAL));
	if (rainValue != lastRainValue) {
		gw.send(msgRain.set(rainValue));
		lastRainValue = rainValue;
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
			gw.send(msgTemp.set(dallasTemp, 1));
			lastDallasTemp = dallasTemp;
		}
	}


	// Battery ------------------------------------------------
	int sensorValue = analogRead(PIN_BATTERY_SENSE);
	int batteryPcnt = (sensorValue - BATTERY_MIN) * batteryRatio;
	if (lastBatteryPcnt != batteryPcnt) {
		gw.sendBatteryLevel(batteryPcnt);
		lastBatteryPcnt = batteryPcnt;
	}


	// Debug ------------------------------------
	if(SERIAL_PRINT_DEBUG){
		Serial.println("#########################################");
		Serial.print("# DHT Humidity    : "); Serial.println(humidity);
		Serial.print("# DHT Temperature : "); Serial.println(temperature);
		Serial.print("# Dallas Temp     : "); Serial.println(dallasTemp);
		Serial.print("# Baro Temp       : "); Serial.println(bmptemp);
		Serial.print("# Baro Pressure   : "); Serial.println(pressure);
		Serial.print("# Lux             : "); Serial.println(lux);
		Serial.print("# Rain            : "); Serial.println(rainValue);
		Serial.print("# Rain Analog     : "); Serial.println(analogRead(PIN_RAIN_ANALOG));
		Serial.print("# Battery         : "); Serial.print(sensorValue); Serial.print(" (");Serial.print(batteryPcnt); ;Serial.println(" %)");
		Serial.println("#########################################");
	}

	gw.sleep(SLEEP_TIME);
}


