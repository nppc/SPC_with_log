/* created on 04/08/2014
   by nppc

Sepecifications for PB (SLA) battery
	2.15v - minimum charging voltage for the cell (6 cells = 12.9v)
	2.35v - safe charging voltage for the cell (6 cells = 14.1v) 
	2.5v - maximum charging (rapid charge at 25t deg) voltage for the cell (6 cells = 15v)
	These batteries can be very long lived if they are charged at a float voltage
	2.25 to 2.3 volts per cell - float voltage (13.5V to 13.8V for a 12V battery). 
	Normal Charging algorythm in 20-25t deg (3 stages):
		1. CC charge at 0.25C rate. Charge until voltage reaches 14.4V (2.40V/cell).
		2. CV charge holds reached volage, sensing charge current. 
		   When the charge current drops to 0.05C Amps, 
		   the battery will have recovered approximately 70-80% of its charge. 
		   At this point output voltage must be reduced to 13.65V (2.275V/cell).
		   The remaining 20-30% of the charge is carried out at this lower voltage 
		   in order to prevent over-charge. Stay in this mode until fully charged.
		3. Switch to float charge.
		For the best performance these voltages will need to be temperature compensated 
		by approximately 4mV/°C/Cell (reduced at higher temperatures and increased at lower temperatures).
		
		Full charge is reached when the current decreases to the 3% level of the rated current 
		(or by timer for old batteries, not more than 48h in CV stage).

		2.10v / storage cell voltage (12.6v)
		
	Solar charging algorytm. It differs from normal, because we don't have reliable charge suppy.
		1. Do charge at max possible CC that not exceeds 0.25C rate. Charge until voltage reaches 14.4V (2.40V/cell).
		   This stage can start anytime as batterz voltage goes lower than 13.5v (float charge).
		2. Switch to CV as soon as voltage reaches 14.4V (still chacking that current will not jump over 0.25C).
		   If Current will jump over then switch back to 1 stage.
		3. When the charge current drops to 0.05C Amps, output voltage must be reduced to 13.65V (2.275V/cell).
		   Once in a hour measure voltage of battery without charge. Monitor it during 2 minutes.
		   If voltage without charge will drop significaly, then concider battery as dead.

	Percentage of charging vs voltage under the charge (per 1 cell):
	         linear  (non linear for C/10 charge)   
	    0% - 2000 mV (2000)
	   10% - 2048 mV (2067)
	   30% - 2145 mV (2133)
	   50% - 2242 mV (2217)
	   70% - 2338 mV (2233)
	   90% - 2435 mV (2350)
	  100% - 2483 mV (2533)

Analog buttons:
	2 Buttons connected to the ADC for pin saving.
		Divider is 1:10 and 10:10 (4.5V, 2.5V).
		Button 1 - Turn ON/OFF LCD. LCD automatically will switch off after 10 min.
 */
#include <Metro.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include "lcd.h"

// battery type
#define BATTERY_CELLS 6
//const byte BATTERY_CELLS=6;
#define BATTERY_MAH 12000

// voltages are in mV at 25t celsium for the 1 cell
//const int C_CHARGEVOLTAGE_MIN = 2150;  // Minimum voltage of the Solar Panel for starting charge
const int C_CHARGEVOLTAGE_CYCLE1 = 2400;  // Maximum voltage while Cycle charge
const int C_CHARGECURRENT_CYCLE1 = 25;    // Maximum Current while Cycle charge (% of C)
const int C_CHARGEVOLTAGE_CYCLE2 = 2275;  // (90% charged) Voltage to charge after current drops below 5% of C
const int C_CHARGECURRENT_CYCLE2 = 5;     // Current when need to switch to Cycle charge Stage 2 (voltage drops to 2275) (% of C)
const int C_CHARGEVOLTAGE_FLOAT = 2250;   // (100% charged) Float charge voltage
const int C_CHARGECURRENT_FLOAT = 2;     // Current when need to switch to Float charge Stage 3 (% of C)
// voltage indicating charge level of battery under the charge proccess
const int C_CHARGEPERCENT_0 = 2000;
const int C_CHARGEPERCENT_10 = 2067;
const int C_CHARGEPERCENT_30 = 2133;
const int C_CHARGEPERCENT_50 = 2217;
const int C_CHARGEPERCENT_70 = 2333;
//const int C_CHARGEPERCENT_90 = 2350;
//const int C_CHARGEPERCENT_100 = 2533;
const int C_CHARGEPERCENT_OPEN_0 = 1850;	// 11,1V
const int C_CHARGEPERCENT_OPEN_10 = 1900;
const int C_CHARGEPERCENT_OPEN_30 = 2000;
const int C_CHARGEPERCENT_OPEN_50 = 2030;
const int C_CHARGEPERCENT_OPEN_70 = 2062;
const int C_CHARGEPERCENT_OPEN_90 = 2100;
const int C_CHARGEPERCENT_OPEN_100 = 2117;	// 12,7v

// current in mAh, voltage in mV
unsigned int cycle1ChargeBat_voltage;	// charging voltage in Cycle1 Charge mode for Battery (6 cells)
unsigned int cycle2ChargeBat_voltage;	// charging voltage in Cycle2 Charge mode for Battery (6 cells)
unsigned int floatChargeBat_voltage;	// charging voltage in Float Charge mode for Battery (6 cells)
unsigned int cycle1ChargeBat_current;	// calculated  (BATTERY_MAH * C_CHARGECURRENT_CYCLE1 / 100)
unsigned int cycle2ChargeBat_current;    // calculated  (BATTERY_MAH * C_CHARGECURRENT_CYCLE2 / 100)
unsigned int floatChargeBat_current;     // calculated  (BATTERY_MAH * C_CHARGECURRENT_FLOAT / 100)

#define floatChargeCellTemperatureAdj 2			// value multiplied by 1000. This value need to substract from nominal voltage with everz 1t deg increasing
#define cycleChargeCellTemperatureAdj 7			// value multiplied by 1000. This value need to substract from nominal voltage with everz 1t deg increasing

// voltages in mV
//TODO lets try to convert this variables to unsigned long...
unsigned long solar_voltage;  // calculated voltage for the Solar Panel
unsigned long solar_currentS1_voltage; // calculated voltage for upper CurrentSense resistor connection
unsigned long solar_currentS2_voltage; // calculated voltage for lower CurrentSense resistor connection. Also it is a battery voltage
unsigned long solar_current;	// in mA
unsigned long solar_power;	// in mW
float ambient_Temperature;	// temperature will be read by DS sensor
unsigned long AccumulatedWatts=0;		// for accumulating mWatts during a second
//unsigned long AccumulatedAmpers=0;		// for accumulating mAmpers during a second
unsigned int AccumulatingCounter=0;		// Counter for averaging values
byte everySecondWatts_storage[60];		// last 60 seconds data
byte counterSecondWatts_storage = 0;	// array pointer
byte everySecondWattsMax = 0;			// max value for the Graph (upper value)
unsigned long everySecondWattsAccumulated=0;
byte everyMinuteWatts_storage[60];		// last hour data
byte counterMinuteWatts_storage = 0;
unsigned long everyMinuteWattsAccumulated=0;
byte everyHourWatts_storage[24];		// last 24 hours data
byte counterHourWatts_storage = 0;
unsigned long everyHourWattsAccumulated=0;
byte everyDayWatts_storage[30];			// last 30 days data
byte counterDayWatts_storage = 0;
unsigned long everyDayWattsAccumulated=0;
byte everyMonthWatts_storage[12];			// last 12 months data
byte counterMonthWatts_storage = 0;
unsigned long everyMonthWattsAccumulated=0;

//float KWattsHour=0;				// (Watt) Calculated mAmpers in a hour (updates everz second)
int sample1;     // reading from Arduino analog pin
int sample2;     // reading from Arduino analog pin
int sample3;     // reading from Arduino analog pin
byte requestTemperature_flag=0;		// If 1 then temperature will be requested from DS sensor. If 2 then temperature will be read from sensor.
byte chargingStage_step=3; // 1 - cycle charge, 2 - float charge, 3 - Trickle charge
byte screenNr_LCD = 1;	// currently displayed screen. Last bit is used for changing an LCD screen.
const byte SCREEN_LCD_TOTAL = 3;	// total LCD screens

// Arduino controller pin definitions
#define CHARGEMOSFET_PIN 6           // pwm output to mosfet
#define LOAD_PIN 9          //load to the battery
#define BACKLIGHT_PIN 13  // pin 13 will control the backlight of LCD
#define RED_PIN 7           // To indicate discharged condition of battery
#define GREEN_PIN 8         // for charging and battery fully charged
#define SOLARVOLTAGE_PIN A0  // Pin for measuring Solar Panel voltage (also upper leg of the current sense)
#define SOLARCURRENTS1_PIN A1  // Pin for measuring current flow from Solar Panel to the battery 
#define SOLARCURRENTS2_PIN A2  // Pin for measuring current flow from Solar Panel to the battery (also this is battery voltage sense pin)
#define BUTTONS_PIN A3         // Pin reading analog values from buttons connected as a resistor voltage devider.
#define ONE_WIRE_BUS_PIN 11			// DS18B20

// different constants
#define filterSamples   13              // filterSamples should  be an odd number, no smaller than 3
const float shuntR=0.55;  // resistance for measuring current flow

int charged_percent =0; 
int pwmout=0;	// variable for the PWM charging output
byte buttonAnalogReadCntr = 0;	// here we count stable reads of analog pin
byte buttonReturnedValue = 0;	// here we store a returned value of a button pressed
int buttonAnalogPrevVal = 512;	// some value read last
unsigned long lcdOnTime;	// storage for the timer to decide when LCD can go off
unsigned long loadOnOffTime;	// storeage for the timer for load On/Off (do avoid rapid switches)
boolean backlight_LCD = HIGH;	// status of the LCD backlight. Initially it is ON
boolean load_RELAY = LOW;	// status of the LCD backlight. Initially it is OFF

// voltage deviders for voltage/current sense
const float VdevSolarBat = (float)270 / (float)(1000 + 270); // 100.0 kOhm; 27.0 kOhm
const float VdevSolar1 = (float)270 / (float)(1000 + 270); // 100.0 kOhm; 27.0 kOhm
const float VdevSolar2 = (float)270 / (float)(1000 + 270); // 100.0 kOhm; 27.0 kOhm

OneWire oneWire(ONE_WIRE_BUS_PIN);// Init OneWire

Metro doChargeTask = Metro(20);
Metro eachSecondTask = Metro(1000);		// task for updating LCD, read temperature etc.
Metro eachMinuteTask = Metro(60000);	// different tasks that need to be runned every minute.
//Metro switchLoad = Metro(5000);

LiquidCrystal lcd(12, 10, 5, 4, 3, 2);	// definition for the LCD 2X16
byte customChar_LCD[8];	// buffer for custom LCD char 

int solar_voltage_SmoothArray [filterSamples];   // array for holding raw sensor values for Pressure sensor 
int solar_currentS1_SmoothArray [filterSamples];   // array for holding raw sensor values for Pressure sensor 
int solar_currentS2_SmoothArray [filterSamples];   // array for holding raw sensor values for Pressure sensor 

void setup()
{
  //TCCR0B = TCCR0B & 0b11111000 | 0x05; // setting prescaler for 61.03Hz pwm
  Serial.begin(9600);
  pinMode(CHARGEMOSFET_PIN,OUTPUT);
  pinMode(LOAD_PIN,OUTPUT);
  pinMode(RED_PIN,OUTPUT);
  pinMode(GREEN_PIN,OUTPUT);
  pinMode(BACKLIGHT_PIN, OUTPUT);
  pinMode(BUTTONS_PIN, INPUT_PULLUP);	// internal resistor is the part of a divider (about 20K)
  digitalWrite(CHARGEMOSFET_PIN,LOW);
  digitalWrite(LOAD_PIN,load_RELAY);
  digitalWrite(RED_PIN,LOW);
  digitalWrite(GREEN_PIN,LOW);
  digitalWrite(BACKLIGHT_PIN, backlight_LCD);       	//controls the backlight intensity 0-255
  lcd.begin(16,2);                     // columns, rows. size of display
  lcd.clear();                         // clear the screen
  lcd.setCursor(0,0);
  lcd.print("Initializing...");
  // Update DigitalSmooth array and initialize variables
  for (int foo = 0; foo < 40; foo++)  {readVoltages();}
  requestOneWire_Temperature();
  delay(800);  
  readOneWire_Temperature();
  AdjustChargeVoltageAmbientTemperature();
  lcdOnTime = millis();		// after startup backlight will be on
  loadOnOffTime = millis();	// load relay will turn on only after 5 seconds
  lcd_loadCustomCharactersDefault();
  updateBatIconCharge();
 
  lcd.clear();
}
void loop()
{
  readVoltages();
  readButtons();
  changeLCDscreen();	// reads the LCD button and changes the screen if needed
  if (doChargeTask.check() == 1) {
    doCharge();	// charging logic here
  }

  if (eachSecondTask.check() == 1) {
  	// check do we need to request or to read a temperature
  	if (requestTemperature_flag==2) {
  		readOneWire_Temperature();
  		requestTemperature_flag=0;
	  	// temperature can change, so adjust voltages
	  	AdjustChargeVoltageAmbientTemperature();
  	}
  	if (requestTemperature_flag==1) {
  		requestOneWire_Temperature();
  		requestTemperature_flag=2;
  	}
	updateWattsCounters();
	updateLCD();
  }

  if (eachMinuteTask.check() == 1) {
  	requestTemperature_flag = 1;	// start requesting temperature. EachSecond task will take care of it.
  	// voltage adjusting is fired automatically after temperature read.
  	updateBatIconCharge();	// once per minute update battery charging icon.
  }
  
  switchLCDBacklightOnOff();
  accumulateWatts();	//this routine helps to get average of Watts.
  switchLoadOnOff();	// this routine controls the load RELAY
// end of main loop
}

// do charging. 2 stages / cycle charge or float charge
/*
		1. If bat voltage is less than 13.5v (2.25V/cell), do charge at max possible CC (0.25C) or with voltage 2.40V/cell.
			- Same time look for current drops below 0.05C, after that switch to the float charge.
		2. In float charge at 13.65V (2.275v/cell) time to time check is battery have less than 13.5v (2.25V/cell), then go to step 1.
			- if solar panel gives less than 2.15v/cell then switch off charging (stage 0) and wait for the panel gets power. 
*/
// DONE! if solar panel voltage drops, need to decrase PWM to have continuous chargiing.
// ?TODO: On stage 2 charge is more than 0.05C (right after solar panel is on). Seems need to switch to stage 1.
// DONE!: While switching to stage 3 voltage on battery goes to 15 volts.
// DONE!: charge % shows wrong on stage 3 and while charge is off (pwmout=0)
// ?TODO: showing of log is wrong.
void doCharge() {
	if (solar_voltage<=(solar_currentS2_voltage+1000)) {	// 1 volt for circuit drop
		// You can do what ever you want, but if solar gives not enough power, we can't charge.
		pwmout--;  // decrease power to maintain solar voltage up.
	} else {
		switch (chargingStage_step) {
			case 1: // Cycle - maximum charging rate
				// first check are we still allowed to stay on this stage
				if (solar_currentS2_voltage>cycle1ChargeBat_voltage || solar_current>cycle1ChargeBat_current) {	// Are we at max charge voltage or charge current?
					pwmout--;	// decrase charging power
				}
				if (solar_currentS2_voltage<cycle1ChargeBat_voltage && solar_current<cycle1ChargeBat_current) {	// Are we under max charge voltage and charge current?
					pwmout++;	// increase charging power
				}
				// check do we jump to stage 2?
				if (solar_currentS2_voltage>=cycle1ChargeBat_voltage && solar_current<=cycle2ChargeBat_current) {
					chargingStage_step = 2;
					Serial.println("Stage 2");
				}
				break;
			case 2:  // Second stage of cycling charge (voltage lowered)
				if (solar_currentS2_voltage>cycle2ChargeBat_voltage) {
					pwmout--;	// decrase charging power
				}
				if (solar_currentS2_voltage<cycle2ChargeBat_voltage) {
					pwmout++;	// increase charging power
				}
				// check do we jump to stage 3?
				if (solar_current<=floatChargeBat_current && solar_currentS2_voltage>=cycle2ChargeBat_voltage) {
					chargingStage_step = 3;
					Serial.println("Stage 3");
				}
				// check do we jump back to stage 1
				if (solar_current>cycle2ChargeBat_current + cycle2ChargeBat_current / 10) {	// take to account little hysterisis 10%
					chargingStage_step = 1;		// battery needs to be charged in stage 1
					Serial.println("Stage 1");
				}
				break;
			case 3:  // Float charging
				if (solar_currentS2_voltage>floatChargeBat_voltage) {
					pwmout--;	// decrase charging power
				}
				if (solar_currentS2_voltage<floatChargeBat_voltage) {
					pwmout++;	// increase charging power
				}
				// check do we jump back to stage 2?
				if (solar_current>(floatChargeBat_current + floatChargeBat_current / 5)) { //}&& solar_currentS2_voltage<floatChargeBat_voltage) { // hysterisis 10%
					chargingStage_step = 2;
					Serial.println("Stage 2");
				}
				break;
		}
	}
	// update output power
	pwmout=constrain(pwmout,0,1023);
	analogWrite(CHARGEMOSFET_PIN,pwmout);
}

void requestOneWire_Temperature() {
	oneWire.reset();   
	oneWire.write(0xCC);  
 	oneWire.write(0x44);  
 	Serial.println("requestOneWire_Temperature");
 	// Can't read next 750ms...
}

void readOneWire_Temperature() {
 byte data[2];  
 oneWire.reset();  
 oneWire.write(0xCC);  
 oneWire.write(0xBE);  
 data[0] = oneWire.read();   
 data[1] = oneWire.read();  
 int Temp = (data[1]<< 8)+data[0];  
 ambient_Temperature = (float)Temp / 16.0;  
 Serial.print("Temperature: ");
 Serial.println(ambient_Temperature);

}

// Read values from ADC and calculate voltage
void readVoltages() {
	sample1 = digitalSmooth(analogRead(SOLARVOLTAGE_PIN), solar_voltage_SmoothArray);
    sample2 = digitalSmooth(analogRead(SOLARCURRENTS1_PIN), solar_currentS1_SmoothArray);
    sample3 = digitalSmooth(analogRead(SOLARCURRENTS2_PIN), solar_currentS2_SmoothArray);    

    solar_voltage = (((float)sample1/1024.0) * 5000.0) / VdevSolarBat;  // 5000mV - ref voltage
    solar_currentS1_voltage = (((float)sample2/1024.0) * 5000.0) / VdevSolar1;  // 5000mV - ref voltage
    solar_currentS2_voltage = (((float)sample3/1024.0) * 5000.0) / VdevSolar2;  // 5000mV - ref voltage
    solar_current = (float)(solar_currentS1_voltage - solar_currentS2_voltage) / shuntR; // in mAh
    if (solar_current<0) {solar_current=0;}
	solar_power = solar_voltage * solar_current / 1000; // mWatts

}


// If ambient (battery) temperature changes, then Charge voltage valuse must be also adjusted. enough to do it every minute.
void AdjustChargeVoltageAmbientTemperature() {
    // calculate the current along with voltages
    cycle1ChargeBat_current = BATTERY_MAH * C_CHARGECURRENT_CYCLE1 / 100;  //mAh
    cycle2ChargeBat_current = BATTERY_MAH * C_CHARGECURRENT_CYCLE2 / 100;  //mAh
    floatChargeBat_current = BATTERY_MAH * C_CHARGECURRENT_FLOAT / 100;  //mAh

	// in mV
	if (ambient_Temperature>25) {
		floatChargeBat_voltage= (C_CHARGEVOLTAGE_FLOAT - ((ambient_Temperature- 25)*floatChargeCellTemperatureAdj)) * BATTERY_CELLS; // 1000;
		cycle1ChargeBat_voltage= (C_CHARGEVOLTAGE_CYCLE1 - ((ambient_Temperature- 25)*cycleChargeCellTemperatureAdj)) * BATTERY_CELLS; // 1000;
		cycle2ChargeBat_voltage= (C_CHARGEVOLTAGE_CYCLE2 - ((ambient_Temperature- 25)*cycleChargeCellTemperatureAdj)) * BATTERY_CELLS; // 1000;
	} else if (ambient_Temperature<25) {
		floatChargeBat_voltage= (C_CHARGEVOLTAGE_FLOAT + ((25 - ambient_Temperature)*floatChargeCellTemperatureAdj)) * BATTERY_CELLS; // 1000;
		cycle1ChargeBat_voltage= (C_CHARGEVOLTAGE_CYCLE1 + ((25 - ambient_Temperature)*cycleChargeCellTemperatureAdj)) * BATTERY_CELLS; // 1000;
		cycle2ChargeBat_voltage= (C_CHARGEVOLTAGE_CYCLE2 + ((25 - ambient_Temperature)*cycleChargeCellTemperatureAdj)) * BATTERY_CELLS; // 1000;
	} else {
		floatChargeBat_voltage= C_CHARGEVOLTAGE_FLOAT * BATTERY_CELLS; // 1000;
		cycle1ChargeBat_voltage= C_CHARGEVOLTAGE_CYCLE1 * BATTERY_CELLS; // 1000;
		cycle2ChargeBat_voltage= C_CHARGEVOLTAGE_CYCLE2 * BATTERY_CELLS; // 1000;
	}
	Serial.print("floatChargeBat_voltage: ");Serial.println(floatChargeBat_voltage);
	Serial.print("cycle1ChargeBat_voltage: ");Serial.println(cycle1ChargeBat_voltage);
	Serial.print("cycle2ChargeBat_voltage: ");Serial.println(cycle2ChargeBat_voltage);

}

// routine uploads to the LCD character indicating level of battery charge level
void updateBatIconCharge() {
	if ((screenNr_LCD & B01111111)==1 || (screenNr_LCD & B01111111)==2) { // only update char if we on a 1 or 2 screen
		if (pwmout==0) {	// charge % open circuit
			if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_0 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &batDead_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_10 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &batEmpty_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_30 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &bat10_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_50 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &bat30_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_70 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &bat50_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_90 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &bat70_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_OPEN_100 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &bat90_char,8);
			} else if (solar_currentS2_voltage>=C_CHARGEPERCENT_OPEN_100 * BATTERY_CELLS) {
				memcpy_P(customChar_LCD, &batFull_char,8);
			}
		} else {	// charge % while charging
			if (solar_currentS2_voltage<C_CHARGEPERCENT_0 * BATTERY_CELLS && chargingStage_step==1) {
				memcpy_P(customChar_LCD, &batDead_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_10 * BATTERY_CELLS && chargingStage_step==1) {
				memcpy_P(customChar_LCD, &batEmpty_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_30 * BATTERY_CELLS && chargingStage_step==1) {
				memcpy_P(customChar_LCD, &bat10_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_50 * BATTERY_CELLS && chargingStage_step==1) {
				memcpy_P(customChar_LCD, &bat30_char,8);
			} else if (solar_currentS2_voltage<C_CHARGEPERCENT_70 * BATTERY_CELLS && chargingStage_step==1) {
				memcpy_P(customChar_LCD, &bat50_char,8);
			} else if (solar_currentS2_voltage>=C_CHARGEPERCENT_70 * BATTERY_CELLS && chargingStage_step==1) {
				memcpy_P(customChar_LCD, &bat70_char,8);
			} else if (chargingStage_step==2) {
				memcpy_P(customChar_LCD, &bat90_char,8);
			} else if (chargingStage_step==3) {
				memcpy_P(customChar_LCD, &batFull_char,8);
			}
		}
		lcd.createChar(0, customChar_LCD);
	}
}

// LCD routines
void updateLCD() {
	//screenNr_LCD = screenNr_LCD & B11111110;	// clear unneeded info
	switch (screenNr_LCD & B01111111) {
		case 1:
		    sub_updateLCD();
		    lcd.setCursor(9,0); // set the cursor at 1st col and 1st row
		    lcd.write(byte(5));	// Amperes sign
		    lcd.print(" ");
		    lcd.print((float)solar_current / 1000.0, 2);	// print in A
		    lcd.print("A");

		    lcd.setCursor(12,1);
		    lcd.print("St:");
		    lcd.print(chargingStage_step);
    		break;
    	case 2:
		    sub_updateLCD();
		    lcd.setCursor(11,0); // set the cursor at 1st col and 1st row
		    lcd.print("Watts");
		    lcd.setCursor(11,1);
		    lcd.print(solar_power / 1000.0,1);
		    lcd.print(" ");
    		break;
    	case 3:
    		if((counterSecondWatts_storage & B11111100)==0) {	// update LCD on every 4th value
	    		lcd.setCursor(0,1);
	    		lcd.print("Minute (max:");
	    		lcd.print(everySecondWattsMax);
	    		lcd.print(") ");
	    		lcd.setCursor(0,0);
	    		// so average every 4 seconds and show
	    		int avgVal=0;
	    		byte tmpIdx=counterSecondWatts_storage;
	    		for (byte foo = 0; foo < 60; foo++) {
	    			avgVal+=everySecondWatts_storage[tmpIdx];
	    			//Serial.print(everySecondWatts_storage[tmpIdx]);Serial.print(" ");
	    			tmpIdx++;
	    			if (tmpIdx>59){tmpIdx=0;}
	    			if((foo & 3)==3) {	// now output average
	    				lcd.write(byte(map(avgVal/4, 0, everySecondWattsMax, 0, 7)));
	    				Serial.print(map(avgVal/4, 0, everySecondWattsMax, 0, 7));Serial.print(" ");
	    				avgVal=0;
	    			} 
	    		}
	    		Serial.println("");
    		}
    		break;
	}
//	Serial.print("solar_voltage: ");Serial.println(solar_voltage);
//	Serial.print("solar_currentS1_voltage: ");Serial.println(solar_currentS1_voltage);
//	Serial.print("solar_currentS2_voltage: ");Serial.println(solar_currentS2_voltage);
	Serial.print("PWM: ");Serial.println(pwmout); //pwmout
//	Serial.print("Float mA: ");Serial.println(floatChargeBat_current);
//	Serial.print("screenNr_LCD: ");Serial.println(screenNr_LCD);

}

// this will be printed more that on one screen
void sub_updateLCD() {
    // display Solar voltage
    lcd.setCursor(0,0); // set the cursor at 1st col and 1st row
    lcd.write(byte(1));
//    lcd.print(" ");
    lcd.print(solar_voltage / 1000.0,1);
    lcd.print("v  ");
	// display Battery voltage
    lcd.setCursor(0,1); // set the cursor at 1st col and 1st row
    lcd.write(byte(0));
//    lcd.print(" ");
    lcd.print(solar_currentS2_voltage / 1000.0, 1);
    lcd.print("v ");
	// display Load On/Off sign
    lcd.setCursor(9,1); 
    if (load_RELAY == LOW) {lcd.write(byte(6));} else {lcd.write(byte(7));}
}

void changeLCDscreen() {
	if (buttonReturnedValue==2) {	// need to change screen
		// but lets change it only after button is released
		if ((screenNr_LCD&B10000000)==0) {	
			// set flag for changing LCD screen
			screenNr_LCD=(screenNr_LCD | B10000000);
			switchLCDBacklightOn();
		}
	} else if(buttonReturnedValue==0) {	// no button is pressed
		if ((screenNr_LCD&B10000000)==128) {	
			// change a screen
			screenNr_LCD=(screenNr_LCD&B01111111); // clear last bit
			screenNr_LCD++;
			if (screenNr_LCD>SCREEN_LCD_TOTAL){screenNr_LCD=1;}
			lcd.clear();
			// do special routines for screens
			switch (screenNr_LCD & B01111111) {
				case 1:
					lcd_loadCustomCharactersDefault();
					break;
				case 2:
					lcd_loadCustomCharactersDefault();
					break;
				case 3:
					lcd_loadCustomCharactersGraph();
					break;
			}
			// force to update LCD screen
			updateLCD();
		}
	}
}

// if button 1 is pressed, then switch LCD backlight on. 
// after some time (1 min) switch LCD backlight off
void switchLCDBacklightOnOff() {
	if (pwmout>0) {	// if solar panel working (charging is on) turn LCD backlight on - no need to save power	
		switchLCDBacklightOn();
	} else if (buttonReturnedValue==1) {
		switchLCDBacklightOn();
	} else if (backlight_LCD == HIGH) {	// turn off if backlight on
		// lets turn backlight off after 1 min
		if (millis()-lcdOnTime>60000) {
			backlight_LCD = LOW;
			digitalWrite(BACKLIGHT_PIN, LOW);
		}
	}
}

// Switch load On/Off depending on the battery voltage.
// Load will be swithced Off when Battery is 10%.
// Load will be switched back On if Battery is more than 50%.
void switchLoadOnOff() {
	if (millis()-loadOnOffTime>10000) {		// delaz for 10 seconds
		if ((pwmout==0 && solar_currentS2_voltage<=C_CHARGEPERCENT_OPEN_0 * BATTERY_CELLS) || (pwmout>0 && solar_currentS2_voltage<=C_CHARGEPERCENT_0 * BATTERY_CELLS)) {
			if (load_RELAY != LOW) {
				load_RELAY = LOW;
				digitalWrite(BACKLIGHT_PIN, LOW);
				loadOnOffTime = millis();	// start a timer for 10 seconds.
			}
		} else if ((pwmout==0 && solar_currentS2_voltage>C_CHARGEPERCENT_OPEN_50 * BATTERY_CELLS) || (pwmout>0 && solar_currentS2_voltage>C_CHARGEPERCENT_50 * BATTERY_CELLS)) {
			if (load_RELAY != HIGH) {
				load_RELAY = HIGH;
				digitalWrite(BACKLIGHT_PIN, HIGH);
				loadOnOffTime = millis();	// start a timer for 10 seconds.
			}
		}
	}		
}


void switchLCDBacklightOn() {
	backlight_LCD = HIGH;
	digitalWrite(BACKLIGHT_PIN, HIGH);
	lcdOnTime = millis();	// start a timer for 1 min.
}

void lcd_createChar_P(byte c_char, byte c_array[]) {
	memcpy_P(customChar_LCD, c_array,8);
	lcd.createChar(c_char, customChar_LCD);
}

// Special characters like Battery, Solar Panel etc
void lcd_loadCustomCharactersDefault() {
	lcd_createChar_P(1,solar_char);
	lcd_createChar_P(2,temperature_char);
	lcd_createChar_P(3,watthour_char);
	lcd_createChar_P(4,celsium_char);
	lcd_createChar_P(5,ampers_char);
	lcd_createChar_P(6,loadOff_char);
	lcd_createChar_P(7,loadOn_char);
	updateBatIconCharge();
}
// Special characters for representing a graph
void lcd_loadCustomCharactersGraph() {
	lcd_createChar_P(0,graph0_char);
	lcd_createChar_P(1,graph1_char);
	lcd_createChar_P(2,graph2_char);
	lcd_createChar_P(3,graph3_char);
	lcd_createChar_P(4,graph4_char);
	lcd_createChar_P(5,graph5_char);
	lcd_createChar_P(6,graph6_char);
	lcd_createChar_P(7,graph7_char);
}

// Buttons routines (analog)
// analog butons little bit tricky to read. We need to make sure, that we read enough times ADC for get stable output.
// SO, the routine first waits for change on the ADC pin, and then collects a stable data until button is released.
//  then data is anlyzed and returned a value corresponding to the pressed button.
void readButtons() {
//	#define FIRSTBUTTONVALUE 0	// divider 20K/0K
	#define SECONDBUTTONVALUE 375 // divider 20K/20K
	int analogpin = analogRead(BUTTONS_PIN);
	if (analogpin>=buttonAnalogPrevVal-2 and analogpin<=buttonAnalogPrevVal+2) { // is value is changes
		// no value is the same - read is successful
		if (buttonAnalogReadCntr>20) { // are ADC value is stable?
			// yes, value is stable, let's return it
			//Serial.print("Button analog value: ");Serial.println(analogpin);
			if (analogpin<=20) {	
				buttonReturnedValue = 1; // first button is pressed
			} else if (analogpin>=SECONDBUTTONVALUE-50 and analogpin<=SECONDBUTTONVALUE+50) {
				buttonReturnedValue = 2; // second button is pressed
			} else {
				buttonReturnedValue = 0; // no button is pressed
			}
			//Serial.print("Button Return value: ");Serial.println(buttonReturnedValue);
		} else {
			// no, value not yet stabilized...
			buttonAnalogReadCntr++;
		}
	} else {
		// yes, value is changes, we are not ready with button
		buttonAnalogReadCntr = 0;	// reset counter of successful reads
		buttonAnalogPrevVal = constrain(analogpin,4,1023);	// update prev value
	}
}

// Accumulates Volts and Ampers
void accumulateWatts() {
	AccumulatedWatts = AccumulatedWatts + solar_power;
	AccumulatingCounter++;
}

// executes every second to get Watts counters values updated
void updateWattsCounters() {
	AccumulatedWatts = AccumulatedWatts / AccumulatingCounter;	// get average
	AccumulatingCounter=1;	// we left one average value in accumulated var
	// update arrays
	// every second
	everySecondWatts_storage[counterSecondWatts_storage]=AccumulatedWatts/1000;	// convert mWatt to Watt (integer)
	if (everySecondWattsMax<everySecondWatts_storage[counterSecondWatts_storage]){everySecondWattsMax=everySecondWatts_storage[counterSecondWatts_storage];}
	everySecondWattsAccumulated+=AccumulatedWatts;
	counterSecondWatts_storage++;
	if(counterSecondWatts_storage==60){
		// every minute
		everyMinuteWatts_storage[counterMinuteWatts_storage]=everySecondWattsAccumulated / 60;
		everyMinuteWattsAccumulated+=everyMinuteWatts_storage[counterMinuteWatts_storage];
		counterSecondWatts_storage=0;
		everySecondWattsMax=0;
		everySecondWattsAccumulated=0;
		counterMinuteWatts_storage++;
		if(counterMinuteWatts_storage==60){
			// every hour
			everyHourWatts_storage[counterHourWatts_storage]=everyMinuteWattsAccumulated / 60;
			everyHourWattsAccumulated+=everyHourWatts_storage[counterHourWatts_storage];
			counterMinuteWatts_storage=0;
			everyMinuteWattsAccumulated=0;
			counterHourWatts_storage++;
			if(counterHourWatts_storage==24){
				// every day
				everyDayWatts_storage[counterDayWatts_storage]=everyHourWattsAccumulated / 24;
				everyDayWattsAccumulated+=everyDayWatts_storage[counterDayWatts_storage];
				counterHourWatts_storage=0;
				everyHourWattsAccumulated=0;
				counterDayWatts_storage++;
				if(counterDayWatts_storage==30){
					// every Month
					everyMonthWatts_storage[counterMonthWatts_storage]=everyDayWattsAccumulated / 30;
					everyMonthWattsAccumulated+=everyMonthWatts_storage[counterMonthWatts_storage];
					counterDayWatts_storage=0;
					everyDayWattsAccumulated=0;
					counterMonthWatts_storage++;
					if(counterMonthWatts_storage==12){
						counterMonthWatts_storage=0;
						everyMonthWattsAccumulated=0;
					}
				}
			}
		}
	}
}


// smooth algorytm for ADC reading
int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }

  return total / k;    // divide by number of samples
}
