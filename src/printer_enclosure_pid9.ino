/* printer_enclosure_pid9
 *  by: Kevin Bernas
 *  Updated: 5-25-2019
 */

#include <SoftwareSerial.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <avr/pgmspace.h>
#include <PID_v1.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//Use this if your room and enclosure temperatures are not correct.
//#define swapDallas      //uncomment to "swap locations" of the dallas temp sensors

//pins///////////////////////////////////////////////
#define dallasPin    2    //To Dallas data pins (requires one 4k7 pullup)
#define freshFanPin  3    //to cooling fan mosfet
#define selectPin    4    //all buttons are momentary N.O. (internal pullups are used)
#define upPin        5
#define downPin      6
#define VFD_RX       8    //not physically used, but required for softserial arguments
#define mixFanPin    9    //to mixing fan mosfet
#define caseLedPin   10   //to case light mosfet
#define spotlightPin 11   //to spotlight driver (LDD-350LL driver)
#define VFD_TX       12   //to VFD serial rx (pin 14)
#define heaterPin    13   //to heater ssr input

//user prefs///////////////////////////////////////////////
#define debounce     50   //button debounce millis (no repeaters, so this can be short)
#define longPress    2000 //millis to hold button for special menus
#define splashTime   3000 //millis to show splash screen
#define displayPeriod 200 //millis between display updates (<200 values may blur)

#define caseLedPWMdefault 255 //bootup case lighting brightness
#define caseLedIncrement 25.5 //PWM change per up/down press
#define spotlightPWMdefault 255 //bootup spotlight brightness
#define spotlightIncrement 25.5 //PWM change per up/down press

#define freshFanStartupPWM 50 //fresh fan PWM value used for kickstarting
#define freshFanMinPWM   13//fresh fan PWM below which the fan stops spinning

#define mixFanPWMdefault 0 //bootup manual mix fan speed
#define mixFanIncrement 25.5 //PWM change per up/down press
#define mixFanStartupPWM 50  //mix fan PWM value used for kickstarting
#define mixFanMinPWM   15  //mix fan PWM below which the fan stops spinning

#define fanStartupPeriod 500 //millis to kickstart all fans

#define heaterMinSwitchTime 10000  //min millis between heater switching
#define heaterTempOffset 1.0  //celsius below desired temp when heater is allowed on (to avoid heater vs cooling fan fights)
#define heaterHysterisis 0.5 //Celsius hysterisis to use for heater
#define heaterMixFanDelay 30000 //millis to keep mixing fan on after heater turns off (>heaterMinSwitchTime)
#define heaterMixFanPWM 255 //pwm value for mix fan while heater is on (255)
#define heaterMaxOffTime 35000 //millis of heater off time to allow before shutting down heating function (heatsoak timer... >heaterMixFanDelay)
#define heaterHoursIncrement 1 //hrs to increment heater timer
#define heaterHoursMax 48 //max hrs allowed for heater timer (any int... 48hrs should cover most prints)

#define tempDefault  44.0   //bootup desired Celsius setting (50)
#define tempMinSetting      20.0   //min desired temperature allowed(20)
#define tempMaxSetting      50.0   //max desired temperature allowed (50). Stepper motors may fail at >50C.
#define tempCooldownDefault 24.0 //default cooldown temperature (30)
#define tempCooldownHysterisis 1.0 //hysterisis for fans in cooldown mode
#define tempIncrement 2.0   //celsius change per up/down push (2)

#define dallasResolution 12 //dallas bit resolution (4,8, or 12)
#define sizeOfBuffer 3    //the number of temperature readings to store for averaging (1 = disable, larger buffers use more memory
                          //and may result in laggy PID feedback, which may increase overshoot)
#define tempPeriod   800  //millis to wait for a requested temp from the Dallas sensor (datasheet says >760 for 12bit)

//PID tuning parameters... will depend on many aspects of your setup. Use ZN or other manual tuning methods.
//The default PID's work well with my enclosure using a specific assortment of hardware, so they
//may be a good starting point for tuning most setups.
#define freshFanPIDsampleTime 200  //200ms should work for most... be aware that PID values scale to sample time
double freshFanP=    10;  //the pid's must be set whether or not you use bang bang
double freshFanI=    1;   //default p,i,d = 10,5,1
double freshFanD=    1;

//(more) global vars///////////////////////////////////////////////
unsigned long currentMillis = 0;   //duh... saves millis() at the start of each loop
unsigned long tempRequestTime = 0; //millis when temp request was sent
unsigned long lastDisplayTime = 0; //store time when last display update was sent
unsigned long lastButton = 0;      //debouncing timer storage
unsigned long buttonTimer = 0;     //long press button time storage
bool upButtonFlag = 0;             //flag to register long button presses
bool downButtonFlag = 0;           //flag to register long button presses
bool selectButtonFlag = 0;           //flag to register long button presses
byte tempState = 0;                //temperature state machine variable
byte mode = 0;                     //flag for mode...
//0=auto+autoh, 1=cooldown, 2=mix fan setting, 3=led setting, 4=heater setting, 5=spotlight setting, 6=heater timer setting
double tempArray[sizeOfBuffer];    //array to store individual temperature readings
double temp2Array[sizeOfBuffer];   //array to store individual temperature readings
unsigned int i = 0;                //temperature buffer index
double tempAverage = 22;           //current value averaged from buffer
double temp2Average = 22;          //current value averaged from buffer
double tempDesired = tempDefault;  //desired temp storage
double tempCooldown = tempCooldownDefault;  //cooldown temp storage
double freshFanPWM = 0;            //fresh fan pwm storage
bool freshFanStarted = 0;          //store fan started/stopped status
unsigned long freshFanStartTime = 0;
double mixFanManualPWM = mixFanPWMdefault; //mix fan manual manual pwm value storage
double mixFanPWM = 0;              //actual output mix fan pwm storage
bool mixFanStarted = 0;            //store fan started/stopped status
unsigned long mixFanStartTime = 0;
double caseLedPWM = caseLedPWMdefault;  //case LED pwm storage
double spotlightPWM = spotlightPWMdefault;  //case LED pwm storage
bool heaterStatus = 0;             //storage for heater on/off status
bool heaterControl = 0;            //heater function on/off (0 = off after bootup)
unsigned long heaterSwitchTime = 0;   //store last time heater was turned on or off
unsigned long heaterHours = 0;               //store heat timer hours
unsigned long heaterStartTime = 0; //store heat timer start timer
bool heaterTimerStarted = 0;       //flag to start heat timer

///////////////////////////////////////////////////////////////////////////////////////////
//Control bytes for the IEE VFD in "LCD" or "Intel" mode, from the IEE S036x2_N manual   //
//(hex converted to decimal). Most of these codes have been tested on an                 //
//IEE 036X2-122-09220 VFD (AKA 05464ASSY35119-02A). Other IEE vfd's are likely to work.  //
///////////////////////////////////////////////////////////////////////////////////////////
const uint8_t VFDCTRL_CURSOR_BACKSPACE = 8;
const uint8_t VFDCTRL_CURSOR_ADVANCE = 9;
const uint8_t VFDCTRL_LINE_FEED = 10;
//const uint8_t VFDCTRL_CURSOR_BLINK_BLOCK = 11; // Applies to 036X2–151–05240 & 036X2-160-05440 only.
//const uint8_t VFDCTRL_CURSOR_UNDERBAR = 12; // Applies to 036X2–151–05240 & 036X2-160-05440 only.
const uint8_t VFDCTRL_CARRIAGE_RETURN = 13;
const uint8_t VFDCTRL_CURSOR_OFF = 14;
const uint8_t VFDCTRL_CURSOR_ON = 15; //default
const uint8_t VFDCTRL_SCROLL_LINE_LOCK = 16; // 2 byte command...
/* 0   Locks line 0
 * 1   Locks lines 0 and 1
 * 2   Locks lines 0, 1 and 2
 * 255   Cancel Line Lock
 */
const uint8_t VFDCTRL_SET_VERTICAL_SCROLL_MODE = 17; //default auto cr+lf
const uint8_t VFDCTRL_SET_HORIZONTAL_SCROLL_MODE = 19;
const uint8_t VFDCTRL_SOFTWARE_RESET = 20;
const uint8_t VFDCTRL_CLEAR_DISPLAY_HOME_CURSOR = 21;
const uint8_t VFDCTRL_HOME_CURSOR = 22;
const uint8_t VFDCTRL_SET_DATABIT_7_HIGH = 23; //for the following byte
const uint8_t VFDCTRL_BEGIN_USER_DEFINED_CHARACTER = 24; //7 byte command follow with...
/* 1 User defined character storage location byte (10 available, from 246 - 255)
 *  and
 * 5 bytes of data for the character dots. 0=Dot off, 1=Dot on.
 * ex: "24 246 P P P P P"
 * where P are position bytes containing 8 bits each (01101010)
 *
CHARACTER DOT MATRIX (physical location of the actual dots)
1   2   3   4   5
6   7   8   9   10
11  12  13  14  15
16  17  18  19  20
21  22  23  24  25
26  27  28  29  30
31  32  33  34  35

Map of data bits for part#'s (036X2-100,-105,-122,-124,-134,-130)
Bit>  7   6   5   4   3   2   1   0
Btye  ------------------------------
3     33  15  34  16  35  17  0   18
4     29  11  30  12  31  13  32  14
5     25  07  26  08  27  09  28  10
6     21  03  22  04  23  05  24  06
7     0   0   0   0   19  01  20  02

Map of data bits for part#'s (036X2-106,-120,-121,-151,-116,-160)
Bit>  7   6   5   4   3   2   1   0
Byte  ------------------------------
3     29  20  11  02  28  19  10  01
4     31  22  13  04  30  21  12  03
5     33  24  15  06  32  23  14  05
6     35  26  17  08  34  25  16  07
7     0   0   0   0   0   27  18  09
 */
const uint8_t VFDCTRL_SET_ADDRESSBIT_0_HIGH = 25; //must be followed by control codes below
const uint8_t VFDCTRL_CURSOR_UP_ONE_LINE = 26;
const uint8_t VFDCTRL_CURSOR_MOVE_TO_LOCATION = 27; //2 byte command...
//Follow with a 1 byte position ID. Screen positions are numbered from left to right, top to
//bottom starting with 0.
const uint8_t VFDCTRL_CHARSET_EUROPEAN = 28; //default
const uint8_t VFDCTRL_CHARSET_KATAKANA = 29;
const uint8_t VFDCTRL_CHARSET_CRYLLIC = 30;
const uint8_t VFDCTRL_CHARSET_HEBREW = 31;

//***THE FOLLOWING VFD CONTROL CODES MUST BE PRECEDED BY the byte '25' (Set Ao high)***
const uint8_t VFDCTRL_SET_SCREEN_OR_COLUMN_BRIGHTNESS = 48; //3 byte command...
/* Follow with Column ID and Brightness Level.
 * Column ID 0-X???, use 255 for entire screen
 * Brightness Level brightest=0 to dimmest=7. default=0
 */
const uint8_t VFDCTRL_BEGIN_BLINKING_CHARACTERS = 49; //2 byte command...
/* Follow with rate and type byte
 * Type       Rate    byte
 * -------------------------------------
 * Character  OFF     00   <----default
 * Character  1hz     01
 * Character  2hz     02
 * Character  4hz     04
 * Underbar   OFF     96  -applies only to 36X2–151–05240
 * Underbar   1hz     97  -" "
 * Underbar   2hz     98  -" "
 * Underbar   4hz     100 -" "
 * Both       OFF     128 -" "
 * Both       1hz     129 -" "
 * Both       2hz     130 -" "
 * Both       4hz     132  -" "
 */
const uint8_t VFDCTRL_END_BLINKING_CHARACTERS = 50;
const uint8_t VFDCTRL_BLANK_DISPLAY_ON = 51;
const uint8_t VFDCTRL_BLANK_DISPLAY_OFF = 52;
//static const char VFDCTRL_COMMA_PERIOD_TRIANGLE_FUNCTION = 53; //Applies to 036X2-121-11120 only.
const uint8_t VFDCTRL_ERASE_LINE_WITH_END_BLINK = 54; //2 byte command...
/*
 *
 */
const uint8_t VFDCTRL_SET_CR_LF_DEFINITIONS = 55; //2 byte command...
/* Follow with the definition byte
 * bits LFin   CRin
 * --------------------------------
 * 0    LF     CR     <---default
 * 1    LF+CR  CR
 * 2    LF     CR+LF
 * 3    LF+CR  CR+LF
 */
//static const char VFDCTRL_UNDERBAR_ON = 56; // Applies to 036X2–151–05240 & 036X2-160-05440 only.
//static const char VFDCTRL_UNDERBAR_OFF = 57; // Applies to 036X2–151–05240 & 036X2-160-05440 only.
const uint8_t VFDCTRL_SELECT_RIGHT_TO_LEFT_DATA_ENTRY = 58;
const uint8_t VFDCTRL_SELECT_LEFT_TO_RIGHT_DATA_ENTRY = 59;  //default
const uint8_t VFDCTRL_SCREEN_SAVER_ON = 60;
const uint8_t VFDCTRL_SCREEN_SAVER_OFF = 61; //default
const uint8_t VFDCTRL_SELF_TEST_EXECUTE = 62;
const uint8_t VFDCTRL_SELF_TEST_TERMINATE = 63;


//dallas sensor///////////////////////////////////////////////
OneWire oneWire(dallasPin);
DallasTemperature sensors(&oneWire);
DeviceAddress dallasAddress, dallas2Address;
//IEE VFD display//////////////////////////////////////////////////////////////////////////////
//Connect the VFD pin-14 to the "VFD_TX". (rx, tx, invert)
SoftwareSerial vfd(VFD_RX,VFD_TX,true); //inverted output for direct (rs232) connection

//pids are used for both pid and bang bang... (&input,&output,&setpoint,p,i,d,direction)
PID freshFanPID(&tempAverage, &freshFanPWM, &tempDesired,freshFanP,freshFanI,freshFanD, P_ON_M, REVERSE);

void setup() {
  //start vfd
  vfd.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for vfd to boot up
  vfd.write(VFDCTRL_CLEAR_DISPLAY_HOME_CURSOR); //clear screen

  //pins
  pinMode(selectPin,INPUT_PULLUP);             //mode button w/pull-up
  pinMode(upPin,INPUT_PULLUP);                 //up button w/ pull-up
  pinMode(downPin,INPUT_PULLUP);               //down button w/pull-up
  pinMode(freshFanPin,OUTPUT);                 //cooling fan signal
  pinMode(mixFanPin,OUTPUT);                   //mixing fan signal
  pinMode(caseLedPin,OUTPUT);                  //lighting signal
  pinMode(spotlightPin,OUTPUT);                //spotlight signal
  pinMode(heaterPin,OUTPUT);                   //heater signal

  //safe init outputs
  analogWrite(freshFanPin,0);
  analogWrite(mixFanPin,0);
  analogWrite(spotlightPin,255);
  analogWrite(caseLedPin,255);
  digitalWrite(heaterPin,0);

  //start sensor
  sensors.begin();
  #ifdef swapDallas
    sensors.getAddress(dallas2Address, 0);  //store address of sensor 2 at index 0
    sensors.getAddress(dallasAddress, 1);  //store address of sensor 1 at index 1
  #endif
  #ifndef swapDallas
    sensors.getAddress(dallasAddress, 0);  //store address of sensor 1 at index 0
    sensors.getAddress(dallas2Address, 1);  //store address of sensor 2 at index 1
  #endif
  sensors.setResolution(dallasAddress, dallasResolution);    //set res
  sensors.setResolution(dallas2Address, dallasResolution);    //set res
  sensors.setWaitForConversion(false); //instead we will use non-blocking code to manually time our readings

  //Splash screen
  //         ("12345678901234567890");
  vfd.print(F("  Smart Enclosure   "));
  vfd.print(F("  by: Kevin Bernas  "));
  delay(splashTime);  //window of time to release a "hold on boot" button

  //start PIDs
  freshFanPID.SetSampleTime(freshFanPIDsampleTime);
  freshFanPID.SetMode(AUTOMATIC);
}

void loop() {
  currentMillis = millis();  //get the time for this loop
  readButtons();
  updateTemp();
  freshFanPID.Compute();
  updateOutputs();
  if(currentMillis - lastDisplayTime > displayPeriod) {
    updateDisplay();
  }
}  //enaloop... ;)

//update Dallas temperature sensor buffers (non-blocking)
void updateTemp()  {
  switch (tempState) {
     case 0:
     sensors.requestTemperatures(); // Send the command to get temperature conversions
     tempRequestTime = currentMillis; // keep track of time of start
     tempState = 1; // change to waiting state
     break;

     case 1: // bypass until temp conversions are ready
     if (currentMillis - tempRequestTime >= tempPeriod) { // conversions should be ready... process the new value
       i++;
       if(i >= sizeOfBuffer) {  //restart buffer if it's full
         i=0;
       }
       tempArray[i] = sensors.getTempC(dallasAddress); //store new temp value (*note, a float is recast to double)
       temp2Array[i] = sensors.getTempC(dallas2Address); //store new temp value (*note, a float is recast to double)
       tempAverage = 0;  //calc new average temp...
       temp2Average = 0;  //calc new average temp...
       for(int j = 0; j < sizeOfBuffer; j++) {
         tempAverage += tempArray[j];
         temp2Average += temp2Array[j];
       }
       tempAverage = tempAverage / sizeOfBuffer;
       temp2Average = temp2Average / sizeOfBuffer;
       tempState = 0; // start another conversion next time around
     }
     break;
  }
}

//Read Buttons Routine (non-blocking)/////////////////////////////////////////
void readButtons(void)  {
  if(currentMillis - lastButton > debounce)  {  //debounced...
    //select button----------------------------------------------------------
    if(!digitalRead(selectPin)) {   //select pushed
      if(!selectButtonFlag) {  //for the first loop select is pressed
        selectButtonFlag = 1;  //set long press timer flag
        buttonTimer = currentMillis;  //start long press timer
      }
      lastButton = currentMillis;
      return;
    }
    //after a short select press
    else if(selectButtonFlag && currentMillis - buttonTimer < longPress && digitalRead(selectPin))  {
      selectButtonFlag = 0;  //reset timer flag
      if(!mode) {  //we are in auto mode
        freshFanPID.SetMode(MANUAL);
        mode = 1;  //change to cooldown mode
      }
      else if(mode == 3) {  //we're in led setting mode
        mode = 5;  //change to spotlight setting mode
      }
      else if(mode == 4) {  //we're in heater setting mode
        mode = 6;  //change to heater timer setting mode
      }
      else {  //we are in either cooldown, mix fan setting, spotlight setting, or heater timer mode
        freshFanPID.SetMode(AUTOMATIC);
        mode = 0;  //change to auto mode
      }
      return;
    }
    //after a long select press
    else if(selectButtonFlag && currentMillis - buttonTimer >= longPress && digitalRead(selectPin))  {
      selectButtonFlag = 0;  //reset timer flag
      mode = 4; //switch to heater setting mode
      return;
    }

    //up button----------------------------------------------------------
    else if(!digitalRead(upPin)) {    //up is pressed
      if(!upButtonFlag) {  //just for the first loop when up is pressed
        upButtonFlag = 1;  //set up long press timer flag
        buttonTimer = currentMillis;  //start long press timer
      }
      lastButton = currentMillis;
      return;
    }
    //after a short up press
    else if(upButtonFlag && currentMillis - buttonTimer < longPress && digitalRead(upPin))  {
      upButtonFlag = 0;  //reset timer flag
      if(!mode)  {  //auto mode
        tempDesired = tempDesired + tempIncrement;  //increment desired temp
        if(tempDesired > tempMaxSetting) {  //stay within range
          tempDesired = tempMaxSetting;
        }
        return;
      }
      else if(mode == 1) {  //cooldown mode
        tempCooldown = tempCooldown + tempIncrement;
        if(tempCooldown > tempMaxSetting) {  //stay within range
          tempCooldown = tempMaxSetting;
        }
        return;
      }
      else if(mode == 2) {  //mix fan setting mode
        mixFanManualPWM = mixFanManualPWM + mixFanIncrement;
        if(mixFanManualPWM > 255) {  //stay within range
          mixFanManualPWM = 255;
        }
        return;
      }
      else if(mode == 3) {  //case light setting mode
        caseLedPWM = caseLedPWM + caseLedIncrement;
        if(caseLedPWM > 255) {  //stay within range
          caseLedPWM = 255;
        }
        return;
      }
      else if(mode == 4) {  //heater setting mode
        heaterControl = 1;  //turn on heater function
        heaterSwitchTime = currentMillis;  //to make sure we don't have a false heater timeout
        return;
      }
      else if(mode == 5) {  //spotlight setting mode
        spotlightPWM = spotlightPWM + spotlightIncrement;
        if(spotlightPWM > 255) {  //stay within range
          spotlightPWM = 255;
        }
        return;
      }
      else if(mode == 6) {  //heater timer setting mode
        heaterHours = heaterHours + heaterHoursIncrement;
        if(heaterHours > heaterHoursMax) {  //stay within range
          heaterHours = heaterHoursMax;
        }
        heaterTimerStarted = 0; //set flag to start/restart timer
        return;
      }
    }
    //after a long up press
    else if(upButtonFlag && currentMillis - buttonTimer >= longPress && digitalRead(upPin))  {
      upButtonFlag = 0;  //reset timer flag
      mode = 2; //switch to mix fan setting mode
      return;
    }

    //downButton----------------------------------------------------------
    else if(!digitalRead(downPin))  { //down pressed
      if(!downButtonFlag) {  //for the first loop down is pressed
        downButtonFlag = 1;  //set long press timer flag
        buttonTimer = currentMillis;  //start long press timer
      }
      lastButton = currentMillis;
      return;
    }
    //after short down press
    else if(downButtonFlag && currentMillis - buttonTimer < longPress && digitalRead(downPin))  {
      downButtonFlag = 0;  //reset timer flag
      if(!mode)  {  //auto mode
        tempDesired = tempDesired - tempIncrement;
        if(tempDesired < tempMinSetting) {  //stay within range
          tempDesired = tempMinSetting;
        }
        return;
      }
      else if(mode == 1) {  //cooldown mode
        tempCooldown = tempCooldown - tempIncrement;
        if(tempCooldown < tempMinSetting) {  //stay within range
          tempCooldown = tempMinSetting;
        }
        return;
      }
      else if(mode == 2) {  //mix fan setting mode
        mixFanManualPWM = mixFanManualPWM - mixFanIncrement;
        if(mixFanManualPWM < 0) {  //stay within range
          mixFanManualPWM = 0;
        }
        return;
      }
      else if(mode == 3) {  //case light setting mode
        caseLedPWM = caseLedPWM - caseLedIncrement;
        if(caseLedPWM < 0) {  //stay within range
          caseLedPWM = 0;
        }
        return;
      }
      else if(mode == 4) {  //heater setting mode
        heaterControl = 0;  //turn off heater function
        heaterTimerStarted = 0; //re-enable heater start flag for next time
        return;
      }
      else if(mode == 5) {  //spotlight setting mode
        spotlightPWM = spotlightPWM - spotlightIncrement;
        if(spotlightPWM < 0) {  //stay within range
          spotlightPWM = 0;
        }
        return;
      }
      else if(mode == 6) {  //heater timer setting mode
        heaterHours = heaterHours - heaterHoursIncrement;
        if(heaterHours < 0) {  //stay within range
          heaterHours = 0;
        }
        heaterTimerStarted = 0; //set flag to start/restart timer
        return;
      }
    }
    //after long down press
    else if(downButtonFlag && currentMillis - buttonTimer >= longPress && digitalRead(downPin))  {
      downButtonFlag = 0;  //reset timer flag
      mode = 3; //switch to case light setting mode
      return;
    }
    else return;                                //no button pressed or recently released...
  }
  else return;                                  //still debouncing...
}

//update VFD routine (non-clearing, non-blocking)/////////////////////////////////////
void updateDisplay(void)  {
  lastDisplayTime = currentMillis;

  //calculate some percentages to display
  int freshFanPercent = 100*freshFanPWM/255;
  int mixFanPercent = 100*mixFanManualPWM/255;
  int caseLedPercent = 100*caseLedPWM/255;
  int spotlightPercent = 100*spotlightPWM/255;

  char fanString[4];             //arrays to store float digits as strings
  char tempAveString[5];
  char temp2AveString[5];
  char tempDesiredString[3];
  char tempCooldownString[3];
  char mixFanString[4];
  char caseLedString[4];
  char spotlightString[4];
  char heaterHoursString[3];
  dtostrf(freshFanPercent,3,0,fanString); //make strings from float data
  //dtostrf automatically pads any preceding blanks :)
  dtostrf(tempAverage,4,1,tempAveString);
  dtostrf(temp2Average,4,1,temp2AveString);
  dtostrf(tempDesired,2,0,tempDesiredString);
  dtostrf(tempCooldown,2,0,tempCooldownString);
  dtostrf(mixFanPercent,3,0,mixFanString);
  dtostrf(caseLedPercent,3,0,caseLedString);
  dtostrf(spotlightPercent,3,0,spotlightString);
  dtostrf(heaterHours,2,0,heaterHoursString);

  //Auto Mode
  if(!mode)  {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Auto  Temp:XX.X/XXdC"
    //          "12345678901234567890";
    if(heaterControl) {
      vfd.print(F("AutoH Temp:"));
    }
    else  {
      vfd.print(F("Auto  Temp:"));
    }
    vfd.print(tempAveString);
    vfd.print(F("/"));
    vfd.print(tempDesiredString);
    vfd.print(char(223)); //degree symbol (hex DF)
    vfd.print(F("C"));

    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //          "Room:XX.XdC Fan:XXX%"
    //          "12345678901234567890";
    vfd.print(F("Room:"));
    vfd.print(temp2AveString);
    vfd.print(char(223)); //degree symbol (hex DF)
    if(!heaterStatus) {  //heater is off
      vfd.print(F("C Fan:"));
      vfd.print(fanString);
      vfd.print(F("%"));
    }
    else  {  //heater is running
      vfd.print(F("C  Heating"));
    }
    return;
  }

  //Cooldown mode
  else if(mode == 1) {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Cool  Temp:XX.X/XXdC"
    //          "12345678901234567890";
    vfd.print(F("Cool  Temp:"));
    vfd.print(tempAveString);
    vfd.print(F("/"));
    vfd.print(tempCooldownString);
    vfd.print(char(223)); //degree symbol (hex DF)
    vfd.print(F("C"));

    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //          "Room:XX.XdC  Cooling"
    //          "Room:XX.XdC     Idle"
    //          "12345678901234567890";
    vfd.print(F("Room:"));
    vfd.print(temp2AveString);
    vfd.print(char(223)); //degree symbol (hex DF)
    vfd.print(F("C"));
    if(freshFanStarted)  {
      vfd.print(F("  Cooling"));
    }
    else  {
      vfd.print(F("     Idle"));
    }
    return;
  }

  //mix fan setting mode
  else if(mode == 2) {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "  Mixing Fan Speed  "
    //          "12345678901234567890";
    vfd.print(F("  Mixing Fan Speed  "));

    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //          "    Speed: XXX%     "
    //          "12345678901234567890";
    vfd.print(F("    Speed: "));
    vfd.print(mixFanString);
    vfd.print(F("%     "));
    return;
  }
  //Case Led setting mode
  else if(mode == 3) {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Case LED Brightness "
    //          "12345678901234567890";
    vfd.print(F("Case LED Brightness "));
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //          "  Brightness: XXX%  "
    //          "12345678901234567890";
    vfd.print(F("  Brightness: "));
    vfd.print(caseLedString);
    vfd.print(F("%  "));
    return;
  }
  //Heater setting mode
  else if(mode == 4) {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Case LED Brightness "
    //          "12345678901234567890";
    vfd.print(F("  Heater Function:  "));
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //            "  Brightness: XXX%  "
    //            "12345678901234567890";
    if(heaterControl) {
      vfd.print(F("         ON         "));
    }
    else  {
      vfd.print(F("         OFF        "));
    }
    return;
  }
  //Spotlight setting mode
  else if(mode == 5) {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Case LED Brightness "
    //          "12345678901234567890";
    vfd.print(F("Spotlight Brightness"));
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //          "  Brightness: XXX%  "
    //          "12345678901234567890";
    vfd.print(F("  Brightness: "));
    vfd.print(spotlightString);
    vfd.print(F("%  "));
    return;
  }
  //Heater Timer setting mode
  else if(mode == 6) {
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "    Heater Timer    "
    //          "12345678901234567890";
    vfd.print(F("    Heater Timer    "));
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //fan & heater output %'s
    //          "        XXhrs       "
    //          "12345678901234567890";
    vfd.print(F("        "));
    vfd.print(heaterHoursString);
    vfd.print(F("hrs       "));
    return;
  }
  else {  //no man's land
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(0)); //move to line 0, column 0
    //          "12345678901234567890";
    vfd.print(F("Undefined Mode Error"));
    vfd.write(VFDCTRL_CURSOR_MOVE_TO_LOCATION);
    vfd.write(uint8_t(20)); //move to line 1, column 0
    //          "12345678901234567890";
    vfd.print(F("                 :-("));
    return;
  }
}

//update the outputs routine (non-blocking)////////////////////////////////////
void updateOutputs(void)  {
  //cooldown mode
  if(mode == 1)  {
    heaterStatus = 0;  //turn off heater
    if(tempAverage < tempCooldown - tempCooldownHysterisis)  {  //cool enough, turn everything off
      analogWrite(freshFanPin,0);  //fresh fan off
      freshFanStarted = false;  //update fan startup flag
      mixFanPWM = 0;               //mix fan off
      mixFanStarted = false;  //raise started flag
    }
    else if(tempAverage > tempCooldown + tempCooldownHysterisis) {  //not cool enough, cooling off
      analogWrite(freshFanPin,255);  //both fans on full
      freshFanStarted = true;  //update fan startup flag
      mixFanPWM = 255;
      mixFanStarted = true;  //raise started flag
    }
  }

  //auto and setting modes
  else  {
    //fresh fan
    if(!freshFanStarted && freshFanPWM >= freshFanMinPWM) {  //fan is stopped and PWM is high enough to keep it spinning
      analogWrite(freshFanPin,freshFanStartupPWM);  //kick start power
      freshFanStarted = true;  //update fan startup flag
      freshFanStartTime = currentMillis; //start kickstart timer
    }
    else if(freshFanStarted && currentMillis - freshFanStartTime >= fanStartupPeriod && freshFanPWM >= freshFanMinPWM) {
      //cruise: fan has been starting long enough & pwm is high enough to keep it spinning
      analogWrite(freshFanPin,freshFanPWM);  //update pwm output
    }
    else if(freshFanPWM < freshFanMinPWM) {  //off: pwm is too low to keep it spinning
      analogWrite(freshFanPin,0);  //turn off fan
      freshFanStarted = false;  //reset startup flag
    }

    //heater feature enabled
    if(heaterControl) {
      //first loop with heater enabled and the timer is enabled
      if(!heaterTimerStarted && heaterHours) {
        heaterTimerStarted = 1;
        heaterStartTime = currentMillis;
      }
      //heater is off, temp too low, and it's been off long enough
      if(!heaterStatus && tempAverage <= tempDesired - heaterTempOffset - heaterHysterisis && currentMillis - heaterSwitchTime >= heaterMinSwitchTime)  {
        //heater has been off too long... must be heat soaked (requires resetting switchTime when heaterControl is activated!)
        if(currentMillis - heaterSwitchTime >= heaterMaxOffTime) {
          heaterControl = 0; //turn off heater function
          heaterTimerStarted = 0; //re-enable heater start flag for next time
        }
        //heater needs to be on
        else{
          heaterStatus = 1;
          heaterSwitchTime = currentMillis;
        }
      }
      //heater is on, temp is high enough, and it's been on long enough
      else if(heaterStatus && tempAverage >= tempDesired - heaterTempOffset + heaterHysterisis && currentMillis - heaterSwitchTime >= heaterMinSwitchTime) {
        heaterStatus = 0;
        heaterSwitchTime = currentMillis;
      }
      //heater timer is on and has elapsed...
      if(heaterHours && currentMillis - heaterStartTime > heaterHours * 3600000)  {
        heaterControl = 0; //turn off heater function
        heaterTimerStarted = 0; //re-enable heater start flag for next time
      }
    }
    else if(heaterStatus) {  //heater function was disabled with heater on
      heaterStatus = 0;
      heaterSwitchTime = currentMillis;
    }
    else heaterStatus = 0;  //heater function disabled and heater off... but just to be sure we turn off every time

    //mixing fan
    if(!mixFanStarted && (mixFanManualPWM >= mixFanMinPWM || freshFanPWM >= 2)) {
      //mix fan is stopped, and either manual mix pwm is high enough to run it or fresh fan pwm is not zero
      mixFanPWM = mixFanStartupPWM; //kick start mixing fan
      mixFanStarted = true;  //raise started flag
      mixFanStartTime = currentMillis;  //start kickstart timer
    }
    else if(mixFanStarted && currentMillis - mixFanStartTime >= fanStartupPeriod && (mixFanManualPWM >= mixFanMinPWM || freshFanPWM >= 2)) {
      //mix fan has been started long enough & fresh fan pwm is not zero or manual is high enough to keep spinning
      if(freshFanPWM <= mixFanManualPWM) {  //fresh fan pwm is lower than manual mix pwm
        mixFanPWM = mixFanManualPWM;  //use manual speed
      }
      else if(freshFanPWM > mixFanMinPWM)  { //fresh pwm is high enough to keep mix fan spinning
        mixFanPWM = freshFanPWM;  //throttle it with fresh fan
      }
      else mixFanPWM = mixFanMinPWM;  //keep the fan spinning until freshPWM goes to zero or manual pwm is inadequate
    }
    else if(freshFanPWM < 2 && mixFanManualPWM < mixFanMinPWM) {
      //fresh pwm<2 (=0 results in "bumping"), and manual pwm is too low to keep it spinning
      mixFanPWM = 0;  //shutoff fan and reset startup flag
      mixFanStarted = false;
    }
    if (mixFanPWM < heaterMixFanPWM && (heaterStatus || currentMillis - heaterSwitchTime < heaterMixFanDelay)) {
      //heater is on or just recently turned off, and mix fan is too slow
      mixFanPWM = heaterMixFanPWM;  //override mixing fan to on
      mixFanStarted = true;  //raise started flag
    }
  }//end of auto & setting modes

  //send updates to the outputs (except fresh fan)
  analogWrite(mixFanPin,mixFanPWM);
  analogWrite(caseLedPin,caseLedPWM);
  digitalWrite(heaterPin,heaterStatus);
  analogWrite(spotlightPin,spotlightPWM);
  return;

}  //end of function
