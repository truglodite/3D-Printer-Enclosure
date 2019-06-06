/* printer_enclosure.ino
 * by: Truglodite
 * Updated: 6-5-2019
 *
 * A heated 3d printer enclosure environment controller, based on Arduino,
 * with PID loop feedback to eliminate abrubt fan speed changes.
 * User configs are located in configuration.h.
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/pgmspace.h>
#include <PID_v1.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#include "configuration.h"

// global vars ////////////////////////////////////////////////////////////////
unsigned long currentMillis = 0;   //duh... saves millis() at the start of each loop
unsigned long tempRequestTime = 0; //millis when temp request was sent
unsigned long lastDisplayTime = 0; //store time when last display update was sent
unsigned long lastButton = 0;      //debouncing timer storage
unsigned long buttonTimer = 0;     //long press button time storage
bool upButtonFlag = 0;             //flag to register long button presses
bool downButtonFlag = 0;           //flag to register long button presses
bool selectButtonFlag = 0;         //flag to register long button presses
byte tempState = 0;                //temperature state machine variable
byte mode = 0;                     //flag for mode...
//0=auto+autoh, 1=cooldown, 2=mix fan setting, 3=led setting, 4=heater setting, 5=spotlight setting, 6=heater timer setting
double tempArray[sizeOfBuffer];    // array to store individual temp readings
double temp2Array[sizeOfBuffer];   // array to store individual temp readings
unsigned int i = 0;                // temperature buffer index
double tempAverage = 22.0;           // current value averaged from buffer
double temp2Average = 22.0;          // current value averaged from buffer
double tempDesired = tempDefault;  // desired temp storage
double tempCooldown = tempCooldownDefault;  //cooldown temp storage
double freshFanPWM = 0.0;            // fresh fan pwm storage
bool freshFanStarted = 0;          // store fan started/stopped status
unsigned long freshFanStartTime = 0;
double mixFanManualPWM = mixFanPWMdefault; // mix fan manual pwm value
double mixFanPWM = 0;              // actual output mix fan pwm
bool mixFanStarted = 0;            // fan started/stopped status
unsigned long mixFanStartTime = 0;
double caseLedPWM = caseLedPWMdefault;  //case LED pwm storage
double spotlightPWM = spotlightPWMdefault;  //case LED pwm storage
bool heaterStatus = 0;             //storage for heater on/off status
bool heaterControl = 0;            //heat function on/off (0 = off after bootup)
unsigned long heaterSwitchTime = 0; //store last time heat was turned on or off
unsigned long heaterHours = heaterHoursDefault; //store heat timer hours
unsigned long heaterStartTime = 0;  //store heat timer start timer
bool heaterTimerStarted = 0;        //flag to start heat timer
bool octoprintHeat = 0;             //op heat flag
bool octoprintCool = 0;             //op cool flag
bool octoprintUpdateFlag =  0;
unsigned long lastOctoprintRead = 0; //timer for op readings

//dallas sensor///////////////////////////////////////////////
OneWire oneWire(dallasPin);
DallasTemperature sensors(&oneWire);
DeviceAddress dallasAddress, dallas2Address;

#ifdef vfd_display
  //IEE VFD display//////////////////////////////////////////////////////////////////////////////
  //Connect the VFD pin-14 to the "VFD_TX". (rx, tx, invert)
  SoftwareSerial vfd(VFD_RX,VFD_TX,true); //inverted output for direct (rs232) connection
#endif
#ifndef vfd_display
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Banggood blue 20x4 i2c display... or 0x27
#endif

//pids are used for both pid and bang bang... (&input,&output,&setpoint,p,i,d,direction)
PID freshFanPID(&tempAverage, &freshFanPWM, &tempDesired,freshFanP,freshFanI,freshFanD, P_ON_M, REVERSE);

void setup() {

  #ifdef vfd_display
    //start vfd
    vfd.begin(9600); // set up serial port for 9600 baud
    delay(500); // wait for vfd to boot up
    vfd.write(VFDCTRL_CLEAR_DISPLAY_HOME_CURSOR); //clear screen
  #endif

  //pins
  pinMode(selectPin,INPUT_PULLUP);             //mode button w/pull-up
  pinMode(upPin,INPUT_PULLUP);                 //up button w/ pull-up
  pinMode(downPin,INPUT_PULLUP);               //down button w/pull-up
  pinMode(freshFanPin,OUTPUT);                 //cooling fan signal
  pinMode(mixFanPin,OUTPUT);                   //mixing fan signal
  pinMode(caseLedPin,OUTPUT);                  //lighting signal
  pinMode(spotlightPin,OUTPUT);                //spotlight signal
  pinMode(heaterPin,OUTPUT);                   //heater signal
  #ifdef octoprintControl
    pinMode(octoprintHeatPin,INPUT);           //octoprint heater signal
    pinMode(octoprintCoolPin,INPUT);           //octoprint cooling signal
  #endif

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

  #ifdef vfd_display
    //Splash screen
    //         ("12345678901234567890");
    vfd.print(F("  Smart Enclosure   "));
    vfd.print(F("   by: Truglodite   "));
    delay(splashTime);  //a window of time to release a "hold on boot" button
  #endif
  #ifndef vfd_display
    //start lcd
    lcd.begin(16,2);
    lcd.clear();
    lcd.setCursor(0,0);                          //shamelessness... :P
    //         ("1234567890123456");
    lcd.print(F("Smart Enclosure "));
    lcd.setCursor(0,1);
    lcd.print(F(" by: Truglodite "));
    delay(splashTime);  //window of time to release a "hold on boot" button
  #endif

  //start PIDs
  freshFanPID.SetSampleTime(freshFanPIDsampleTime);
  freshFanPID.SetMode(AUTOMATIC);

  #ifdef octoprintControl  // initialize Octoprint signals
    octoprintHeat = digitalRead(octoprintHeatPin);
    octoprintCool = digitalRead(octoprintCoolPin);
  #endif
}

void loop() {
  currentMillis = millis();  //get the time for this loop
  readButtons();
  updateTemps();
  freshFanPID.Compute();
  updateOutputs();
  if(currentMillis - lastDisplayTime > displayPeriod) {
    #ifdef vfd_display
     updateVFD();
    #endif
    #ifndef vfd_display
     updateLCD();
    #endif
  }

  #ifdef octoprintControl
    // time to read pins
    if(currentMillis - lastOctoprintRead > octoprintUpdateRate) {
      bool octoprintHeatTemp = digitalRead(octoprintHeatPin);
      bool octoprintCoolTemp = digitalRead(octoprintCoolPin);
      // heat and/or cool input has changed
      if(octoprintHeatTemp != octoprintHeat || octoprintCoolTemp != octoprintCool)  {
        octoprintHeat = octoprintHeatTemp; // update the inputs...
        octoprintCool = octoprintCoolTemp;
        octoprintUpdateFlag = 1;  // set the update flag
      }
    }
    // one of the inputs has changed
    if(octoprintUpdateFlag)  {
      // We currently in a GUI input mode
      if(mode == 2 || mode == 3 || mode == 4 || mode == 5 || mode == 6) {
        // do nothing, manual input ALWAYS discards op changes
        return;
      }     // after this, we must be in either auto or cool mode
      // OP sent new 'auto no-heat' mode (low heat, low cool)
      else if(!octoprintCool && !octoprintHeat) {
        if(mode == 1) {  // we were in cooling mode
          freshFanPID.SetMode(AUTOMATIC);
        } // cooling mode
        mode = 0; // auto mode
        heaterControl = 0; //turn off heater
        heaterTimerStarted = 0; //re-enable heater start flag for next time
      }
      // op sent new cool signal
      else if(octoprintCool) {
        mode = 1; // cooling mode
        freshFanPID.SetMode(MANUAL);
        heaterControl = 0; //turn off heater
        heaterTimerStarted = 0; //re-enable heater start flag for next time
      }
      // OP sent new heat signal, and heater is not on
      else if(octoprintHeat && !heaterControl) {
        if(mode == 1) {  // we were in cooldown mode
          freshFanPID.SetMode(AUTOMATIC);
        }
        mode = 0;  // auto mode (in case we are in cooldown)
        heaterControl = 1;  // ...with heater function
        heaterHours = heaterHoursDefault;  //reset timer count
        heaterSwitchTime = currentMillis;  //to make sure we don't have a false heater timeout
        heaterTimerStarted = 0; //set flag to restart timer
      }
      octoprintUpdateFlag = 0;  // done processing, data is marked as stale
    }
  #endif
}  //enaloop... ;)

//update Dallas temperature sensor buffers (non-blocking)
void updateTemps()  {
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
       tempAverage = 0;  //calc new average temps...
       temp2Average = 0;
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
        if(heaterHours < 0) {  //stay within range ...heaterHours is unsigned
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
}// End of readButtons()

// update outputs routine (non-blocking) ////////////////////////////////////
void updateOutputs(void)  {
  //cooldown mode
  if(mode == 1)  {
    heaterStatus = 0;  //turn off heater
    if(tempAverage < tempCooldown - tempCooldownHysteresis)  {  //cool enough, turn everything off
      analogWrite(freshFanPin,0);  //fresh fan off
      freshFanStarted = false;  //update fan startup flag
      mixFanPWM = 0;               //mix fan off
      mixFanStarted = false;  //raise started flag
    }
    else if(tempAverage > tempCooldown + tempCooldownHysteresis) {  //not cool enough, cooling off
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
      //first loop with heater flag on, and the timer is enabled
      if(!heaterTimerStarted && heaterHours) {
        heaterTimerStarted = 1;
        heaterStartTime = currentMillis;
      }
      //heater is off, temp too low, and it's been off long enough
      if(!heaterStatus && tempAverage <= tempDesired - heaterTempOffset - heaterHysteresis && currentMillis - heaterSwitchTime >= heaterMinSwitchTime)  {
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
      else if(heaterStatus && tempAverage >= tempDesired - heaterTempOffset + heaterHysteresis && currentMillis - heaterSwitchTime >= heaterMinSwitchTime) {
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

}  //end of updateOutputs()

#ifdef vfd_display
// update VFD routine (non-clearing, non-blocking)/////////////////////////////
void updateVFD(void)  {
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
}// End of updateVFD()
#endif

#ifndef vfd_display
// update LCD routine (non-clearing, non-blocking)/////////////////////////////
void updateLCD(void)  {
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
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Auto T:XX.X/XXdC"
    //          "1234567890123456";
    if(heaterControl) {
      lcd.print(F("Heat T:"));
    }
    else  {
      lcd.print(F("Auto T:"));
    }
    lcd.print(tempAveString);
    lcd.print(F("/"));
    lcd.print(tempDesiredString);
    lcd.print(char(223)); //degree symbol (hex DF)
    lcd.print(F("C"));

    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //          "Rm:XX.XdC F:XXX%"
    //          "1234567890123456";
    lcd.print(F("Rm:"));
    lcd.print(temp2AveString);
    lcd.print(char(223)); //degree symbol (hex DF)
    if(!heaterStatus) {  //heater is off
      lcd.print(F("C F:"));
      lcd.print(fanString);
      lcd.print(F("%"));
    }
    else  {  //heater is running
      lcd.print(F("C   Heat"));
    }
    return;
  }

  //Cooldown mode
  else if(mode == 1) {
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Cool T:XX.X/XXdC"
    //          "1234567890123456";
    lcd.print(F("Cool T:"));
    lcd.print(tempAveString);
    lcd.print(F("/"));
    lcd.print(tempCooldownString);
    lcd.print(char(223)); //degree symbol (hex DF)
    lcd.print(F("C"));

    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //          "Room:XX.XdC Cool"
    //          "Room:XX.XdC Idle"
    //          "1234567890123456";
    lcd.print(F("Room:"));
    lcd.print(temp2AveString);
    lcd.print(char(223)); //degree symbol (hex DF)
    lcd.print(F("C"));
    if(freshFanStarted)  {
      lcd.print(F(" Cool"));
    }
    else  {
      lcd.print(F(" Idle"));
    }
    return;
  }

  //mix fan setting mode
  else if(mode == 2) {
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Mixing Fan Speed"
    //          "1234567890123456";
    lcd.print(F("Mixing Fan Speed"));

    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //          "  Speed: XXX%   "
    //          "1234567890123456";
    lcd.print(F("  Speed: "));
    lcd.print(mixFanString);
    lcd.print(F("%   "));
    return;
  }

  //Case Led setting mode
  else if(mode == 3) {
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Case LED Bright "
    //          "1234567890123456";
    lcd.print(F("Case LED Bright "));
    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //          "Brightness: XXX%"
    //          "1234567890123456";
    lcd.print(F("Brightness: "));
    lcd.print(caseLedString);
    lcd.print(F("%"));
    return;
  }
  //Heater setting mode
  else if(mode == 4) {
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "Case LED Bright "
    //          "1234567890123456";
    lcd.print(F("Heater Function:"));
    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //            "Brightness: XXX%"
    //            "1234567890123456";
    if(heaterControl) {
      lcd.print(F("       ON       "));
    }
    else  {
      lcd.print(F("       OFF      "));
    }
    return;
  }
  //Spotlight setting mode
  else if(mode == 5) {
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "1234567890123456";
    lcd.print(F("Spotlight Bright"));
    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //          "Brightness: XXX%"
    //          "1234567890123456";
    lcd.print(F("Brightness: "));
    lcd.print(spotlightString);
    lcd.print(F("%"));
    return;
  }
  //Heater Timer setting mode
  else if(mode == 6) {
    lcd.setCursor(0,0); //move to line 0, column 0
    //Mode, actual, & set temperatures
    //          "  Heater Timer  "
    //          "1234567890123456";
    lcd.print(F("  Heater Timer  "));
    lcd.setCursor(0,1); //move to line 1, column 0
    //fan & heater output %'s
    //          "1234567890123456";
    lcd.print(F("      "));
    lcd.print(heaterHoursString);
    lcd.print(F("hrs     "));
    return;
  }
  else {  //no man's land
    lcd.setCursor(0,0); //move to line 0, column 0
    //          "12345678901234567890";
    lcd.print(F("Undefined Mode Error"));
    lcd.setCursor(0,1); //move to line 1, column 0
    //          "1234567890123456";
    lcd.print(F("             :-("));
    return;
  }
}// End of updateLCD()
#endif
