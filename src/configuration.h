// configuration.h
// by: Truglodite
// updated:
//
// User configuration for printer_enclosure.ino
//
///////////////////////////////////////////////////////////////////////////////

// Uncomment if your room and enclosure temperatures are swapped.
//#define swapDallas

// Uncomment if using octoprint triggering
#define octoprintControl
#ifdef octoprintControl
  // Note, octoprint input changes will trigger a mode change ONLY IF they
  // occur during an automated mode (auto, cooldown, and heating). IE, if a
  // user is changing a setting in a GUI input mode, the OP command will be
  // ignored, and the user's end settings will carry on until one of the pins
  // changes again.
  #define octoprintHeatPin  A0  // triggers 'heater on' when high
  #define octoprintCoolPin  A1  // triggers 'cooling on' when high
  // triggers 'auto on' when both heat and cool pins are low
  #define octoprintUpdateRate  3000  // millis between reading op input pins
#endif

// Uncomment if using a 2x20 IEE VFD display
#define vfd_display
#ifdef vfd_display
  #include "ieeVfdCommands.h"
  #include <SoftwareSerial.h>
  #define VFD_RX       8    // not used, but required for softserial
  #define VFD_TX       12   // connect to VFD serial rx (pin 14)
#endif

// A 1602 i2c LCD display is default
// defaults... A4 = SDA, A5 = SCL
#ifndef vfd_display
  #include <LiquidCrystal_I2C.h>
  #include <Wire.h>
#endif

// Pinout /////////////////////////////////////////////////////////////////////
#define dallasPin    2    // To Dallas data pins (requires one 4k7 pullup)
#define freshFanPin  3    // to cooling fan mosfet
#define selectPin    4    // buttons are momentary N.O. (internal pullups)
#define upPin        5
#define downPin      6
#define mixFanPin    9    // to mixing fan mosfet
#define caseLedPin   10   // to case light mosfet
#define spotlightPin 11   // to spotlight LED driver (LDD-350LL)
#define heaterPin    13   // to heater ssr input

// General prefs ///////////////////////////////////////////////
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
#define heaterHysteresis 0.5 //Celsius hysteresis to use for heater
#define heaterMixFanDelay 30000 //millis to keep mixing fan on after heater turns off (>heaterMinSwitchTime)
#define heaterMixFanPWM 255 //pwm value for mix fan while heater is on (255)
#define heaterMaxOffTime 35000 //millis of heater off time to allow before shutting down heating function (heatsoak timer... >heaterMixFanDelay)
#define heaterHoursIncrement 1 //hrs to increment heater timer
#define heaterHoursMax 48 //max hrs allowed for heater timer (any int... 48hrs should cover most prints)
#define heaterHoursDefault 2

#define tempDefault  44.0   //bootup desired Celsius setting (50)
#define tempMinSetting      20.0   //min desired t allowed(20)
#define tempMaxSetting      50.0   //max desired t allowed (50). Steppers may fail at >50C.
#define tempCooldownDefault 24.0 //default cooldown temperature (30)
#define tempCooldownHysteresis 1.0 //hysteresis for fans in cooldown mode
#define tempIncrement 2.0   //celsius change per up/down push (2)

#define dallasResolution 12 //dallas bit resolution (4,8, or 12)
#define sizeOfBuffer 3    //the number of t readings to store for averaging (1 = disable, larger buffers use more memory
                          //and may result in laggy PID feedback, which may increase overshoot)
#define tempPeriod   800  //millis to wait for a requested temp (>760 for 12bit)

//PID tuning parameters... will depend on many aspects of your setup. Use ZN or other manual tuning methods.
//The default PID's work well with my enclosure using a specific assortment of hardware, so they
//may be a good starting point for tuning most setups.
#define freshFanPIDsampleTime 200  //200ms should work for most... be aware that PID values scale to sample time
double freshFanP=    10;  //the pid's must be set whether or not you use bang bang
double freshFanI=    1;   //default p,i,d = 10,5,1
double freshFanD=    1;
