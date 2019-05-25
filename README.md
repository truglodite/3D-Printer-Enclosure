# 3D-Printer-Enclosure
### Arduino PID heated/cooled 3d printer enclosure controller w/ IEE VFD.

Version 9 adds a heater timer feature.

## Features:
* Displays: enclosure temp, room temp, cooling fan %, heater status, cooldown status
* Simple 3 button navigation (up/down/select) w/ long press sub-menus
* PID+PWM Cooling Fan Control w/ Cooldown Mode
* PWM Mixing Fan Control w/ Manual Override
* Fan kickstart
* Anti fan stall (obeys minimum fan speeds)
* Heater Control (with hysteresis + min switching time)
* Heater temp offset (prevents heating/cooling "battles")
* Heat Soak Monitor (Auto Heater Off)
* Heater timer (for cold climates/weaker heaters, when heat soak does not occur)
* Case LED PWM Control
* Spotlight LED PWM Control
* Cooldown Mode
* Dallas swap #define (no messing with 1-wire addresses)
* Several other #defines to make customizing & tuning easier

## Hardware:
 - 2x Dallas 1-wire temp sensors (enclosure and room temps)
 - 20x2 IEE serial VFD display (softserial used... easily ported to other serial displays)
 - 3x logic level MOSFETs (for the 2 fan outputs and 1 LED output)
 - 2-3 or more 12V fans (at least 1 for fresh air, 1 for mixing, and 1 for the heater if used)
 - LEDs for case lighting
 - High powered LED spotlight (Luxeon Star or similar)
 - Power LED driver (Meanwell LDD-350LL or similar 5V PWM dimmable device)
 - 3x momentary NO toggle buttons (6mm square type if using my printed VFD enclosure)
 - 5VDC/110VAC 40A SSR (to switch the heater)
 - ~250W personal space heater (120VAC ceramic type preferred)

## Operation-----------------------------------------------------------------------

### Auto & AutoH mode:
Boots to auto mode, with heater function disabled. In auto mode, the fresh fan controlled by PID to maintain
tempDesired. If heating is enabled, it is hysteresis controlled to heat the enclosure up to tempDesired
minus heaterTempOffset, plus or minus heaterTempHysteresis. The heater function automatically turns off
if heaterMaxOffTime is elapsed since the heater was turned off. This is an easy way to determine if the
enclosure is heat soaked, then heating is automatically disabled to prevent continued heating after a print
is finished. If the heater timer feature is used, the heater will turn off after the set hours has elapsed.
The LCD displays "Auto" when heater feature is disabled, or "AutoH" when heater is enabled,
the actual/set temperatures, room temp, and the fan output % (or "Heating").Short pressing the Up/down buttons
will adjust tempDesired, and the select button will switch to cooldown mode.

### Cooldown mode:
In cooldown mode, the heater remains off and fresh fan spins 100% until
the temperature falls below tempCooldown. Hysteresis is used to prevent frequent fan cycling
in cooldown mode. Also, in this mode the display shows "Cool",
actual/set temperatures, Room temp, and "Cooling" or "Idle" depending on fan state.
Short pressing the up/down buttons will adjust tempCooldown, and short pressing  the
select button will switch to auto mode.

### Mixing fan control:
The mixing fan runs at a manually set speed if that speed is greater than the fresh
air fan speed or minimum mix fan speed. If the fresh air fan speed exceeds the manual speed setting,
the manual mixing fan speed is overridden, and the mixing fan will run at the fresh air fan
speed. The mixing fan turns 100% on whenever the heater is on, and stays on for heaterMixFanDelay
millis after the heater turns off.

### Fan kickstart & anti-stall:
All fans have a kickstart feature, which initially sends extra power to the fans to get
them moving. Also no fans will startup (or continue to buzz) unless their input pwm
is enough to keep them spinning. This combination of kickstart and minimum PWM
control reduces PID reduced delay and overshoot, and prevents unnecessary buzzing/heating
of non-moving fans. StartupPWM and MinPWM compiler defines can be used to tune behavior
of both fresh and mixing fans. Adjust them as needed to match your choice of fans.

### Changing manual mix fan speed:
Long pressing (>2sec default) the up button switches to the manual mixing fan speed
setting. Press select to return to auto or autoh mode.

### Changing LED brightness:
Long pressing the down button switches to the case led setting. In case
led brightness mode, pressing up or down will adjust the case LED brightness.
Press select to advance to the spotlight setting menu, where up/down adjusts
spotlight brightness. Press select again to return to auto or autoh mode.

### Changing Heater settings (if a heater is used):
Long pressing select goes to the heater enable/disable setting. Pressing up enables heating,
and down disables it. This setting will persist until the next reboot, if the heat soak
timer is triggered, or if the heater timer has elapsed. Short pressing select then goes
to the heater timer setting. If left at 0hrs, the heater timer function is disabled (thus the
heater function will remain on until heat soak is sensed, it is manually turned off, or a reboot).
Pressing up/down increases/decreases the timer by 1 hour, from 0 to heaterTimerMax.
Any time the heater hours is adjusted, the heat timer is restarted at t=0. Short pressing select
again returns to auto or autoh mode.

*While working in any long press menu, the fans and heater will be controlled as if in auto or autoh mode.*

*The desired & cooldown temperature adjustments are limited to max/min values for safety.
Change them as needed to better suit your setup.*
