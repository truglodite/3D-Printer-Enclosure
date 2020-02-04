// ieeVfdCommands.h
// by: truglodite
// updated: 6/6/2019
//
// Control bytes for the IEE VFD in "LCD" or "Intel" mode (IEE S036x2_N manual,
// hex converted to decimal). Most of these codes have been tested on an
// IEE 036X2-122-09220 VFD (AKA 05464ASSY35119-02A). Other IEE vfd's are
// likely to work.
////////////////////////////////////////////////////////////////////////////////
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
// ?? 2nd byte?
const uint8_t VFDCTRL_SET_CR_LF_DEFINITIONS = 55; //2 byte command...
/* Follow with the definition byte
 * bits LFin   CRin
 * --------------------------------
 * 0    LF     CR     <---default
 * 1    LF+CR  CR
 * 2    LF     CR+LF
 * 3    LF+CR  CR+LF
 */

// Underbar on/off, applies to 036X2–151–05240 & 036X2-160-05440 only.
// static const char VFDCTRL_UNDERBAR_ON = 56;
// static const char VFDCTRL_UNDERBAR_OFF = 57;

const uint8_t VFDCTRL_SELECT_RIGHT_TO_LEFT_DATA_ENTRY = 58;
const uint8_t VFDCTRL_SELECT_LEFT_TO_RIGHT_DATA_ENTRY = 59;  //default
const uint8_t VFDCTRL_SCREEN_SAVER_ON = 60;
const uint8_t VFDCTRL_SCREEN_SAVER_OFF = 61; //default
const uint8_t VFDCTRL_SELF_TEST_EXECUTE = 62;
const uint8_t VFDCTRL_SELF_TEST_TERMINATE = 63;
