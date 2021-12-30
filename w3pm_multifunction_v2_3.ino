/*
  Menu driven ultifunction program to allow an Arduino Nano(or UNO), a Si5351 clock generator
  board (25 MHz), a DS3231 real time clock board RTC), and a monochrome 0.96" 128x64 serial I2C
  display board to perform as a VFO, WSPR source, frequency counter, and a date/time clock.
  The DS3231 RTC is used to provide a high accuracy frequency and time reference.

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Copyright (C) 2019,  Gene Marcus W3PM GM4YRE

  22 October, 2019

  30 December 2021  v2_3 Newer baatches of Si5351's default to spread spectrum if
                         not set to disable. This version includes code to disable
                         the spread spectrum mode.     

  The code was sucessfully compiled using Arduino 1.8.10


  COMPILER NOTE:    Uses libraries SSD1306Ascii by Bill Greiman and PinChangeInterrupt
                    by NicoHood
                    Load from: Sketch > Include Library > Manage Libraries
                    (Windows users will find the menu bar on top of the active window.  Linux and MAC users
                    will find the menu bar at the top of the primary display screen.)


  ------------------------------------------------------------------------------------------------------------
  Five pushbuttons are used to control the functions.
  Pushbutton pin allocations follow:
  ------------------------------------------------------------------------------------------------------------
        WSPR               VFO            COUNTER    CLOCK        CLOCK SET                DATE SET
  PB1     N/A         Decrease Frequency    N/A       N/A     Time sync / Set Hour          Set Day
  PB2   ON / OFF      Increase Frequency    N/A       N/A         Set Minute                Set Month
  PB3  Band Select       Band Select        N/A       N/A           N/A                     Set Year
  PB4     N/A         Resolution Select     N/A       N/A     Hold to change time       Hold to change date

  menu - exit function and return to menu

  -------------------------------------------------------------------------------------------------------------
  Nano Digital Pin Allocations follow:
  -------------------------------------------------------------------------------------------------------------
  D0
  D1
  D2  1pps input from RTC
  D3  rotary encoder pin A
  D4  rotary encoder pin B
  D5  2.5 MHz input from Si5351 CLK0 pin / counter input
  D6  pushButton 1
  D7  pushButton 2
  D8  pushButton 3
  D9  pushButton 4
  D10 menu
  D11 VFO CW offset control
  D12
  D13
  A0/D14 XmitLED
  A1/D15
  A2/D16 SecLED
  A3/D17
  A4/D18 Si5351 & OLED SDA
  A5/D19 Si5351 & OLED SCL

*/


//----------------------------------------------------------------------------------------------------
//                         WSPR configuration data follows:
//----------------------------------------------------------------------------------------------------
// Enter your callsign below:
// Note: Upper or lower case characters are acceptable. Do not use compound callsigns e.g. W3PM/4
char call2[13] = "A9AA";
char locator[5] = "AA99";

// Enter TX power in dBm below: (e.g. 0 = 1 mW, 10 = 10 mW, 20 = 100 mw, 30 = 1 W)
// Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43
const int  ndbm = 13;

// Enter desired transmit interval below:
// Avoid entering "1" as the unit will transmit every 2 minutes not allowing for
// autocal updates between transmissions
const byte TXinterval = 2; // Transmit interval (e.g. 3 = transmit once every 3rd 2 minute transmit slot)

// Enter desired transmit offset below:
// Transmit offset frequency in Hz. Range = 1400-1600 Hz (used to determine TX frequency within WSPR window)
long TXoffset = 1533;

// Enter In-band frequency hopping option:
// This option ignores TXoffset above and frequency hops within the WSPR 200 Hz wide window
// Before using this option be sure the system is calibrated to avoid going outside band edges.
// In-band transmit frequency hopping? (true = Yes, false = No)
const bool FreqHopTX = false;


/*
  ----------------------------------------------------------------------------------------------------
                         VFO configuration data follows:
  ----------------------------------------------------------------------------------------------------
  VFO Band Select frequency format:

  Column 1 = CLK1 VFO start frequency in Hz
  Column 2 = CLK2 LO frequency in Hz
  Column 3 = LCD display arithmetic operation
             0 = no change
             1 = Add CLK1 and CLK2
             2 = Subtract CLK1 from CLK2

  Example: {5286500,8998500,1}, will result in
  OLED display: 14.285000 MHz
  CLK1 output:   5.286500 MHz
  CLK2 output:   8.998500 MHz

  Enter any number of Band Select frequencies.
  Use (0,0,0) as the last entry.

  Restrict frequency entries to > 110 kHz and < 112.5 MHz

  Enter Band Select frequencies below:
*/
const unsigned long Freq_array [] [3] = {
  {  7030000, 0, 0      },    // CLK1=7.030 MHz, CLK2=0 MHz, Display=7,030.000 kHz
  {   136000, 0, 0      },
  {   474200, 0, 0      },
  {  1810000, 0, 0      },
  {  3560000, 0, 0      },
  {  7040000, 0, 0      },
  { 10106000, 0, 0      },
  { 14060000, 0, 0      },
  { 18096000, 0, 0      },
  { 21060000, 0, 0      },
  { 24960000, 0, 0      },
  { 28060000, 0, 0      },
  { 5286500, 8998500, 1 },    // CLK1:5.2865 MHz, CLK2:8.9985 MHz, Display:14,285.000 kHz
  { 5016500, 9001500, 2 },    // CLK1:5.0165 MHz, CLK2:9.0015 MHz, Display:3,985.00 kHz

  (0, 0, 0)
};



//  ----------------------------------------------------------------------------------------------------
//                         CW offset variable follows:
//  ----------------------------------------------------------------------------------------------------
// The Offset variable controls the CW frequency offset. When the Arduino pin D10 is held LOW the
// programmed offset (Hz) is added or subtracted. Any positive or negative value of Hz can be
// used to program the frequency offset.
//
// Enter offset frequency (Hz) below:
int fOffset = -600;        // -600 Hz offset



//  ----------------------------------------------------------------------------------------------------
//                        Counter gate time variable follows:
//  ----------------------------------------------------------------------------------------------------
// The gate time may be changed to smooth out gate time jitter or provide for a X10 pre-scaler
// Enter gate time (Seconds) below:
byte gateTime = 1;


//----------------------------------------------------------------------------------------------------
//                         Constants and variables data follows:
//----------------------------------------------------------------------------------------------------
// include the library code:
#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "PinChangeInterrupt.h"

// Set up MCU pins
#define InterruptPin             2
#define pushButton1              6
#define pushButton2              7
#define pushButton3              8
#define pushButton4              9
#define menu                    10
#define XmitLED                 14
#define SecLED                  13
#define encoderPinA              3
#define encoderPinB              4
#define Offset                  11

// Set DS3231 I2C address
#define DS3231_addr           0x68

// Set sI5351A I2C address
#define Si5351A_addr          0x60

// Define OLED address
#define I2C_ADDRESS           0x3C

// initialize oled display
SSD1306AsciiAvrI2c oled;

// Define Si5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define SSC_EN                 149
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183

typedef struct {  // Used in conjunction with PROGMEM to reduce RAM usage
  char description [4];
} descriptionType;

// configure variables
byte fStepcount, offsetFlag = 0, WSPRband, band, gateCount, sym[170];
byte c[11], symt[170], tcount3;
String resolution = "1 Hz  ";
bool toggle = false, WSPR_toggle = false, timeSet_toggle = false, StartCalc = false, startFlag = false, VFOstartFlag = true, LEDtoggle;
bool WSPRflag = false, suspendUpdateFlag = false;
volatile bool fired = false;
int nadd, nc, n, ntype, TXcount;
int p, m, dBm, AutoCalFactor[50], CFcount;
int hours, minutes, seconds, xday, xdate, xmonth, xyear, startCount = 0;
char call1[7], grid4[5];
unsigned long XtalFreq = 25000000, tcount = 2, WSPR_symbol_interval = 681, time_now = 0, VFOtime_now;
unsigned long mult = 0, Freq_1, Freq_2, fStep = 1, InputFreq;
unsigned long t1, ng, n2, cc1, n1;
volatile long rotaryCount = 0;
const descriptionType daysOfTheWeek [7] PROGMEM = { {"SUN"}, {"MON"}, {"TUE"}, {"WED"}, {"THU"}, {"FRI"}, {"SAT"},};
const descriptionType monthOfTheYear [12] PROGMEM = { {"JAN"}, {"FEB"}, {"MAR"}, {"APR"}, {"MAY"}, {"JUN"}, {"JUL"}, {"AUG"}, {"SEP"}, {"OCT"}, {"NOV"}, {"DEC"},};
const byte daysInMonth [] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };


// Load WSPR dial frequencies (TXoffet will be added to obtain actual transmit frequency)
const long DialFreq [] =
{
  136000      ,
  474200      ,
  1836600     ,
  3568600     ,
  3592600     ,
  5287200     ,
  7038600     ,
  10138700    ,
  14095600    ,
  18104600    ,
  21094600    ,
  24924600    ,
  28124600    ,
  50293000    ,
  0
};

// Load WSPR symbol frequency offsets
const long OffsetFreq[4] = {
  -22, // 0 Hz
  -7,  // 1.4648 Hz
  7,   // 2.9296 Hz
  22   // 4.3944 Hz
};

// Load WSPR sync vector values
const byte  SyncVec[162] PROGMEM = {
  1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
  1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
};


//***********************************************************************
// This interrupt is used for Si5351 25MHz crystal frequency calibration
// Called every second by from the DS3231 RTC 1PPS to Nano pin D2
//***********************************************************************
void Interrupt()
{
  tcount++;
  if (tcount == 4)                                    // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                       //Clock on falling edge of pin 5
  }
  else if (tcount == 44)                              //Total count is = XtalFreq x 4 - stop counting
  {
    TCCR1B = 0;                                       //Turn off counter
    unsigned long TempFreq = (mult * 0x10000 + TCNT1);//Calculate corrected XtalFreq
    TCNT1 = 0;                                        //Reset count to zero
    mult = 0;
    tcount = 0;                                       //Reset the seconds counter

    // The following is an averageing alorithm used to smooth out DS3231 1pps jitter
    // Note: The upper and lower frequecny constraint prevents autocalibration data corruption
    //       which may occur if the COUNTER switch is in the wrong position.
    if (TempFreq > 99988000L & TempFreq < 100012000L) // Check bounds
    {
      int N = 12;
      if (CFcount == N)
      {
        CFcount = 0;
        StartCalc = true;
      }
      if (StartCalc == false)                       // This is the initial warm-up period
      {
        AutoCalFactor[CFcount] = 100000000UL - TempFreq;
        if (suspendUpdateFlag == true) CFcount++;   // Don't update corrected crystal frequency while transmitting

        else
        {
          XtalFreq = TempFreq;                      // Update corrected crystal frequency when not transmitting
          CFcount++;
        }
      }
      else                                          // Warm-up period completed, go into averaging mode
      {
        long temp = 0;
        AutoCalFactor[CFcount] = 100000000UL - TempFreq;
        for (int i = 0; i < N ; i++) temp = temp + AutoCalFactor[i];
        if (suspendUpdateFlag == false) XtalFreq = (100000000UL - (round(temp / N))); //Average the correction factors and update crystal frequency
        CFcount++;
      }
    }
  }
  LEDtoggle = !LEDtoggle;
  if (LEDtoggle == true) digitalWrite(SecLED, HIGH);
  else digitalWrite(SecLED, LOW);
}


//*********************************************************************
// counterInterrupt interrupt is used as a one second frequency counter
// clock gate
// Called every second by from the DS3231 RTC 1PPS to Nano pin D2
//*********************************************************************
void counterInterrupt()
{
  if (gateCount == gateTime)
  {
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    gateCount = 0;
    InputFreq = round(InputFreq / gateTime);
  }
  else
  {
    gateCount++;
    InputFreq = mult * 0x10000 + TCNT1;          //Calculate frequency
  }
}


//******************************************************************
// Timer 1 overflow intrrupt vector.
// Called upon counter overflow from timer 1
//******************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                        //Increment multiplier
  TIFR1 = (1 << TOV1);                           //Clear overlow flag
}


//******************************************************************
// Rotary encoder intrrupt vector.
// Credit for this algorithm goes to Nick Gammon http://www.gammon.com.au
// Called upon by VFO function when rotary encoder turns
// "increment" may require tweaking depending upon the encoder type
//******************************************************************
void doEncoder()
{
  static boolean ready;
  static unsigned long lastFiredTime;
  static byte pinA, pinB;

  // wait for main program to process it
  if (fired) return;

  byte newPinA = digitalRead (encoderPinA);
  byte newPinB = digitalRead (encoderPinB);

  // Forward is: LH/HH or HL/LL
  // Reverse is: HL/HH or LH/LL

  // so we only record a turn on both the same (HH or LL)

  if (newPinA == newPinB)
  {
    if (ready)
    {
      long increment = 1 * fStep;

      // As the encoder turns faster, the count goes up more
      unsigned long now = millis ();
      unsigned long interval = now - lastFiredTime;
      lastFiredTime = now;

      if (interval < 10)
        increment = 5 * fStep;
      else if (interval < 20)
        increment = 3 * fStep;
      else if (interval < 50)
        increment = 2 * fStep;

      if (newPinA == HIGH)
      {
        if (pinA == LOW)
          Freq_1 -= increment;
        else
          Freq_1 += increment;
      }
      else
      {
        if (pinA == LOW)
          Freq_1 += increment;
        else
          Freq_1 -= increment;
      }
      fired = true;
      ready = false;
    }  // end of being ready
  }  // end of completed click
  else
    ready = true;

  pinA = newPinA;
  pinB = newPinB;
}



//----------------------------------------------------------------------------------------------------
//                         Initial sketch setup follows:
//----------------------------------------------------------------------------------------------------
void setup()
{

  Serial.begin(9600);

  Wire.begin(1);                                  // join I2C bus (address = 1)
  si5351aStart();

  // Set up DS3231 for 1 Hz squarewave output
  // Needs only be written one time provided DS3231 battery is not removed
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E);
  Wire.write(0);
  Wire.endTransmission();

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer1 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag

  // Inititalize 1 Hz interrupt input pin
  pinMode(InterruptPin, INPUT);
  digitalWrite(InterruptPin, HIGH);         // internal pull-up enabled

  // Set pin 2 for external 1 Hz interrupt input
  attachInterrupt(digitalPinToInterrupt(InterruptPin), Interrupt, FALLING);

  // Make XtalFreq compatable with autocalibrate correction variable
  XtalFreq *= 4;

  // Allow time for autocalibration and Si5351 warm up for upon initial start
  // The first WSPR transmission will be delayed by one transmit interval
  TXcount = TXinterval + 1;

  // Set up push buttons
  pinMode(pushButton1, INPUT);
  digitalWrite(pushButton1, HIGH);       // internal pull-up enabled
  pinMode(pushButton2, INPUT);
  digitalWrite(pushButton2, HIGH);       // internal pull-up enabled
  pinMode(pushButton3, INPUT);
  digitalWrite(pushButton3, HIGH);       // internal pull-up enabled
  pinMode(pushButton4, INPUT);
  digitalWrite(pushButton4, HIGH);       // internal pull-up enabled
  pinMode(Offset, INPUT);
  digitalWrite(Offset, HIGH);            // internal pull-up enabled
  pinMode(menu, INPUT);
  digitalWrite(menu, HIGH);              // internal pull-up enabled


  // Set up rotary encoder
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);       // internal pull-up enabled
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // internal pull-up enabled


  // Set up LEDs
  pinMode(XmitLED, OUTPUT);              // Use with dropping resistor on pin D14
  digitalWrite(XmitLED, LOW);
  pinMode(SecLED, OUTPUT);               // Uses on board Nano (Uno) LED
  digitalWrite(SecLED, LOW);

  // Retrieve stored data
  EEPROM.get (10, band);                 // Get stored VFO band
  if (band > 20 | band < 0) band = 0;    //Ensures valid EEPROM data - if invalid will default to band 0

  EEPROM.get (12, fStepcount);           // Get stored VFO step resolution
  if (fStepcount > 6 | fStepcount < 0) fStepcount = 0;    //Ensures valid EEPROM data - if invalid will default to 0
  EEPROM.get (14, Freq_1);               // Get stored VFO CLK1 frequency
  if (Freq_1 > 1200000000 | Freq_1 < 0) Freq_1 = 10000000;  //Ensures valid EEPROM data - if invalid will default to 10 MHz

  // Get stored WSPR band
  WSPRband = EEPROM.read(0);
  if (WSPRband > 12 | WSPRband < 0) WSPRband = 1; //Ensures valid EEPROM data - if not valid will default to 630m

  // Enable the Si5351
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010);   // Enable CLK0 and CLK2 - disable CLK1

  // Set CLK2 to 2.5 MHz for WSPR frequency stabilization. This will be disable if VFO is selected and CLK2 = 0.
  si5351aSetFreq(SYNTH_MS_2, 25000000);           // Frequency x 10 for higher resolution Multisynth alorithm

  // Set CLK0 to 2.5 MHz for autocalibration
  si5351aSetFreq(SYNTH_MS_0, 25000000);           // Frequency x 10 for higher resolution Multisynth alorithm

  // Ensure WSPR input data is upper case
  for (int i = 0; i < 13; i++)if (call2[i] >= 97 && call2[i] <= 122)call2[i] = call2[i] - 32;
  for (int i = 0; i < 7; i++)if (locator[i] >= 97 && locator[i] <= 122)locator[i] = locator[i] - 32;

  // Set oled font size and type, then display startup menu message
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);

  // Check for calibration function
  if (digitalRead(pushButton4) == LOW) Calibrate(); //Used to enter calibration function upon reset

  // Setup OLED initial display
  oled.clear();
  oled.setCursor(0, 4);
  oled.println(F("PB3: scroll"));
  oled.print(F("PB2: select"));

  // Generate unique WSPR message for your callsign, locator, and power
  wsprGenCode();

  // Store time reference for sketch timing i.e. delays, EEPROM writes, display toggles
  // The Arduino "delay" function is not used because of critical timing conflicts
  time_now = millis();
}



//----------------------------------------------------------------------------------------------------
// Loop starts here:
// Loops consecutively to check MCU pins for activity
//----------------------------------------------------------------------------------------------------
void loop()
{
  // Loop through the  menu rota until a selection is made
  while (digitalRead(pushButton2) == HIGH & startFlag == false)
  {
    if (digitalRead(pushButton3) == LOW)
    {
      altDelay(500);
      startCount = startCount + 1;;
      if (startCount > 5) startCount = 0;
    }

    switch (startCount) {
      case 0:
        oled.setCursor(0, 0);
        oled.println(F("   WSPR  "));
        break;
      case 1:
        oled.setCursor(0, 0);
        oled.println(F("    VFO  "));
        break;
      case 2:
        oled.setCursor(0, 0);
        oled.println(F("  COUNTER"));
        break;
      case 3:
        oled.setCursor(0, 0);
        oled.println(F("   CLOCK "));
        break;
      case 4:
        oled.setCursor(0, 0);
        oled.println(F(" SET CLOCK"));
        break;
      case 5:
        oled.setCursor(0, 0);
        oled.println(F(" SET DATE "));
        break;
    }
  }

  if (startFlag == false)
  {
    oled.clear();
    oled.set1X();
    startFlag = true;
  }
  switch (startCount) {
    case 0: // Begin WSPR processing
      WSPR();
      break;
    case 1: // VFO
      VFO();
      break;
    case 2: // Frequency counter
      // Set pin 2 for external 1 Hz interrupt input
      attachInterrupt(digitalPinToInterrupt(InterruptPin), counterInterrupt, FALLING);
      Counter();
      break;
    case 3: // Display the clock
      DisplayClock2();
      break;
    case 4: // RTC clock adjustment begins here:
      adjClock();
      break;
    case 5: // RTC date adjustment begins here:
      setDATE();
      break;
      // Selection is made at this point, now go to the selected function
  }

  if (digitalRead (menu) == LOW)
  {
    startFlag = false;
    Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1
    detachPCINT(digitalPinToPCINT(encoderPinA));
    detachPCINT(digitalPinToPCINT(encoderPinB));
    VFOstartFlag = true;
    tcount = 0;
    attachInterrupt(digitalPinToInterrupt(InterruptPin), Interrupt, FALLING);
    oled.clear();
    oled.setCursor(0, 4);
    oled.println(F("PB3: scroll"));
    oled.print(F("PB2: select"));
  }
}



//******************************************************************
// WSPR function follows:
// Used to select band, determine if it is time to transmit, and
// blink WSPR related LEDs.
// Called called by "switch case0" in loop
//******************************************************************
void WSPR()
{
  setfreq();
  WSPRstatus();
  getTime();

  // Update for new day
  if (hours == 0 & minutes == 0 & seconds == 0) WSPRstatus();

  // Determine if it is time to transmit
  if (bitRead(minutes, 0) == 0 & seconds == 0 & WSPRflag == true) transmit();

  //Turn WSPR tranmit inhibit on and off during non-transmit period
  if (digitalRead(pushButton2) == LOW & suspendUpdateFlag == false)
  {
    altDelay(500);
    WSPR_toggle = !WSPR_toggle;
    if (WSPR_toggle == false)
    {
      WSPRflag = false;
      WSPRstatus();
    }
    else
    {
      WSPRflag = true;
      //Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK2 and CLK0 - disable CLK1
      WSPRstatus();
    }
    altDelay(1000);
  }

  // Band selection during non-transmit period
  if (digitalRead(pushButton3) == LOW & suspendUpdateFlag == false)
  {
    altDelay(500);
    WSPRband++;
    if (DialFreq [WSPRband] == 0)WSPRband = 0;
    EEPROM.write(0, WSPRband);
    setfreq();
    altDelay(500);
    WSPRstatus();
  }

  // Update time during non-transmit period
  if (suspendUpdateFlag == false) displayClock();
}


//******************************************************************
// VFO function follows:
// Used to select band, change frequency, and control offset function
// Called called by "switch case1" in loop
//******************************************************************
void VFO()
{
  if (VFOstartFlag == true)
  {
    // This is a one time call at startup to set resolution to EEPROM stored value
    // and to correct the frequency
    setResolution();     // Call the set resolution subroutine
    Freq_1 -= fStep;     // Decrease CLK1 by frequency step
    VFOsetfreq();        // Set and display new frequency
    Si5351_write(CLK_ENABLE_CONTROL, 0b00000100); // Turn ON CLK0 & CLK1
    si5351aSetFreq(SYNTH_MS_1, Freq_1 * 10);
    Freq_2 = Freq_array [band] [1];         // Get value of Freq_2 for stored band
    if (Freq_2 != 0)
    {
      Si5351_write(CLK_ENABLE_CONTROL, 0b00000000); // Turn ON all CLKs
      Freq_2 = Freq_array [band] [1];               // Load CLK2
      si5351aSetFreq(SYNTH_MS_2, Freq_2 * 10);      // Set CLK2 frequency
    }
    attachPCINT(digitalPinToPCINT(encoderPinA), doEncoder, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderPinB), doEncoder, CHANGE);
    VFOstartFlag = false;
  }

  if (tcount == 1)
  {
    // Update the SI5351A after every autocal correction
    si5351aSetFreq(SYNTH_MS_1, Freq_1 * 10);
    if (Freq_2 > 0)
    {
      si5351aSetFreq(SYNTH_MS_2, Freq_2 * 10);
    }
  }

  // If rotary encoder was active update VFO frequency
  if (fired)
  {
    VFOsetfreq();                                   //Update and display new frequency
    resDisplay();                                   //Update resolution display and set display timer
    fired = false;
  }

  // The frequency step resolution selection begins here:
  if (digitalRead(pushButton4) == LOW)
  {
    altDelay(500);
    fStepcount++;
    if (fStepcount > 6)fStepcount = 0;
    EEPROM.put(12, fStepcount);
    setResolution();                                // Call the set resolution subroutine
  }

  // Band selection begins here:
  if (digitalRead(pushButton3) == LOW)
  {
    altDelay(500);
    band = band + 1;                                 // Increment band selection
    EEPROM.put(10, band);
    if (Freq_array [band] [0] == 0)band = 0;         // Check for end of frequency array
    if (Freq_array [band] [1] == 0)                  // Is CLK2 = 0?
    {
      Si5351_write(CLK_ENABLE_CONTROL, 0b00000100);  // Turn OFF CLK2
    }
    else
    {
      Si5351_write(CLK_ENABLE_CONTROL, 0b00000000);  // Turn ON CLK2
      Freq_2 = Freq_array [band] [1];                // Load CLK2 frequency
      si5351aSetFreq(SYNTH_MS_2, Freq_2 * 10);       // Set CLK2 frequency
    }
    Freq_1 = Freq_array [band] [0];                  // Load CLK1 frequency
    VFOsetfreq();                                    // Display and set CLK1 frequency
  }

  // Frequency offset algorithm begins here:
  if (digitalRead(Offset) == LOW & offsetFlag == 0) // Check for offset pin 10 LOW
  {
    offsetFlag = 1;                                  // Set flag
    Freq_1 += fOffset;                               // Add offset frequency
    oled.setCursor(108, 4);                          // Display a "*" on the display
    oled.print(F("*"));
    VFOsetfreq();                                    // Display and set CLK1 frequency + offset
  }

  if (digitalRead(Offset) == HIGH & offsetFlag == 1) // Check for offset pin 10 HIGH
  {
    offsetFlag = 0;                                   // Reset flag
    Freq_1 -= fOffset;                                // Subtract the offset frequency
    oled.setCursor(108, 4);                           // Clear the "*" on the display
    oled.print(F(" "));
    VFOsetfreq();                                     // Display and set CLK1 frequency - offset
  }

  // Frequency Up/Down pushbutton algorithm begin here:
  if (digitalRead(pushButton2) == LOW) // Check for frequency increase pushbutton 2 LOW
  {
    altDelay(500);
    // Increase frequency by the selected frequency step
    Freq_1 += fStep;                                 // Increase CLK1 by frequency step
    VFOsetfreq();                                    // Set and display new frequency
    resDisplay();                                    // Call the resolution display subroutine
  }

  if (digitalRead(pushButton1) == LOW) // Check for frequency decrease pushbutton 1 LOW
  {
    altDelay(500);
    // Decrease frequency by the selected frequency step and check for 1-80 MHz limits
    Freq_1 -= fStep;                                // Decrease CLK1 by frequency step
    VFOsetfreq();                                   // Set and display new frequency
    resDisplay();                                   // Call the resolution display subroutine
  }

  displayClock();

  // per Arduino data: "EEPROM.put" only writes if Freq_1 is changed, however,
  // a 10 second delay is used to prevent unnecessary EEPROM updates during
  // short frequency changes
  if (millis() > VFOtime_now + 10000)
  {
    EEPROM.put(14, Freq_1);
    VFOtime_now = millis();
  }

}


//******************************************************************
// Frequency counter function follows:
// Used to count frequency applied to Nano pin D5
// The DS3231 RTC 1pps signal applied to Nano pin D2 is used as
// the counter gate.
// Called called by "switch case2" in loop
//******************************************************************
void Counter()
{
  oled.setCursor(0, 0);
  if (gateCount == 0) DisplayFrequency(InputFreq);
  displayClock();
}



//******************************************************************
// Date/Time Clock display function follows:
// Used to display date and time as well as DS3231 chip temperature
// Note: displayed temperature is usually a few degrees warmer than
//       ambient temperature
// Called called by "switch case3" in loop
//******************************************************************
void DisplayClock2()
{
  //oled.setFont(fixed_bold10x15);
  oled.set2X();
  oled.setCursor(3, 2);

  getTime();

  if (hours < 10) oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));

  if (minutes < 10) oled.print(F("0"));
  oled.print(minutes);

  oled.set1X();
  oled.setCursor(0, 6);
  oled.print(call2);
  oled.setCursor(96, 6);

  if (seconds < 10) oled.print(F("0"));
  oled.print(seconds);

  if (millis() > time_now + 10000)
  {
    toggle = !toggle;
    time_now = millis();
  }

  if (toggle == true)
  {
    getDate();
    oled.setCursor(0, 0);
    descriptionType oneItem;
    //   memcpy_P (&oneItem, &daysOfTheWeek [now.dayOfTheWeek()], sizeof oneItem);
    memcpy_P (&oneItem, &daysOfTheWeek [xday], sizeof oneItem);
    oled.print (oneItem.description);
    //    oled.print(daysOfTheWeek[now.dayOfTheWeek()]);
    oled.print(F(" "));

    if (xdate < 10) oled.print(F("0"));
    oled.print(xdate);
    oled.print(F(" "));
    memcpy_P (&oneItem, &monthOfTheYear [xmonth - 1], sizeof oneItem);
    oled.print (oneItem.description);
    oled.print(F("  "));
  }

  else
  {
    Wire.beginTransmission(DS3231_addr);         // Start I2C protocol with DS3231 address
    Wire.write(0x11);                            // Send register address
    Wire.endTransmission(false);                 // I2C restart
    Wire.requestFrom(DS3231_addr, 2);            // Request 11 bytes from DS3231 and release I2C bus at end of reading
    int temperature_msb = Wire.read();           // Read temperature MSB
    int temperature_lsb = Wire.read();           // Read temperature LSB

    temperature_lsb >>= 6;

    // Convert the temperature data to F and C
    // Note: temperature is DS3231 temperature and not ambient temperature
    //       correction factor of 0.88 is used
    oled.setCursor(0, 0);
    int DegC = int(((temperature_msb * 100) + (temperature_lsb * 25)) * 0.88);
    oled.print(DegC / 100);
    //oled.print(temp);
    oled.print(F("."));
    oled.print((DegC % 100) / 10);
    oled.print(F("C "));
    int DegF = int(DegC * 9 / 5 + 3200);
    oled.print(DegF / 100);
    oled.print(F("."));
    oled.print((DegF % 100) / 10);
    oled.print(F("F"));
  }
}


//******************************************************************
// Clock adjust function follows:
// Used to adjust the system time
// Note: Quickly depressing pushbutton 1 will synchronize the clock
//       to the top of the minute "0"
//       Holding down pushbutton 4 while depressing pushbutton 1
//       will advance the hours.
//       Holding down pushbutton 3 while depressing pushbutton 2
//       will advance the minutes.
// Called called by "switch case4" in loop
//******************************************************************
void adjClock()
{
  getTime();

  // Display current time
  int tempHour = hours;
  int tempMinute = minutes;
  int tempSecond = seconds;
  oled.setCursor(0, 0);
  oled.print(F("  "));
  int a = hours;
  int b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);
  oled.print(F(":"));
  a = minutes;
  b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);
  oled.print(F(":"));
  a = seconds;
  b = a % 10;
  a = a / 10;
  oled.print(a);
  oled.print(b);

  // Display legend
  oled.setCursor(0, 2);
  oled.print(F("Hold PB4"));
  oled.setCursor(0, 4);
  oled.print(F("PB1: Hours"));
  oled.setCursor(0, 6);
  oled.print(F("PB2: Minutes"));

  // Start one button time synchronization routine
  if (digitalRead(pushButton1) == LOW)
  {
    if (tempSecond > 30) tempMinute++;
    if (tempMinute > 59) tempMinute = 0;
    tempSecond = 0;
    updateTime(tempSecond, tempMinute, tempHour);
    altDelay(500);
  }
  timeSet_toggle = false;

  // Start set time routine
  while (digitalRead(pushButton4) == LOW)
  {
    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(500);
      tempHour++;
      if (tempHour > 23) tempHour = 0;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton2) == LOW)
    {
      altDelay(500);
      tempMinute++;
      if (tempMinute > 59) tempMinute = 0;
      timeSet_toggle = true;
    }

    // Display set time
    oled.setCursor(0, 0);
    oled.print(F("  "));
    int a = tempHour;
    int b = a % 10;
    a = a / 10;
    oled.print(a);
    oled.print(b);
    oled.print(F(":"));
    a = tempMinute;
    b = a % 10;
    a = a / 10;
    oled.print(a);
    oled.print(b);
    oled.print(F(":00"));
  }

  // Update time if change is made
  if (timeSet_toggle == true)
  {
    int tempSecond = 0;
    updateTime(tempSecond, tempMinute, tempHour);
    timeSet_toggle = false;
  }
}


//******************************************************************
// Date adjust function follows:
// Used to adjust the system date
// Note:
//       Holding down pushbutton 4 while depressing pushbutton 1
//       will advance the date.
//       Holding down pushbutton 4 while depressing pushbutton 2
//       will advance the month.
//       Holding down pushbutton 4 while depressing pushbutton 3
//       will advance the year.
// Called called by "switch case5" in loop
//******************************************************************
void setDATE()
{
  getDate();

  int updateDate = xdate;
  int updateMonth = xmonth;
  int updateYear = xyear;

  // Display currently stored date
  oled.setCursor(0, 0);
  oled.print(updateDate);
  oled.print(F(" "));
  descriptionType oneItem;
  memcpy_P (&oneItem, &monthOfTheYear [updateMonth - 1], sizeof oneItem);
  oled.print (oneItem.description);
  oled.print(F(" "));
  oled.print(updateYear);

  // Display legend
  oled.setCursor(0, 2);
  oled.print(F("Hold PB4"));
  oled.setCursor(0, 4);
  oled.print(F("1:Date"));
  oled.setCursor(0, 6);
  oled.print(F("2:Month 3:Yr"));

  // Start update
  while (digitalRead(pushButton4) == LOW)
  {
    if (digitalRead(pushButton1) == LOW)
    {
      altDelay(500);
      updateDate++;
      if (updateDate > 31) updateDate = 0;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton2) == LOW)
    {
      altDelay(500);
      updateMonth++;
      if (updateMonth > 12) updateMonth = 1;
      timeSet_toggle = true;
    }

    if (digitalRead(pushButton3) == LOW)
    {
      altDelay(500);
      updateYear++;
      if (updateYear > 30) updateYear = 19;
      timeSet_toggle = true;
    }

    // Display updates
    oled.setCursor(0, 0);
    //if (xdate < 10) oled.print(F("0"));
    oled.print(updateDate);
    oled.print(F(" "));
    descriptionType oneItem;
    memcpy_P (&oneItem, &monthOfTheYear [updateMonth - 1], sizeof oneItem);
    oled.print (oneItem.description);
    oled.print(F(" "));
    oled.print(updateYear);
  }

  // Save data if updated
  if (timeSet_toggle == true)
  {
    // Convert DEC to BCD
    updateDate = ((updateDate / 10) * 16) + (updateDate % 10);
    updateMonth = ((updateMonth  / 10) * 16) + (updateMonth % 10);
    updateYear = ((updateYear / 10) * 16) + (updateYear % 10);

    // Write the data
    Wire.beginTransmission(DS3231_addr);
    Wire.write((byte)0x04); // start at register 4
    Wire.write(updateDate);
    Wire.write(updateMonth);
    Wire.write(updateYear);
    Wire.endTransmission();
  }
}



//******************************************************************
// Step resolution function follows:
// Sets the frequency step resolution
// Called by the VFO function
//******************************************************************
void setResolution()
{
  switch (fStepcount)
  {
    case 0:
      fStep = 1;
      resolution = "1 Hz  ";
      break;
    case 1:
      fStep = 10;
      resolution = "10 Hz  ";
      break;
    case 2:
      fStep = 100;
      resolution = "100 Hz ";
      break;
    case 3:
      fStep = 1000;
      resolution = "1 kHz  ";
      break;
    case 4:
      fStep = 10000;
      resolution = "10 kHz ";
      break;
    case 5:
      fStep = 100000;
      resolution = "100 kHz";
      break;
    case 6:
      fStep = 1000000;
      resolution = "1 MHz  ";
      break;
  }
  resDisplay();
}


//******************************************************************
// Step resolution display function follows:
// Display the frequency step resolution
// Called by the VFO and step resolution functions
//******************************************************************
void resDisplay()
{
  oled.setCursor(0, 2);
  oled.print(F("Res= "));
  oled.setCursor(54, 2);
  oled.print(resolution);
}


//******************************************************************
// WSPR status function follows:
// Displays WSPR transmitter ON/OFF status
// Called by the WSPR function
//******************************************************************
void WSPRstatus()
{
  if (WSPRflag == false)
  {
    oled.setCursor(84, 0);
    oled.print(F("OFF"));
  }
  else
  {
    oled.setCursor(84, 0);
    oled.print(F(" ON"));
  }
}


//******************************************************************
// displayClock follows:
// Displays the time and date during WSPR function operation
//
// Called by loop()
//******************************************************************
void displayClock()
{
  oled.setCursor(12, 4);
  getTime();

  if (hours < 10) oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));

  if (minutes < 10) oled.print(F("0"));
  oled.print(minutes);
  oled.print(F(":"));

  if (seconds < 10) oled.print(F("0"));
  oled.print(seconds);

  if (millis() > time_now + 2000)
  {
    if (toggle == true)
    {
      tcount3++;
      if (tcount3 > 5) tcount3 = 0;
      oled.setCursor(0, 6);
      oled.print(F("            "));
    }
    toggle = !toggle;
    time_now = millis();
  }

  if (toggle == false)
  {
    if (startCount != 0) tcount3 = 5; // If not WSPR only display date
    oled.setCursor(0, 6);
    switch (tcount3)
    {
      case 0: // Display WSPR TX offset
        oled.print (F("Offset:"));
        oled.print(TXoffset);
        break;

      case 1: // Display grid square
        oled.print(F("Grid: "));
        oled.print (locator);
        break;

      case 2: // Display power
        oled.print(F("PWR: "));
        oled.print (ndbm);
        oled.print(F("dBm "));
        break;

      case 3: // Display transmit interval
        oled.print(F("TXint: "));
        oled.print (TXinterval);
        break;

      case 4: // Display frequency hopping YES/NO
        oled.print(F("TXhop: "));
        if (FreqHopTX == false) oled.print(F("NO"));
        else oled.print(F("YES"));
        break;

      case 5: // Display date dats
        getDate();
        oled.setCursor(0, 6);
        descriptionType oneItem;
        memcpy_P (&oneItem, &daysOfTheWeek [xday], sizeof oneItem);
        oled.print (oneItem.description);
        oled.print(F(" "));

        if (xdate < 10) oled.print(F("0"));
        oled.print(xdate);
        oled.print(F(" "));
        memcpy_P (&oneItem, &monthOfTheYear [xmonth - 1], sizeof oneItem);
        oled.print (oneItem.description);
        oled.print(F("  "));
        break;
    }
  }
}



//******************************************************************
// Set WSPR frequency function follows:
// Calculates the frequency data to be sent to the Si5351 clock generator
// and displays the frequency on the olded display
// Called by WSPR and transmit function
//******************************************************************
void setfreq()
{
  unsigned long  Freq_temp = DialFreq [WSPRband] + TXoffset; // Temporarily store Freq_1

  // Print callsign and frequency to the display
  oled.setCursor(0, 0);
  oled.println(call2);
  oled.setCursor(0, 2);
  DisplayFrequency(Freq_temp);
}


//******************************************************************
// Set VFO frequency function follows:
// Calculates the frequency data to be sent to the Si5351 clock generator
// and displays the frequency on the olded display
// Called by the VFO function
//******************************************************************
void VFOsetfreq()
{
  unsigned long  Freq_temp = Freq_1; // Temporarily store Freq_1
  switch (Freq_array [band] [2])     // Get math function from frequency array
  {
    case 1:  // If math function is 1 then add Freq_1 and Freq_2 for display
      Freq_temp = Freq_1 + Freq_2;
      break;
    case 2:  // If math function is 2 then subtract Freq_1 from Freq_2 for display
      Freq_temp = Freq_2 - Freq_1;
      break;
  }

  oled.setCursor(0, 0);
  char buf[11];
  DisplayFrequency(Freq_temp);
  si5351aSetFreq(SYNTH_MS_1, Freq_1 * 10); // Set CLK1 frequency
}

long DisplayFrequency(long frequency)
{
  char buf[11];
  if (frequency >= 1000000L)
  {
    int MHz = int(frequency / 1000000L);
    int kHz = int ((frequency - (MHz * 1000000L)) / 1000L);
    int Hz = int (frequency % 1000);
    snprintf(buf, sizeof(buf), "%2u,%03u,%03u", MHz, kHz, Hz);
  }

  else if (frequency >= 1000L & frequency < 1000000L)
  {
    int kHz = int (frequency / 1000L);
    int Hz = int (frequency % 1000L);
    snprintf(buf, sizeof(buf), "%6u,%03u", kHz, Hz);
  }
  else if (frequency < 1000L)
  {
    int Hz = int (frequency);
    snprintf(buf, sizeof(buf), "%10u", Hz);
  }
  oled.print(buf);
}


//******************************************************************
// Alternate delay function follows:
// altDelay is used because the command "delay" causes critical
// timing errors.
//
// Called by all functions
//******************************************************************
unsigned long altDelay(unsigned long delayTime)
{
  time_now = millis();
  while (millis() < time_now + delayTime) //delay 1 second
  {
    __asm__ __volatile__ ("nop");
  }
}


//******************************************************************
// WSPR transmit algorithm follows:
// Configures WSPR data and timing and then sends data to the Si5351 clock timer
//
// Called by WSPR function
//******************************************************************
void transmit()
{
  TXcount++;
  altDelay(1000);
  if (TXcount > TXinterval) TXcount = 1;
  if (TXcount != TXinterval) return;
  if (FreqHopTX == true) // Enables in-band TX frequency hopping in incremental 10Hz steps
  {
    TXoffset = TXoffset + 10;
    if (TXoffset > 1550) TXoffset = 1440; //Bounds are 40 Hz from band edge to allow for frequency error
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000100); // Enable CLK0 CLK1 - disable CLK2 (to enhance Si5351 short term freq stability)
  suspendUpdateFlag = true;
  setfreq();
  oled.setCursor(0, 4);
  oled.print(F(" XMITTING "));
  digitalWrite(XmitLED, HIGH);
  unsigned long currentTime = millis();
  for (int count = 0; count < 162; count++)
  {
    unsigned long timer = millis();
    si5351aSetFreq(SYNTH_MS_1, (DialFreq [WSPRband] + TXoffset) * 10L + OffsetFreq[sym[count]]);
    while ((millis() - timer) <= 682UL) {
      __asm__("nop\n\t");
    };
  }
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000010); // Enable CLK0 CLK2 - disable CLK1 (drift compensation)
  si5351aSetFreq(SYNTH_MS_2, 25000000);        // CLK2 is enabled to balance thermal drift between transmissions
  suspendUpdateFlag = false;
  digitalWrite(XmitLED, LOW);
}


//***********************************************************************
// WSPR message generation algorithm follows:
// Configures the unique WSPR message based upon your callsign, location, and power.
// Note: Type 2 callsigns (e.g. GM/W3PM) are not supported in this version
//
// Called by sketch setup.
//***********************************************************************
void wsprGenCode()
{
  for (int i = 0; i < 7; i++) {
    call1[i] = call2[i];
  };
  for (int i = 0; i < 5; i++) {
    grid4[i] = locator[i];
  };
  packcall();
  packgrid();
  n2 = ng * 128 + ndbm + 64;
  pack50();
  encode_conv();
  interleave_sync();
}


//******************************************************************
void packpfx()
{
  char pfx[3];
  int Len;
  int slash;

  for (int i = 0; i < 7; i++)
  {
    call1[i] = 0;
  };
  Len = strlen(call2);
  for (int i = 0; i < 13; i++)
  {
    if (call2[i] == 47) slash = i;
  };
  if (call2[slash + 2] == 0)
  { //single char add-on suffix
    for (int i = 0; i < slash; i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    nadd = 1;
    nc = int(call2[slash + 1]);
    if (nc >= 48 && nc <= 57) n = nc - 48;
    else if (nc >= 65 && nc <= 90) n = nc - 65 + 10;
    else if (nc >= 97 && nc <= 122) n = nc - 97 + 10;
    else n = 38;
    ng = 60000 - 32768 + n;
  }
  else if (call2[slash + 3] == 0)
  {
    for (int i = 0; i < slash; i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    n = 10 * (int(call2[slash + 1]) - 48) +  int(call2[slash + 2]) - 48;
    nadd = 1;
    ng = 60000 + 26 + n;
  }
  else
  {
    for (int i = 0; i < slash; i++)
    {
      pfx[i] = call2[i];
    };
    if (slash == 2)
    {
      pfx[2] = pfx[1];
      pfx[1] = pfx[0];
      pfx[0] = ' ';
    };
    if (slash == 1)
    {
      pfx[2] = pfx[0];
      pfx[1] = ' ';
      pfx[0] = ' ';
    };
    int ii = 0;
    for (int i = slash + 1; i < Len; i++)
    {
      call1[ii] = call2[i];
      ii++;
    };
    packcall();
    ng = 0;
    for (int i = 0; i < 3; i++)
    {
      nc = int(pfx[i]);
      if (nc >= 48 && nc <= 57) n = nc - 48;
      else if (nc >= 65 && nc <= 90) n = nc - 65 + 10;
      else if (nc >= 97 && nc <= 122) n = nc - 97 + 10;
      else n = 36;
      ng = 37 * ng + n;
    };
    nadd = 0;
    if (ng >= 32768)
    {
      ng = ng - 32768;
      nadd = 1;
    };
  }
}

//******************************************************************
void packcall()
{
  // coding of callsign
  if (chr_normf(call1[2]) > 9)
  {
    call1[5] = call1[4];
    call1[4] = call1[3];
    call1[3] = call1[2];
    call1[2] = call1[1];
    call1[1] = call1[0];
    call1[0] = ' ';
  }

  n1 = chr_normf(call1[0]);
  n1 = n1 * 36 + chr_normf(call1[1]);
  n1 = n1 * 10 + chr_normf(call1[2]);
  n1 = n1 * 27 + chr_normf(call1[3]) - 10;
  n1 = n1 * 27 + chr_normf(call1[4]) - 10;
  n1 = n1 * 27 + chr_normf(call1[5]) - 10;
}

//******************************************************************
void packgrid()
{
  // coding of grid4
  ng = 179 - 10 * (chr_normf(grid4[0]) - 10) - chr_normf(grid4[2]);
  ng = ng * 180 + 10 * (chr_normf(grid4[1]) - 10) + chr_normf(grid4[3]);
}

//******************************************************************
void pack50()
{
  // merge coded callsign into message array c[]
  t1 = n1;
  c[0] = t1 >> 20;
  t1 = n1;
  c[1] = t1 >> 12;
  t1 = n1;
  c[2] = t1 >> 4;
  t1 = n1;
  c[3] = t1 << 4;
  t1 = n2;
  c[3] = c[3] + ( 0x0f & t1 >> 18);
  t1 = n2;
  c[4] = t1 >> 10;
  t1 = n2;
  c[5] = t1 >> 2;
  t1 = n2;
  c[6] = t1 << 6;
}

//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc )
{
  char cc = 36;
  if (bc >= '0' && bc <= '9') cc = bc - '0';
  if (bc >= 'A' && bc <= 'Z') cc = bc - 'A' + 10;
  if (bc >= 'a' && bc <= 'z') cc = bc - 'a' + 10;
  if (bc == ' ' ) cc = 36;

  return (cc);
}


//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc = 0;
  int cnt = 0;
  int cc;
  unsigned long sh1 = 0;

  cc = c[0];

  for (int i = 0; i < 81; i++) {
    if (i % 8 == 0 ) {
      cc = c[bc];
      bc++;
    }
    if (cc & 0x80) sh1 = sh1 | 1;

    symt[cnt++] = parity(sh1 & 0xF2D05351);
    symt[cnt++] = parity(sh1 & 0xE4613C47);

    cc = cc << 1;
    sh1 = sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while (li != 0)
  {
    po++;
    li &= (li - 1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii, ij, b2, bis, ip;
  ip = 0;

  for (ii = 0; ii <= 255; ii++) {
    bis = 1;
    ij = 0;
    for (b2 = 0; b2 < 8 ; b2++) {
      if (ii & bis) ij = ij | (0x80 >> b2);
      bis = bis << 1;
    }
    if (ij < 162 ) {
      sym[ij] =  pgm_read_byte(&SyncVec[ij]) + 2 * symt[ip];
      ip++;
    }
  }
}
// WSPR messgage generation algorithm ends here


//******************************************************************
//  Si5351 Multisynch processing follows:
//  Generates the Si5351 clock generator frequence message
//  Note: This is a modified version of the algorithm I used in the
//        GPS VFO sketch. This algorith trades bandwith for the frequency
//        resolution required for WSPR.
//
//  Called by sketch setup, WSPR and VFO functions
//******************************************************************
void si5351aSetFreq(int synth, unsigned long long freq)
{
  unsigned long long CalcTemp, d;
  unsigned long b, c, p1, p2, p3;
  bool HFflag;
  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  if (freq < 5000000)
  {
    HFflag = false;
    freq = freq * 4;
  }
  else
  {
    HFflag = true;
  }

  d = (XtalFreq * 360LL) / 4; // 360 is derived from 900MHz * 10 / 25 MHz}
  unsigned long long a = d / freq;
  CalcTemp = round(d % freq);
  CalcTemp *= c;
  CalcTemp /= freq ;
  b = CalcTemp;  // Calculated numerator

  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  Si5351_write(synth, 0xFF);
  Si5351_write(synth + 1, 0xFF);
  if (HFflag == true)
  {
    Si5351_write(synth + 2, (p1 & 0x00030000) >> 16);
  }
  else
  {
    Si5351_write(synth + 2, 0b00100000 ^ ((p1 & 0x00030000) >> 16));
  }
  Si5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(synth + 4, (p1 & 0x000000FF));
  Si5351_write(synth + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(synth + 7, (p2 & 0x000000FF));

}


//******************************************************************
//  Si5351 initialization functions follow:
//  Used to set up Si53351 clock generator parameters and PLLs
//
//  Called by sketch setup
//******************************************************************
void si5351aStart()
{
  // Initialize Si5351A
  Si5351_write(XTAL_LOAD_CAP, 0b11000000);     // Set crystal load to 10pF
  Si5351_write(CLK_ENABLE_CONTROL, 0b00000111); // Turn off all outputs
  Si5351_write(CLK0_CONTROL, 0b00001111);      // Set PLLA to CLK0, 8 mA output
  Si5351_write(CLK1_CONTROL, 0b00001111);      // Set PLLA to CLK1, 8 mA output
  Si5351_write(CLK2_CONTROL, 0b00101111);      // Set PLLB to CLK2, 8 mA output
  Si5351_write(PLL_RESET, 0b10100000);         // Reset PLLA and PLLB
  Si5351_write(SSC_EN,0b00000000);             // Disable spread spectrum

  // Set PLLA and PLLB to 600 MHz
  unsigned long  a, b, c, p1, p2, p3;

  a = 36;           // Derived from 900/25 MHz
  b = 0;            // Numerator
  c = 0xFFFFF;      // Denominator derived from max bits 2^20

  // Refer to Si5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to PLL registers
  Si5351_write(SYNTH_PLL_A, 0xFF);
  Si5351_write(SYNTH_PLL_A + 1, 0xFF);
  Si5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  Si5351_write(SYNTH_PLL_A + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  Si5351_write(SYNTH_PLL_B, 0xFF);
  Si5351_write(SYNTH_PLL_B + 1, 0xFF);
  Si5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  Si5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  Si5351_write(SYNTH_PLL_B + 5, 0xF0 | ((p2 & 0x000F0000) >> 16));
  Si5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  Si5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));

}


//******************************************************************
//  Time set function follows:
//  Used to set the DS3231 RTC time
//
//  Called by adjClock()
//******************************************************************
void updateTime(int updateSecond, int updateMinute, int updateHour)
{
  // Convert BIN to BCD
  updateSecond = updateSecond + 6 * (updateSecond / 10);
  updateMinute = updateMinute + 6 * (updateMinute / 10);
  updateHour = updateHour + 6 * (updateHour / 10);

  // Write the data
  Wire.beginTransmission(DS3231_addr);
  Wire.write((byte)0); // start at location 0
  Wire.write(updateSecond);
  Wire.write(updateMinute);
  Wire.write(updateHour);
  Wire.endTransmission();
}


//******************************************************************
//  Time update function follows:
//  Used to retrieve the correct time from the DS3231 RTC
//
//  Called by WSPR(), adjClock(), displayClock(), and dispalyClock2
//******************************************************************
void  getTime()
{
  // Send request to receive data starting at register 0
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 3); // request three bytes (seconds, minutes, hours)

  while (Wire.available())
  {
    seconds = Wire.read(); // get seconds
    minutes = Wire.read(); // get minutes
    hours = Wire.read();   // get hours

    seconds = (((seconds & 0b11110000) >> 4) * 10 + (seconds & 0b00001111)); // convert BCD to decimal
    minutes = (((minutes & 0b11110000) >> 4) * 10 + (minutes & 0b00001111)); // convert BCD to decimal
    hours = (((hours & 0b00100000) >> 5) * 20 + ((hours & 0b00010000) >> 4) * 10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)
  }
}


//******************************************************************
//  Date update function follows:
//  Used to retrieve the correct date from the DS3231 RTC
//
//  The day of the week algorithm is a modified version
//  of the open source code found at:
//  Code by JeeLabs http://news.jeelabs.org/code/
//
//  Called by displayClock() and dispalyClock2
//******************************************************************
void  getDate()
{
  int nowDay;
  int nowDate;
  int tempdate;
  int nowMonth;
  int nowYear;

  // send request to receive data starting at register 3
  Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
  Wire.write((byte)0x03); // start at register 3
  Wire.endTransmission();
  Wire.requestFrom(DS3231_addr, 4); // request four bytes (day date month year)

  while (Wire.available())
  {
    nowDay = Wire.read();    // get day (serves as a placeholder)
    nowDate = Wire.read();   // get date
    nowMonth = Wire.read();  // get month
    nowYear = Wire.read();   // get year

    xdate = (((nowDate & 0b11110000) >> 4) * 10 + (nowDate & 0b00001111)); // convert BCD to decimal
    tempdate = xdate;
    xmonth = (((nowMonth & 0b00010000) >> 4) * 10 + (nowMonth & 0b00001111)); // convert BCD to decimal
    xyear = ((nowYear & 0b11110000) >> 4) * 10 + ((nowYear & 0b00001111)); // convert BCD to decimal

    for (byte i = 1; i < xmonth; ++i)
      tempdate += pgm_read_byte(daysInMonth + i - 1);
    if (xmonth > 2 && xyear % 4 == 0)
      ++tempdate;
    tempdate = tempdate + 365 * xyear + (xyear + 3) / 4 - 1;
    xday = (tempdate + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
  }
}


//******************************************************************
// Write I2C data function for the Si5351A follows:
// Writes data over the I2C bus to the appropriate device defined by
// the address sent to it.

// Called by sketch setup, VFO, WSPR, transmit, si5351aSetFreq, and
// si5351aStart functions.
//******************************************************************
uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(Si5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}


//******************************************************************
// Set the DS3231 RTC crystal oscillator aging offset function follows:
// This is effectively the system calibration routine. Small
// capacitors can be swithed in or out within the DS3231 RTC to control
// frequency accuracy. Positive aging values add capacitance to the
// array, slowing the oscillator frequency. Negative values remove
// capacitance from the array, increasing the oscillator frequency.
// One offset count provides about 0.1 ppm change in frequency.

// To enter the calibration function hold down pushbutton 4 and invoke
// a reset. Refer to the calibration notes below.
//******************************************************************

void Calibrate()
{
  int CF, tempCF, busy;
  //oled.setFont(fixed_bold10x15);
  oled.set1X();
  oled.clear();
  oled.print(F("Cal CF ="));
  oled.setCursor(0, 2);
  oled.println(F("PB1 = Down"));
  oled.println(F("PB2 = Up"));
  oled.print(F("reset = Exit"));

  while (digitalRead(pushButton3) == HIGH)
  {
    // send request to receive data starting at register 3
    Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
    Wire.write((byte)0x10); // start at register 0x10
    Wire.endTransmission();
    Wire.requestFrom(DS3231_addr, 1); // request one byte (aging factor)
    while (Wire.available())
    {
      CF = Wire.read();
    }
    if (CF > 127) CF -= 256;
    tempCF = CF;
    oled.setCursor(96, 0);
    oled.print(F("    "));
    oled.setCursor(96, 0);
    oled.println(CF);
    if (digitalRead(pushButton1) == LOW) CF -= 1;
    if (digitalRead(pushButton2) == LOW) CF += 1;

    Wire.beginTransmission(DS3231_addr); // DS3231_addr is DS3231 device address
    Wire.write((byte)0x0F); // start at register 0x0F
    Wire.endTransmission();
    Wire.requestFrom(DS3231_addr, 1); // request one byte to determine DS3231 update status
    while (Wire.available())
    {
      busy = Wire.read();
    }
    busy = bitRead(busy, 2);
    if (CF != tempCF & bitRead(busy, 2) == 0)
    {
      setAgingOffset(CF);
      forceConversion();
    }
    altDelay(500);
  }
}


void setAgingOffset(int offset)
{
  if (offset < 0) offset += 256;

  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x10);
  Wire.write(offset);
  Wire.endTransmission();
}


void forceConversion()
{
  Wire.beginTransmission(DS3231_addr);
  Wire.write(0x0E);

  Wire.write(B00111100);
  Wire.endTransmission();
}

/*******************************************************************************************

                                 CALIBRATION

 *******************************************************************************************


   While the default RTC is already very accurate its accuracy can be pushed even higher
   by adjusting its aging offset register.

   For normal operation calibration is not required. The default 2 parts per million accuracy
   of the RTC will result in an uncertainty of less than +/- 30 Hz on 20 meters.

   If WSPR is used, the time will require synchronization every 7 – 10 days without calibration.
   The re-synchronization timeframe can be stretched to a month or more with calibration.


   There are three ways to perfom DS3231 RTC aging offset calibration:
   1) Measure and adjust the 32 kHz output
   2) Set VFO frequency to 10 MHz and use the frequency delta as the aging offset
   3) Track the time over a period of days.

   A calibration function is provided view the current aging offset and enter a new offset.
   To enter the calibration funtion, hold down pushbutton 4 and invoke a reset. Invoke a
   reset to exit the calibration function.

   IMPORTANT NOTE: When using methods 2 and 3 above, any change will not take place until
                   the auto-calibration algorithm calculates the correction factor for the
                   Si5351A’s 25 MHz clock and the DS3231's temperature compensation algorithm
                   performs its calculation. This may take a minute or two before any
                   change appears. Additionally, the auto-calibration routine uses an
                   averaging alogithm over a few minutes to reduce the effects of 1pps gate
                   time jitter. Any adjustment (other than the 32 KHz method) will be an
                   iterative process and will take some time.


   32 kHz & FREQUENCY COUNTER METHOD:
   Attach a 10 kohm pull-up resistor from the VCC to the 32K pins of the RTC. Attach a high
   resolution accurate counter from the 32K pin to ground.
   Enter the calibration function (see above) and use pushbuttons 1 and 2 to adjust the
   frequency as closely to 32768 Hz as possible.


   VFO & FREQUENCY COUNTER METHOD:
   Set the VFO to 10 Mhz and measure the frequency from CLK1 with an accurate frequency
   counter. Measure to the nearest Hz and subract this number from 10 MHz. This will be
   the aging offset. Enter the calibration function (see above). If this is the first
   time you performed a calibration you should see "CF = 0" on the display. If not, add
   the measured aging factor to the displayed number.  Use pushbuttons 1 and 2 to set the
   device to the aging factor. Invoke a reset to exit.

   Note: Per the DS3231 datasheet at  +25°C,  one LSB (in the offset register) typically
         provides about 0.1ppm change in frequency. At 10MHx 0.1ppm equates to 1 Hz.
         Theoretically, the process described above should produce get you right on
         target. Practically, I found that I had to go back and forth to obtain the
         greatest accuracy. Allow the system a few minutes to stabilize before making
         any adjustments (refer to the note above).


   TIME TRACKING METHOD:
   Sychronize the device's displayed time with an accuate time source such as WWV.
   After a few days compare the displayed time the reference time source. The
   following formula is not exact, but should bring you closer to nominal:

     aging offset = (change in seconds / (173400 x number of days)) x 10^7

   Use a positive integer if the clock is running fast and a negative integer if it is
   running slow.

   Enter the calibration function (see above). If this is the first time you
   performed a calibration you should see "CF = 0" on the display. If not, add the
   measured aging offset to the displayed number.  Use pushbuttons 1 and 2 to set the
   device to the aging factor. Invoke a reset to exit.

 ********************************************************************************************
*/
