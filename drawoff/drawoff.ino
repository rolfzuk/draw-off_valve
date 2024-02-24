#if 1

// 01/27/20214 - REZ Adding support for 5 Volt motor - Relay 2 is connected to the motor RED wire with NC on
//                   relay contact connected to ground and relay coil energized to Normally Open.  Relay 2 is
//                   controlled by pin 31 of the Arduino Board
//
//                   Relay 1 is connected to the motor BLACK wire, again with the unenergized coil the BLACK
//                   wire is held to ground.  Relay 1 is controlled by pin 33 of the Arduino board.
//
//                   In the motor OFF state, both relays are to be unenergized (held to ground) although you can
//                   also turn the motor OFF with both relays energized, although this is not recommended as 
//                   it is not a standard industrial practice (and also consumes energy needlessly.)
//
//                   To cause the motor to open the valve, Relay 1 is to be energized and Relay 2 is to be
//                   unenergized.
//
//                   To cause the motor to close the valve, Relay 2 is to be energized and Relay 1 is to be
//                   unenergized.
//
//                   When the valve is completely open, the signal on the GREEN wire from the motor will go to
//                   ground which can be read on Arduino pin 37.
//
//                   When the valve is completely closed, the signal on the Yellow wire from the motor will go to
//                   ground which can be read on Arduino pin 35.
//
//                   The selection of the 110 Volt motor or the 5 Volt motor is set in the configuration byte, 
//                   OPTIONS_EEPROM_ADDRESS in databit C (a.k.a. 0x1000).  When the databit is low, the motor
//                   is a 5 Volt motor (much, much safer) and when it is high, a 110 Volt AC motor is used.





#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
MCUFRIEND_kbv ID=0x9341;
#include <TouchScreen.h>
#include <Fonts/FreeSans12pt7b.h>
#include <FreeDefaultFonts.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <EEPROM.h>
#include <elapsedMillis.h>

void openValve(int openTime);
void closeValve(int closeTime);
void stopValve(void);


// EEPROM Address Space/Memory Map
#define OPTIONS_EEPROM_ADDRESS  0                 /* short int of 2 bytes */
#define TARGET_TEMPERATURE_EEPROM_ADDRESS 2       /* float of 4 bytes */
#define BAROMETER_CORRECTION_EEPROM_ADDRESS 6     /* float of 4 bytes */
#define RTD_CORRECTION_EEPROM_ADDRESS 10          /* float of 4 bytes */

#define CALIBRATE   3
#define BAROMETRIC  2
#define AUTOMATIC   1
#define MANUAL      0

#define MINPRESSURE 200
#define MAXPRESSURE 1000

#include <SPI.h>
#include <Adafruit_BME280.h>

#define BME_SCK  (52)
#define BME_MISO (51)                           // a.k.a. SDI on Adafruit BME280 board
#define BME_MOSI (50)                           // a.k.a. SDO on Adafruit BME280 board
#define BME_CS   (47)

#define RELAY_SETTLE_TIME 1                     // milliseconds to allow a relay reach new state


#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define BLACK   0x0000

Adafruit_BME280 bme(BME_CS); // hardware SPI

    // The boiling point of water at selected pressures.  The first entry is for
    // 82500 Pascals and every successive table entry is an additional 500 Pascals.
     const float boilingTemp[51] = { 201.905, 202.202, 202.497, 202.791, 203.083,
                                    203.373, 203.661, 203.948, 204.233, 204.516,
                                    204.798, 205.078, 205.356, 205.633, 205.909,
                                    206.078, 206.366, 206.636, 206.906, 207.176,
                                    207.446, 207.716, 207.968, 208.238, 208.508,
                                    208.778, 209.030, 209.300, 209.552, 209.804,
                                    210.074, 210.326, 210.578, 210.830, 211.082,
                                    211.334, 211.586, 211.838, 212.090, 212.342,
                                    212.576, 212.828, 213.080, 213.314, 213.566,
                                    213.800, 214.052, 214.286, 214.520, 214.772,
                                    215.006  };  

    int tableIndex, interpolate, i;
    float boilingPoint;                     // computed boiling point of water
    float displayedBoilingPoint;            // What is currently display on screen. rounded (truncate?)

//elapsedMillis upButtonTimer = 0;          // The one degree increment timer for fast increment
//elapsedMillis downButtonTimer = 0;        // The one degree decrement timer for fast decrement
//elapsedMillis optionButtonTimer = 0;      // Timer to switch to calibration screen
//elapsedMillis calibrationTimer = 0;       // Timer to tell when calibration temperature has stabilized
elapsedMillis automaticManualTimer = 0;     // Timer to allow the comm  buffers to be emptied before switching modes


// int globalOpenTime;                       // The current limit for the openingValveTimer above
// int globalCloseTime;                      // The current limit for the closingValveTimer above
int globalAutomaticManual = AUTOMATIC;
bool globalAutoManualFirstPass = true;
float differential = 0.0;                 // The amount above the boiling point of water to start opening the valve
float boilingWaterCorrection = 0.0;       // The number of degrees Fahrenheit to adjust the RTD value
long barometerCorrection = 0;           
bool updateTargetTemp = true;
bool  down;                               // Is a button pressed?
void setCursor(uint16_t x0, uint16_t y0);
void setTextColor(uint16_t color);
void setTextColor(uint16_t color, uint16_t backgroundcolor);
void setTextSize(uint8_t size);
void setTextWrap(boolean w);

// ALL Touch panels and wiring is DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
//const int XP = 6, XM = A2, YP = A1, YM = 7; //ID=0x9341
//const int TS_LEFT = 907, TS_RT = 136, TS_TOP = 942, TS_BOT = 139;
const int XP = 8, XM = A2, YP = A3, YM = 9;
const int TS_LEFT = 912, TS_RT = 125, TS_TOP = 86, TS_BOT = 906;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Adafruit_GFX_Button up_btn, down_btn, option_btn, open_btn, close_btn, stop_btn;

int powerRelayPin = 43;                 // I/O pin to control the power to the valve
int directionRelayPin  = 39;            // I/O pin to control the direction on the valve
int dcMotorRedPin = 31;                 // I/O pin to control 5V motor relay
int dcMotorBlackPin = 33;               // I/O pin to control 5V motor relay
int dcMotorYellowPinClosed = 35;        // Input pin indicating 5V valve is fully closed when low
int dcMotorGreenPinOpen = 37;           // Input pin indicating 5V valve is fully open when low

float displayedTargetValue = 0.0;       // The last displayed Target Value to see if we need to update screen
float currentValue = 0.0;               // The current value of the sap/syrup (temperature, brix)
float temperature = 0.0;                // The current temperature returned by the thermometer  Used to see if we need to update screen or can we avoid the flicker zzzzzzzzz Change to 0 for production
float alarmSetpoint = 0.2;              // Temperature Alarm Setpoint
float displayedTemperature = 0.0;       // The temperature currently displayed.  Used to see if we need to update screen or can we avoid the flicker zzzzzzzzz Change to 0 for production
int pixel_x, pixel_y;     
short int options;                      // Options enabled for this instrument
                                        //   Bit  Meaning
                                        //    F   Low - Barometer has been calibrated and stored in EEPROM address BAROMETER_CORRECTION_EEPROM_ADDRESS as float
                                        //    E   Low - RTD has been calibrated and stored in EEPROM address RTD_CORRECTION_EEPROM_ADDRESS as float
                                        //    D   Low - Target temperature spread has been set by user and stored in EEPROM address TARGET_TEMPERATURE_EEPROM_ADDRESS as float



// Valve State Machine States
#define  VALVE_AT_POSITION         0
#define  VALVE_CLOSING             1
#define  VALVE_OPENING             2
#define  VALVE_UNKNOWN             3

// Valve State Machine Events
#define  OPEN_COMMAND              0
#define  CLOSE_COMMAND             1
#define  STOP_COMMAND              2
#define  REACHED_POSITION          3



//int      valvePosition = 0;             // The position (rough) of the valve in milliseconds with 0 being closed and 12,000 being completely open
int      valveState;                    // Knows the state of the valve as VALVE_OPEN and VALVE_CLOSED

volatile elapsedMillis openingValveTimer = 0;         // How many milliseconds in opening the valve
int                    openingValveTimerTarget = 0;   // Target value to open the valve
volatile elapsedMillis closingValveTimer = 0;         // How many milliseconds left in closing the valve
int                    closingValveTimerTarget = 0;   // Target value to close the valve


/**********************************************************************************************************
 * 
 *  Finite State Machine for Valve Operation by Timing
 *  
 *  Because of the requirement of the customer, we had no choice on the valve to be used.  By experimental data,
 *  it was determined it took approximately 12 seconds to fully open the valve.  Thus, upon start-up, we close the
 *  valve for 12,000 milliseconds to close the valve.  (The valve protects itself by turning off its power to the
 *  motor when it reaches its end stop in either direction, i.e., fully open or fully closed.  This is why we are
 *  not concerned if the motor runs too long.
 *  
 *  At the initial state we do not know the position of the valve so we issue a 12,000 ms close command to put the
 *  valve in a known position.  By definition, this is the zero position.
 *  
 *  The valve has three states besides the initial state.  These are:
 *  
 *    VALVE_AT_POSITION  
 *    VALVE_CLOSING             
 *    VALVE_OPENING 
 *    VALVE_UNKNOWN
 * 
 * There are four events that work on the valve.  These are:
 * 
 *    OPEN_COMMAND
 *    CLOSE_COMMAND
 *    STOP_COMMAND
 *    REACHED_POSITION
 *    
 *  The state machine keeps track of the valve position by using the elapsedMillis variable type.  
 *  
 *  States/Events       | OPEN_COMMAND | CLOSE_COMMAND | STOP_COMMAND | REACHED_POSITION |
 *  --------------------+--------------+---------------+--------------+------------------|
 * 0  VALVE_AT_POSITION |      2       |      1        |    IGNORE    |      ERROR       |
 *  --------------------+--------------+---------------+--------------+------------------|
 * 1  VALVE_CLOSING            2       |      1        |       0      |        0         |
 *  --------------------+--------------+---------------+--------------+------------------|
 * 2  VALVE_OPENING     |      2       |      1        |       0      |        0         |
 *  --------------------+--------------+---------------+--------------+------------------|
 * 3  VALVE_UNKNOWN     |   Restart    |    Restart    |   Restart    |      Restart     |
 *  --------------------+--------------+---------------+--------------+------------------|
 * 
 */
//zzzzzzzzzzzzzzzzzzzzzzzzz 06/12/2022 TODO: The framework for the Valve State Machine is complete but needs the calls to perform the actions

 void valveFSM(int event, int moveTime = 0)
 {
  if(globalAutomaticManual == AUTOMATIC) 
  {
    Serial.print("Valve State Machine - entered valveState/event: "); Serial.print(valveState, HEX); Serial.print("/"); Serial.println(event, HEX);

    switch (valveState)
    {
      case VALVE_AT_POSITION:
        switch (event)
        {
          case OPEN_COMMAND:
            noInterrupts();
            openValve(100);
            valveState = VALVE_OPENING; 
            interrupts();          
            break;

          case CLOSE_COMMAND:
            if (digitalRead(dcMotorYellowPinClosed) == 0) break;    // Already closed, nothing to do
            noInterrupts();
            closeValve(200);
            valveState = VALVE_CLOSING;
            interrupts();
            break;

          case STOP_COMMAND:
            Serial.print("IGNORE; Valve State Machine - Aleady reached position, Nothing to do. State-VALVE_AT_POSITION, Event-STOP_COMMAND "); Serial.println(event, HEX);
            break;

          case REACHED_POSITION:
            Serial.println("FAULT; Valve State Machine - Aleady reached position, Nothing to do. State-VALVE_AT_POSITION, Event-REACHED_POSITION ");
            break;

          default:
            Serial.print("FAULT; Valve State Machine 'Event' out of range: "); Serial.println(event, HEX);
        }
        break;

      case VALVE_CLOSING:
        switch (event)
        {
          case OPEN_COMMAND:
            noInterrupts();                                     // Freeze counters for the moment,
            openValve(200);                                     // Tell the valve to open
            valveState = VALVE_OPENING;                         // Set the state to VALVE_OPENING
            interrupts();          
            break;

          case CLOSE_COMMAND:
            if (digitalRead(dcMotorYellowPinClosed) == 0) break;    // Already closed, nothing to do
            noInterrupts();                                     // Freeze counters for the moment,
            closeValve(200);
            interrupts();          
            break;

          case STOP_COMMAND:
            noInterrupts();                                     // Freeze counters for the moment,
            stopValve();
            valveState = VALVE_AT_POSITION;
            interrupts();          
            break;

          case REACHED_POSITION:
            noInterrupts();                                     // Freeze counters for the moment,
            stopValve();
            valveState = VALVE_AT_POSITION;
            interrupts();
            break;

          default:
            Serial.print("FAULT; Valve State Machine Event out of range: "); Serial.println(event, HEX);
            break;
        }
        break;

      case VALVE_OPENING:
        switch (event)
        {
          case OPEN_COMMAND:
            noInterrupts();                                     // Freeze counters for the moment,
            openValve(100);
            interrupts();          
            break;

          case CLOSE_COMMAND:
            if (digitalRead(dcMotorYellowPinClosed) == 0) break;    // Already closed, nothing to do
            noInterrupts();                                     // Freeze counters for the moment,
            closeValve(2000);
            valveState = VALVE_CLOSING;
            interrupts();          
            break;

          case STOP_COMMAND:
            noInterrupts();                                     // Freeze counters for the moment,
            stopValve();
            valveState = VALVE_AT_POSITION;            
            interrupts();          
            break;

          case REACHED_POSITION:
            noInterrupts();                                     // Freeze counters for the moment,
            stopValve();
            valveState = VALVE_AT_POSITION;
            interrupts();
            break;

          default:
            Serial.print("FAULT; Valve State Machine Event out of range: "); Serial.println(event, HEX);
            break;                                               // 01/21/24 zzz
        }
        break;

      default:
        Serial.print("FAULT; Valve State Machine State out of range: "); Serial.println(valveState, HEX);
        if (digitalRead(dcMotorYellowPinClosed) == 0) break;    // Already closed, nothing to do
        noInterrupts();                                     // Freeze counters for the moment,
        closeValve(2000);
        interrupts();          
        break;

    }
  }
 }


void(* resetFunc) (void) = 0;           // Function to do a soft Reset of the Arduino  REZ 05/07/20



//Touch_getXY() updates global vars
bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    }
    return pressed;
}



// define the thermometer
// using hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(53);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000

#define RNOMINAL  100.0


void updateRTDtemperature(void)
{
    // Eventually make this a thermometer.  There will eventually be two kinds, a digital and an RTD
    // For now, only a RTD
    //
    // The RTD temperature is calibrated during the boiling water calibration and then should be accurate
    // to about 0.0001 degrees C.
    //
    // Updates the global "temperature" to the current probe value in degrees F.

    uint16_t rtd = thermo.readRTD();
    float ratio = rtd;
    ratio /= 32768;
    temperature = thermo.temperature(RNOMINAL, RREF);
    // Convert to Fahrenheit using (0°C × 9/5) + 32
    temperature = (temperature * 9.0 / 5.0) + 32.0 + boilingWaterCorrection;

    // Check and print any faults
    uint8_t fault = thermo.readFault();
    // zzzzzzzzzzz Need to add logging of faults zzzzzzzzzzzzzzzzzzzzzzzzzzzz
    if (fault) {
      Serial.print("Fault 0x"); Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
      }
      thermo.clearFault();
    }
}

void updateDisplayedTemperature(void)   // Update Actual Displayed Temperature if needed
{    
    if(((displayedTemperature - temperature) >= 0.095) || ((temperature - displayedTemperature) >= 0.095))
    {
      tft.setFont(&FreeSans24pt7b);
      tft.setTextSize(2);
      if((temperature - (boilingPoint + differential)) > alarmSetpoint)
        tft.setTextColor(RED, BLACK);
      else
        tft.setTextColor(BLUE, BLACK);
      tft.fillRect(0, 2, 240, 90, BLACK);
      tft.setCursor(05, 78);
      tft.print(temperature, 1);
      displayedTemperature = temperature;
    }
}

// 01/27/20214 - REZ Adding support for 5 Volt motor - Relay 2 is connected to the motor RED wire with NC on
//                   relay contact connected to ground and relay coil energized to Normally Open.  Relay 2 is
//                   controlled by pin 31 of the Arduino Board
//
//                   Relay 1 is connected to the motor BLACK wire, again with the unenergized coil the BLACK
//                   wire is held to ground.  Relay 1 is controlled by pin 33 of the Arduino board.
//
//                   In the motor OFF state, both relays are to be unenergized (held to ground) although you can
//                   also turn the motor OFF with both relays energized, although this is not recommended as 
//                   it is not a standard industrial practice (and also consumes energy needlessly.)
//
//                   To cause the motor to open the valve, Relay 1 is to be energized and Relay 2 is to be
//                   unenergized.
//
//                   To cause the motor to close the valve, Relay 2 is to be energized and Relay 1 is to be
//                   unenergized.
//
//                   When the valve is completely open, the signal on the GREEN wire from the motor will go to
//                   ground which can be read on Arduino pin 37.
//
//                   When the valve is completely closed, the signal on the Yellow wire from the motor will go to
//                   ground which can be read on Arduino pin 35.
//
//                   The selection of the 110 Volt motor or the 5 Volt motor is set in the configuration byte, 
//                   OPTIONS_EEPROM_ADDRESS in databit C (a.k.a. 0x1000).  When the databit is low, the motor
//                   is a 5 Volt motor (much, much safer) and when it is high, a 110 Volt AC motor is used.




void openValve(int openTime = 100)
{
  // First check to see whether 5 VDC or 110 VAC motor is being used
  //                   The selection of the 110 Volt motor or the 5 Volt motor is set in the configuration byte, 
  //                   OPTIONS_EEPROM_ADDRESS in databit C (a.k.a. 0x1000).  When the databit is low, the motor
  //                   is a 5 Volt motor (much, much safer) and when it is high, a 110 Volt AC motor is used.

  Serial.print("Opening Valve - Direction relay high, time to open: "); Serial.println(openTime);
  EEPROM.get(OPTIONS_EEPROM_ADDRESS, options);
  if ((options && 0x1000) == 0x1000)                         // Is this a 110 Volt motor?
  {                                                          //   Yes it is 110 Volt,
    digitalWrite(directionRelayPin,  HIGH);                  //   Relays are active high so this turns coil on and direction open,
    delay(RELAY_SETTLE_TIME);                                //   give time for relay to settle
    //Serial.println("Power relay closed");
    digitalWrite(powerRelayPin,      HIGH);                  //   Relays are active high so this turns coil on giving power
    closingValveTimer = -30000;                              //   Reset the closing timer if on,
    openingValveTimer = 0;                                   //   Wait the desired amount of opening
    openingValveTimerTarget = openTime;                      //   by resetting the timer and updating the global timer variable,
  }
  else                                                       //  Not 110 Volt, must be 5 Volt
  {
    digitalWrite(dcMotorRedPin,  HIGH);                      //   Relays are active high so this turns coil on and direction open,
    delay(RELAY_SETTLE_TIME);                                //   give time for relay to settle
    Serial.println("5 V red high; black low");
    digitalWrite(dcMotorBlackPin, LOW);                      //   Relays are active high so this turns coil on giving power
    closingValveTimer = -30000;                              //   Reset the closing timer if on,
    openingValveTimer = 0;                                   //   Wait the desired amount of opening
    openingValveTimerTarget = openTime;                      //   by resetting the timer and updating the global timer variable,
    
  }
}

// int dcMotorRedPin = 31;                 // I/O pin to control 5V motor relay
// int dcMotorBlackPin = 33;               // I/O pin to control 5V motor relay
// int dcMotorYellowPinClosed = 35;        // Input pin indicating 5V valve is fully closed when low
// int dcMotorGreenPinOpen = 37;           // Input pin indicating 5V valve is fully open when low

void closeValve(int closeTime = 12000)
{
  Serial.print("Closing Valve - Direction relay low, time to close: "); Serial.println(closeTime);
  //Serial.print("Opening Valve - Direction relay high, time to open: "); Serial.println(closeTime);
  EEPROM.get(OPTIONS_EEPROM_ADDRESS, options);
  if ((options && 0x1000) == 0x1000)                         // Is this a 110 Volt motor?
  {                                                          // Yes it is 110 Volt,
    digitalWrite(directionRelayPin, LOW);                    //   Relays are active high so this turns coil off and direction open,
    delay(RELAY_SETTLE_TIME);                                //   give time for relay to settle
    digitalWrite(powerRelayPin,      HIGH);                  //   Relays are active high so this turns coil on giving power
    openingValveTimer = -30000;                              //   Turn off the other valve timer so no conflicts,
    closingValveTimer = 0;                                   //   Wait the desired amount of opening
    closingValveTimerTarget = closeTime;                     //   by resetting the timer and updating the global timer variable,
  }
  else                                                       // Not 110 Volt, must be 5 Volt Motor
  {
    if (digitalRead(dcMotorYellowPinClosed) != 0)            //   Already closed? 
    {
      digitalWrite(dcMotorRedPin,   LOW);                    //     Not Clocsed, start closing, Relays are active high so this turns coil on and direction open,
      digitalWrite(dcMotorBlackPin, HIGH);                   //     Relays are active high so this turns coil on giving power
      openingValveTimer = -30000;                            //     Reset the closing timer if on,
      closingValveTimer = 0;                                 //     Wait the desired amount of opening
      closingValveTimerTarget = closeTime;                   //     by resetting the timer and updating the global timer variable,
    }
    else                                                     //   Already closed
    {
      stopValve();                                           //     Stop Valve motor.  
    } 
  }  
}



void stopValve(void)
{
  // First check to see whether 5 VDC or 110 VAC motor is being used.
  //
  //                   The selection of the 110 Volt motor or the 5 Volt motor is set in the configuration byte, 
  //                   OPTIONS_EEPROM_ADDRESS in databit C (a.k.a. 0x1000).  When the databit is low, the motor
  //                   is a 5 Volt motor (much, much safer) and when it is high, a 110 Volt AC motor is used.

  Serial.println("Stopping Valve - 110 or 5 Volt motor");
  EEPROM.get(OPTIONS_EEPROM_ADDRESS, options);
  
  if ((options && 0x1000) == 0x1000)                         // Is this a 110 Volt motor?
  {                                                          //  It is a 110 Volt motor.
    Serial.println("Stopping Valve - 110 Volt motor");
    digitalWrite(directionRelayPin, LOW);                    //   Relays are active high so this turns coil off and direction open,
    delay(RELAY_SETTLE_TIME);                                //   give time for relay to settle
    digitalWrite(powerRelayPin,     LOW);                    //   Relays are active high so this turns coil off stopping motor,
    openingValveTimerTarget = -30000;                        //   Turn off the other valve timers so no conflicts,
    closingValveTimerTarget = -30000;
    valveState = VALVE_UNKNOWN;                              //   Triggers a restart after manual mode
  }
  else
  {
    Serial.println("Stopping Valve - 5 Volt motor");
    digitalWrite(dcMotorRedPin,   LOW);                      //   Relays are active high so this turns coil off and direction open,
    digitalWrite(dcMotorBlackPin, LOW);                      //   Relays are active high so this turns coil off stopping motor,
    openingValveTimerTarget = -30000;                        //   Turn off the other valve timers so no conflicts,
    closingValveTimerTarget = -30000;
    valveState = VALVE_UNKNOWN;                              //   Triggers a restart after manual mode
  }
}


void calibrateRTD(void)
{
  {
    bool calibrating = true;
    float maxTemperature = 0;      // The maximum temperature the probe has reached
    int countOfMaxTemperature = 0; // Number of times in a row max temperature was reached.
    
    
    Serial.print("Reached Calibration Screen ");
    // clear screen and give instructions
    tft.fillScreen(BLACK);
    option_btn.initButton  (&tft,  40, 300, 80, 40, WHITE, CYAN, BLACK, "Cancel", 2);
    option_btn.drawButton(false);
    tft.setFont(&FreeSans12pt7b);
    tft.setTextColor(CYAN, BLACK);
    tft.setTextSize(1);
    tft.fillRect(5, 170, 240, 30, BLACK);
    tft.setCursor(10, 100);  //195
    tft.print("Place Probe in\nBOILING distilled\nwater and wait for\n'Calibration Complete'\nmessage.  Do NOT let probe tip touch the\nbottom or sides.");

    updateRTDtemperature();       // Get the current probe temperature
    maxTemperature = 0;
    boilingWaterCorrection = 0.0;
    
    while(calibrating)
    {
      // Handle the user abort of calibration
      down = Touch_getXY();

      option_btn.press(down && option_btn.contains(pixel_x, pixel_y));
      
      if (option_btn.justPressed()) 
      {
        tft.setTextSize(1);
        option_btn.drawButton(true);
      }
      
      if (option_btn.justReleased())
      {
        tft.setFont(NULL);
        tft.setTextSize(1);
        option_btn.drawButton(false);
        globalAutomaticManual = MANUAL;        // Switch to manual mode
        globalAutoManualFirstPass = true;
        calibrating = false;
      }
     
      if( calibrating == true )
      {
        updateRTDtemperature();   // update temperature variable
        Serial.print("temperature: ");
        Serial.println(temperature);
        //Serial.println(countOfMaxTemperature);
        if(temperature > maxTemperature)
        {
          maxTemperature = temperature;
          countOfMaxTemperature = 0;
        }
        if(temperature == maxTemperature)
        {
          countOfMaxTemperature++;
          Serial.print("temperature/maxTempCount: ");
          Serial.println(temperature);
          Serial.println(countOfMaxTemperature);
          
          if(countOfMaxTemperature > 1)
          {
            // Found the boiling point of water at this barometric pressure according to the RTD.
            // Now look up what the boiling point of water should be at this barometric pressure.
            // The difference between these two numbers is the calibration for the RTD which should be close 
            // to zero.
            Serial.print("Found RTD Boiling Point: ");
            Serial.println(maxTemperature);

            
            (countOfMaxTemperature);
            float deltaT = 0.0;
            if ((maxTemperature - boilingPoint) < 0.0)
            {
              deltaT = boilingPoint - maxTemperature;
            }
            else
            {
              deltaT = maxTemperature - boilingPoint;
            }
            EEPROM.put(RTD_CORRECTION_EEPROM_ADDRESS, deltaT);
            EEPROM.get(RTD_CORRECTION_EEPROM_ADDRESS, deltaT);
            Serial.print("RTD correction is: ");
            Serial.println(deltaT);
            boilingWaterCorrection = deltaT;

            // zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
            tft.fillScreen(BLACK);
            tft.setCursor(10, 195);
            tft.print(" Calibration Complete. Power Cycle to\r\nResume");
            calibrating = false;
            globalAutomaticManual = AUTOMATIC;
            resetFunc();                    // 01/21/24 Do a cold boot to start up
          }
        }
      }
    }
  }
}


void setup(void)
{
    long barometerPascals = 0;

    // Setup pins for 110 Volt motor
    pinMode(powerRelayPin, OUTPUT);
    pinMode(directionRelayPin,  OUTPUT);
    digitalWrite(powerRelayPin, LOW);                        // Relays are active HIGH so this turns coil off
    digitalWrite(directionRelayPin,  LOW);                   // Relays are active HIGH so this turns coil off and
                                                             //   Sets direction to close
    // Setup pins for 5 Volt DC motor
    pinMode(dcMotorRedPin, OUTPUT);                          // With DC motors, we reverse the polarity to change the
    pinMode(dcMotorBlackPin,  OUTPUT);                       // direction.
    digitalWrite(dcMotorRedPin, LOW);                        // Relays are active HIGH so this turns coil off
    digitalWrite(dcMotorBlackPin,  LOW);                     // Relays are active HIGH so this turns coil off and
    pinMode(dcMotorYellowPinClosed, INPUT);                  // Configure fully closed valve input
    pinMode(dcMotorGreenPinOpen, INPUT);                     // Configure fully open valve input
    
                                                            
    String rx_str = "";                                      // Input string in case first time running

    // zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
    // Debug to make calibrating of Barometer mandatory

    // EEPROM.put(OPTIONS_EEPROM_ADDRESS, 0xFFFF);
    
    // zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz


    // Start off by closing the valve if not already closed and setting the valve state
    closeValve();
    valveState = VALVE_AT_POSITION;
    
    Serial.begin(9600);
    Wire.begin();                                             // Join the I2C bus as master;
    uint16_t ID = tft.readID();
    Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    Serial.println("Calibrate for your Touch Panel");
    //if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.begin(ID);
    tft.setRotation(0);            //PORTRAIT
    tft.fillScreen(BLACK);

    //  zzzzzzz    Need to add test for 5or 110 V motor
    Serial.println("Closing valve for 20 seconds.");
    digitalWrite(dcMotorRedPin, LOW);
    digitalWrite(dcMotorBlackPin,  HIGH);


    
    while(0)
    {
    Serial.println("Closing valve for one second.");
    digitalWrite(dcMotorRedPin, LOW);
    digitalWrite(dcMotorBlackPin,  HIGH);
    delay(1000);
    Serial.println("Opening valve for two seconds.");
    digitalWrite(dcMotorRedPin, HIGH);
    digitalWrite(dcMotorBlackPin, LOW);
    delay(2000);
    Serial.print("Green Open Valve Pin:  ");
    Serial.println(digitalRead(dcMotorGreenPinOpen));
    }
 

    up_btn.initButton      (&tft, 140, 300, 60, 40, WHITE, CYAN, BLACK, "UP", 2);
    down_btn.initButton    (&tft, 205, 300, 60, 40, WHITE, CYAN, BLACK, "DOWN", 2);
    option_btn.initButton  (&tft,  40, 300, 80, 40, WHITE, CYAN, BLACK, "Manual", 2);
    open_btn.initButton    (&tft,  30, 240, 60, 40, WHITE, CYAN, BLACK, "Open", 2);
    close_btn.initButton   (&tft, 105, 240, 65, 40, WHITE, CYAN, BLACK, "Close", 2);
    stop_btn.initButton    (&tft, 180, 240, 60, 40, WHITE, CYAN, BLACK, "Stop", 2);
    up_btn.drawButton(false);
    down_btn.drawButton(false);
    option_btn.drawButton(false);
    open_btn.drawButton(true);
    close_btn.drawButton(true);
    stop_btn.drawButton(true);


    Serial.println("Starting thermometer");
    
    // Set thermometer up for reading
    thermo.begin(MAX31865_3WIRE);  // set to 3WIRE

    Serial.println("Started thermometer?");

    // Debug
    updateRTDtemperature();
    Serial.print("RTD temp without correction:  ");
    Serial.println(temperature);
    
    Serial.println("Starting barometer");


    // Set barometer up for reading
    pinMode(BME_CS, OUTPUT);        // Set BME280 for SPI interface
    digitalWrite(BME_CS, LOW);   

    int barometerStatus = bme.begin();
    while (!barometerStatus)  
    {
      barometerStatus = bme.begin();
      Serial.println("Could not find a valid BME280 sensor, check wiring! Result: ");
      Serial.print("Barometer Status: ");
      Serial.println(barometerStatus, HEX);  //zzzzzzz Put on touchscreen
      delay(1000);
    }
    
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BME280::SAMPLING_X16,    /* Temp. oversampling */
                    Adafruit_BME280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BME280::SAMPLING_X16,    /* Humidity oversampling */                   
                    Adafruit_BME280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BME280::STANDBY_MS_0_5); /* Standby time. */
                    
    Serial.println("Started barometer");

    barometerPascals = bme.readPressure();
    Serial.print("checking barameter pressure Pascals:  ");
    Serial.println(bme.readPressure());

 //   Bit  Meaning
 //    F   Low - Barometer has been calibrated and stored in EEPROM address BAROMETER_CORRECTION_EEPROM_ADDRESS as float
 //    E   Low - RTD has been calibrated and stored in EEPROM address RTD_CORRECTION_EEPROM_ADDRESS as float
 //    D   Low - Target temperature spread has been set by user and stored in EEPROM address TARGET_TEMPERATURE_EEPROM_ADDRESS as float
 //    C   Low - 5 Volt DC motor.  High - 110 Volt AC motor.
    
 // zzzzzzzzzzz  01/28/24 For debug purposes set Barom calibration done, RTD calibrated, Target Set, 5 Volt motor
    EEPROM.put(OPTIONS_EEPROM_ADDRESS, 0x0FFF);


    // Dump EEPROM contents
    EEPROM.get(OPTIONS_EEPROM_ADDRESS, options);
    Serial.print("Options Byte:  ");
    Serial.println(options);
    if((options & 0x2000) == 0)
    {
      EEPROM.get(TARGET_TEMPERATURE_EEPROM_ADDRESS, differential);
      Serial.print("Target temperature:  ");
      Serial.println(differential);
    }
    else
    {
      //differential = 7.1;
      differential = -143.0;                // debug 01/21/24 zzzzzz
      EEPROM.put(TARGET_TEMPERATURE_EEPROM_ADDRESS, differential);
      Serial.print("Systen Default Target temperature:  ");
      Serial.println(differential);
    }
    if ((options & 0x8000) == 0)
    {
      EEPROM.get(BAROMETER_CORRECTION_EEPROM_ADDRESS , barometerCorrection);
      Serial.print("Barometer Correction in Pascals:  ");
      Serial.println(barometerCorrection);
    }
    else
    {
      barometerPascals = bme.readPressure();

      // Now prompt for barometric correction using the monitor window.
      Serial.println("Enter current barometric pressure in Pascals:  ");
      
      char rx_byte = 0;             // Place to receive input character from serial monitor
      long int barometerCorrection = 0;  // Running total of input barometric pressure

      while(1)
      {
        if (Serial.available() != 0)                // Wait for a character.
        {
          rx_byte = Serial.read();                    // Get the character,
          if (rx_byte == '\n') break;                 // If character is line feed, got the string.
          if (rx_byte >= '0' && rx_byte <= '9')
          {
            barometerCorrection = (barometerCorrection * 10) + (rx_byte - '0');
          }
        else
          {
            Serial.println("Not a decimal number.  Ignored.");
          }
        }
      }
      Serial.print("Current barometric pressure correction in Pascals:  ");
      Serial.println(barometerCorrection);
      Serial.print("Current barametric pressure in Pascals from barometer:  ");
      Serial.println(barometerPascals);

      if(barometerCorrection > barometerPascals) 
        barometerCorrection = barometerCorrection - barometerPascals;
      else
        barometerCorrection = barometerPascals - barometerCorrection;
      Serial.print("Correction value to add to instrument barometric pressure:  ");
      Serial.println(barometerCorrection);
      EEPROM.put(BAROMETER_CORRECTION_EEPROM_ADDRESS, barometerCorrection);
      EEPROM.get(BAROMETER_CORRECTION_EEPROM_ADDRESS, barometerCorrection);  
      Serial.print("Barometer Correction value stored in EEPROM:  ");
      Serial.println(barometerCorrection);
      
      // Update the barometric option bit in "options" and update the EEPROM
      options = options & 0x7FFF;
      EEPROM.put(OPTIONS_EEPROM_ADDRESS, options);  
    }

    // Now set the boiling water correction for the RTD.  If the correction
    // value is already set in EEPROM as noted by the "options" value in 
    // EEPROM, then use that.  Otherwise use no correction.
    
    if ((options & 0x4000) == 0)
    {
      EEPROM.get(RTD_CORRECTION_EEPROM_ADDRESS , boilingWaterCorrection);
      Serial.print("RTD Correction in degrees F:  ");
      Serial.println(boilingWaterCorrection);
    }
    else
    {
      boilingWaterCorrection = 0.0;
      EEPROM.put(RTD_CORRECTION_EEPROM_ADDRESS , boilingWaterCorrection);
      Serial.print("Not Calibrated - Using default RTD Correction in degrees F:  ");
      Serial.println(boilingWaterCorrection);
    }

    // Is this the first time the EEPROM been read?
    EEPROM.get(OPTIONS_EEPROM_ADDRESS, options);
   
    // Get the value of the differential between boiling point of water and syrup in degrees F setpoint from EEPROM
    EEPROM.get(TARGET_TEMPERATURE_EEPROM_ADDRESS, differential);            // Target temperature value
    Serial.print("Read/updated eeprom B differential ");
    Serial.println(differential);
    EEPROM.get(BAROMETER_CORRECTION_EEPROM_ADDRESS, barometerCorrection);   // barometric pressure calibration value.
    Serial.print("Barometric Correction in Pascals:  ");
    Serial.println(barometerCorrection);
    EEPROM.get(RTD_CORRECTION_EEPROM_ADDRESS, boilingWaterCorrection);      // RTD temperature probe calibration value.
    Serial.print("RTD Correction in degrees F:  ");
    Serial.println(boilingWaterCorrection);

    // Read the barometer in pascals            zzzzzzzzzzzzzzz
    barometerPascals = bme.readPressure() + barometerCorrection;
    barometerPascals -= 82500;

    Serial.print("Barometer Pascals -82500 is ");
    Serial.println(barometerPascals);
    
    //  Now get the index into the table by dividing by 500 Pascals,
    //  the step size of the table.
            
    tableIndex = barometerPascals / 500;
    interpolate = (barometerPascals % 500) / 100;
    boilingPoint = boilingTemp[tableIndex];
    boilingPoint = boilingPoint + (((float)interpolate / 5.0) * (boilingTemp[tableIndex + 1] - boilingTemp[tableIndex]));
    //targetValue = boilingPoint + differential;
    Serial.print("boilingPoint A ");
    Serial.println(boilingPoint);
    
    Serial.print("(boilingPoint + differential) A ");
    Serial.println((boilingPoint + differential));
    displayedTargetValue = (boilingPoint + differential);

    globalAutomaticManual = AUTOMATIC;                        // Mode in which the instrument is running
    globalAutoManualFirstPass = true;                         // Force initialization of mode in loop
                                                              // Maybe should be a cold boot? (resetFunc)) 01/21/24 zzzzzz
    Serial.println("\n\nEEPROM values after initialization\n");
    Serial.print("OPTIONS_EEPROM:  ");
    int temp = 0;
    EEPROM.get(OPTIONS_EEPROM_ADDRESS, temp);
    Serial.println(temp);
    Serial.print("Target Temperature above water boiling point:  ");
    float ftemp = 0;
    EEPROM.get(TARGET_TEMPERATURE_EEPROM_ADDRESS, ftemp);
    Serial.println(ftemp);
    ftemp = 0;
    EEPROM.get(BAROMETER_CORRECTION_EEPROM_ADDRESS, ftemp);
    Serial.print("Barometer Correction in Pascals:  ");
    Serial.println(ftemp);
    ftemp = 0;
    EEPROM.get(RTD_CORRECTION_EEPROM_ADDRESS, ftemp);
    Serial.print("RTD Correction in degrees F:  ");
    Serial.println(ftemp);
    Serial.println();
   

    // Set up real time interrupt using timer 1
    /*
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 34286;            // preload timer 65536-16MHz/256/2Hz
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
    Serial.println("Completed Set-up");
    */

    // closeValve(12000);

}

long  timerCount = 0;

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  TCNT1 = 34286;            // preload timer
  timerCount++;
  //Serial.print(".");
  //Serial.println(timerCount);
}

float lastTemperature = 0.0;
long  barometerPascals;

// =======================================================================================


void loop(void)
{
  down = Touch_getXY();
  if(globalAutomaticManual == AUTOMATIC)
  {
    if(globalAutoManualFirstPass)
    {
      Serial.println("Starting AUTOMATIC first Pass");
      tft.fillScreen(BLACK);
      automaticManualTimer = 0;
      EEPROM.get(TARGET_TEMPERATURE_EEPROM_ADDRESS, differential);
      //targetValue = boilingPoint + differential;
      displayedTemperature = 0.0; 
      displayedTargetValue = 0.0;
      displayedBoilingPoint = 0.0;
      open_btn.initButton    (&tft,  30, 240, 60, 40, WHITE, CYAN, BLACK, "Open", 2);
      tft.setFont(NULL);
      tft.setTextSize(1);
      open_btn.drawButton(true);
      close_btn.initButton   (&tft, 105, 240, 65, 40, WHITE, CYAN, BLACK, "Close", 2);
      close_btn.drawButton(true);
      stop_btn.initButton    (&tft, 180, 240, 60, 40, WHITE, CYAN, BLACK, "Stop", 2);
      stop_btn.drawButton(true);
      option_btn.initButton  (&tft,  40, 300, 80, 40, WHITE, CYAN, BLACK, "Auto", 2);
      option_btn.drawButton(false);
      up_btn.initButton      (&tft, 140, 300, 60, 40, WHITE, CYAN, BLACK, "UP", 2);
      up_btn.drawButton(false);
      down_btn.initButton    (&tft, 205, 300, 60, 40, WHITE, CYAN, BLACK, "DOWN", 2);
      down_btn.drawButton(false);
      globalAutoManualFirstPass = false;
    }
    up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
    down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
    option_btn.press(down && option_btn.contains(pixel_x, pixel_y));
    
    if (up_btn.justReleased())
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        up_btn.drawButton(false);
        differential = differential + 0.1;
        EEPROM.put(TARGET_TEMPERATURE_EEPROM_ADDRESS, differential);
        updateTargetTemp = true;
    }
    if (down_btn.justReleased())
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        down_btn.drawButton(false);
        differential = differential - 0.1;
        EEPROM.put(TARGET_TEMPERATURE_EEPROM_ADDRESS, differential);
        updateTargetTemp = true;
    }
    if (option_btn.justReleased())
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        option_btn.drawButton(false);
        globalAutomaticManual = MANUAL;        // Switch to manual mode
        globalAutoManualFirstPass = true;
    }
    
    if (up_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        up_btn.drawButton(true);
    }
    
    if (down_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        down_btn.drawButton(true);
    }
    
    if (option_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        option_btn.drawButton(true);
    }
   
    // Get the temperature from the thermometer
    // Eventually make this a thermometer.  There will eventually be two kinds, a digital and an RTD
    // For now, only a RTD

    updateRTDtemperature();
    
    // Update Display

    updateDisplayedTemperature();
    Serial.print("RTD temp:  ");
    Serial.print(temperature);
    Serial.print("  displayed temperature:  ");
    Serial.print(displayedTemperature);
    Serial.print("  EEPROM RTD correction:  ");
    Serial.println(boilingWaterCorrection);
    
    // Update Target if needed
    if(updateTargetTemp)
    {
      updateTargetTemp = false;
      Serial.print("Updating displayedTargetValue = ");
      Serial.println(displayedTargetValue);
      Serial.print("Updating (boilingPoint + differential) = ");
      Serial.println((boilingPoint + differential));
      
      tft.setFont(&FreeSans24pt7b);
      tft.setTextSize(2);
      tft.setTextColor(GREEN, BLACK);
      tft.fillRect(5, 80, 240, 88, BLACK);
      tft.setCursor(05, 160);
      tft.print((boilingPoint + differential), 1);
      displayedTargetValue = (boilingPoint + differential);
    }
    // Read the barometer in pascals            zzzzzzzzzzzzzzz
    barometerPascals = bme.readPressure() + barometerCorrection;
    //Serial.print("barometerPascals: ");
    //Serial.println(barometerPascals);
    barometerPascals -= 82500;
            
    //  Now get the index into the table by dividing by 500 Pascals,
    //  the step size of the table.
            
    tableIndex = barometerPascals / 500;
    interpolate = (barometerPascals % 500) / 100;
    boilingPoint = boilingTemp[tableIndex];
    boilingPoint = boilingPoint + (((float)interpolate / 5.0) * (boilingTemp[tableIndex + 1] - boilingTemp[tableIndex]));
    //Serial.print(" boilingPoint: ");
    //Serial.print(boilingPoint);
           
    // Display Boiling Point of Water as degrees F
    if(displayedBoilingPoint != boilingPoint)              //   REZ 02/13/22 Removed as didn't work and just changed to not equal >= 0.1) || ((displayedBoilingPoint - boilingPoint) <= -0.1))
    {
      updateTargetTemp = true;                              // Also need to update the target temperature so set a flag so it will be updated.
      tft.setFont(&FreeSans12pt7b);
      tft.setTextSize(1);
      tft.setTextColor(CYAN, BLACK);
      tft.fillRect(5, 170, 240, 30, BLACK);
      tft.setCursor(10, 195);
      tft.print(" Water B.P.  ");
      tft.print(boilingPoint, 1);
      Serial.print("displayedBoilingPoint:  ");
      Serial.print(displayedBoilingPoint);
      Serial.print("  boilingPoint:  ");
      Serial.println(boilingPoint);
      /*if(displayedBoilingPoint > boilingPoint)
      {
        targetValue = targetValue - (displayedBoilingPoint - boilingPoint);
      }
      else
      {
        targetValue = targetValue + (displayedBoilingPoint - boilingPoint);
      }*/
      displayedBoilingPoint = boilingPoint;
    }
    /* Determine what command to send to FSM.
     *  
     * if temperature (of sap/syrup) < target temeperature (boilingPoint + differntial)
     *    RunFSM(CLOSE_COMMAND)
     *  
     * elseif temperature >= target temperature && temperature <= target temperature + .005
     *    RunFSM(REACHED_POSITION)
     *    
     * elseif temperature >= target temperature && temperature > target temperature + .005
     *    RunFSM(OPEN_COMMAND)
     *    
     */
     
     if (temperature < (boilingPoint + differential))
        valveFSM(CLOSE_COMMAND);
      
     if ((temperature >= boilingPoint + differential) && (temperature <= (boilingPoint + differential + .005)))
        valveFSM(REACHED_POSITION);
       
     if ((temperature >= boilingPoint + differential) && temperature > (boilingPoint + differential + .005))
        valveFSM(OPEN_COMMAND);

/*
    // Control the Valve
    // 1.  If the valve is closed and the temperature is below the setpoint, do nothing.
    // 2.  If the valve is closed and the temperature is at or above the setpoint, crack open the valve.
    // 3.  If the valve is open and the temperature has gone up, open the valve some more.
    // 4.  If the valve is open and the temperature is < .05 degrees above the setpoint, close the valve some.
    // 5.  If the valve is open and the temperature is below the setpoint, close the valve.
    
    // 1.  If the valve is closed and the temperature is below the setpoint, do nothing.
    if((valveState == VALVE_CLOSED) && (temperature < (boilingPoint + differential))) {}
        
    // 2.  If the valve is closed and the temperature is at or above the setpoint, crack open the valve.
    if(((valveState == VALVE_CLOSED) || (valveState == VALVE_STOPPED))  && (temperature >= (boilingPoint + differential)))
    {
      openValve(3000);
      valveState = VALVE_OPEN;
      lastTemperature = temperature;
    }

    // 3.  If the valve is open and the temperature has gone up, open the valve some more.
    if(((valveState == VALVE_OPEN) || (valveState == VALVE_STOPPED)) && ((temperature > lastTemperature) || temperature > ((boilingPoint + differential) + .05)))
    {
      openValve(1000);
      valveState = VALVE_OPEN;
      lastTemperature = temperature;
    }

    // 4.  If the valve is open and the temperature is < .05 degrees above the setpoint, close the valve some.
    if(((valveState == VALVE_OPEN) || (valveState == VALVE_STOPPED)) && ((temperature < ((boilingPoint + differential) + .05)) && (temperature >= (boilingPoint + differential))))
    {
      closeValve(1000);
    }
    
    // 5.  If the valve is open and the temperature is below the setpoint, close the valve.
    if(((valveState == VALVE_OPEN) || (valveState == VALVE_STOPPED)) && (temperature < (boilingPoint + differential)))
    {
      closeValve();
      valveState = VALVE_CLOSED;
    }

    // 6. If the valve is open and temperature is more than 1 degree above the setpoint, fully open the valve.
    if(temperature > (boilingPoint + differential) + 1.0)
    {
      openValve();
      valveState == VALVE_OPENING;
    }

    // Check to see if it is time to stop the valve motor by checking the elapsedMillis variables against their
    // respective global variable endpoints.

    if(openingValveTimer >= globalOpenTime)
    {
      if((valveState = VALVE_OPENING) || (valveState == VALVE_AT_POSITION));
      {
        digitalWrite(powerRelayPin,     LOW);                 // Turn power off to hold valve at this position.
        digitalWrite(directionRelayPin, LOW);
      }
      openingValveTimer = 0;                                   // Turn off the timer
    }
    
    if(closingValveTimer >= globalCloseTime)
    {
      if((valveState = VALVE_CLOSED) || (valveState == VALVE_STOPPED));
      {
        digitalWrite(powerRelayPin,     LOW);                  // Turn power off to hold valve at this position.
        digitalWrite(directionRelayPin, LOW);
      }
      closingValveTimer = 0;                                   // Turn off the timer
    }  */
  }
  // End of Automatic Processing.  

  else if(globalAutomaticManual == MANUAL)  // *********Start of Manual Processing ************************
  {
    if(globalAutoManualFirstPass)
    {
      tft.fillScreen(BLACK);
      option_btn.initButton  (&tft,  40, 300, 80, 40, WHITE, CYAN, BLACK, "Manual", 2);
      down_btn.initButton    (&tft, 205, 300, 60, 40, WHITE, CYAN, BLACK, "Calib", 2);
      tft.setFont(NULL);
      tft.setTextSize(1);
      up_btn.drawButton(true);
      down_btn.drawButton(false);
      option_btn.drawButton(false);
      open_btn.drawButton(false);
      close_btn.drawButton(false);
      stop_btn.drawButton(false);
      globalAutoManualFirstPass = false;
      displayedTemperature = 0.0;
      displayedBoilingPoint = 0.0;
    }
    option_btn.press(down && option_btn.contains(pixel_x, pixel_y));
    open_btn.press(down && open_btn.contains(pixel_x, pixel_y));
    close_btn.press(down && close_btn.contains(pixel_x, pixel_y));
    stop_btn.press(down && stop_btn.contains(pixel_x, pixel_y));
    down_btn.press(down && down_btn.contains(pixel_x, pixel_y));

    if (option_btn.justReleased())
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        option_btn.drawButton(false);
        globalAutomaticManual = AUTOMATIC;        // Switch to automatic mode
        globalAutoManualFirstPass = true;
        Serial.println("in Manual mode going to Auto mode, auto button released.  Doing Cold Boot");
        delay(500);                               // Give time to write log,
        resetFunc();                              // Do a cold boot zzz 01/21/24
    }
    
    if (open_btn.justReleased())
    {
        Serial.println("in Manual mode Open button released.");
        openValve(12000);
        tft.setFont(NULL);
        tft.setTextSize(1);
        open_btn.drawButton(false);
    }

    if (close_btn.justReleased())
    {
        Serial.println("in Manual mode Close button released.");
        closeValve(12000);
        tft.setFont(NULL);
        tft.setTextSize(1);
        close_btn.drawButton(false);
    }
    
    if (stop_btn.justReleased())
    {
        Serial.println("in Manual mode Stop button released.");
        stopValve();
        tft.setFont(NULL);
        tft.setTextSize(1);
        stop_btn.drawButton(false);
    }

    // down_btn in manual mode is the calibration button
    if (down_btn.justReleased())
    {
        calibrateRTD();
        tft.setFont(NULL);
        tft.setTextSize(1);
        down_btn.drawButton(false);
    }
    
    if (option_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        option_btn.drawButton(true);
    }
    
    if (open_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        open_btn.drawButton(true);
    }

    if (close_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        close_btn.drawButton(true);
    }

    if (stop_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        stop_btn.drawButton(true);
    }

    // down_btn in manual mode invokes calibration
    if (down_btn.justPressed()) 
    {
        tft.setFont(NULL);
        tft.setTextSize(1);
        down_btn.drawButton(true);
    }
    

    updateRTDtemperature();

    updateDisplayedTemperature();

    // Read the barometer in pascals            zzzzzzzzzzzzzzz
    barometerPascals = bme.readPressure() + barometerCorrection;
    barometerPascals -= 82500;
            
    //  Now get the index into the table by dividing by 500 Pascals,
    //  the step size of the table.
            
    tableIndex = barometerPascals / 500;
    interpolate = (barometerPascals % 500) / 100;
    boilingPoint = boilingTemp[tableIndex];
    boilingPoint = boilingPoint + (((float)interpolate / 5.0) * (boilingTemp[tableIndex + 1] - boilingTemp[tableIndex]));
            
    // Display Boiling Point of Water as degrees F
    if(displayedBoilingPoint != boilingPoint) // >= 0.1) || ((displayedBoilingPoint - boilingPoint) <= -0.1))
    {
      tft.setFont(&FreeSans12pt7b);
      tft.setTextColor(CYAN, BLACK);
      tft.setTextSize(1);
      tft.fillRect(5, 170, 240, 30, BLACK);
      tft.setCursor(10, 195);
      tft.print(" Water B.P.  ");
      tft.print(boilingPoint, 1);
      Serial.print("displayedBoilingPoint:  ");
      Serial.print(displayedBoilingPoint);
      Serial.print("  boilingPoint:  ");
      Serial.println(boilingPoint);
      /*if(displayedBoilingPoint > boilingPoint)
      {
        targetValue = targetValue - (displayedBoilingPoint - boilingPoint);
      }
      else
      {
        targetValue = targetValue + (displayedBoilingPoint - boilingPoint);
      }*/
      displayedBoilingPoint = boilingPoint;
    }
  } //
  //else // Calibration Screen/Action
}
/*
boilingWaterCorrection  float of what to add to the RTD probe to correct for RTD offset.

EEPROM.get(BAROMETER_CORRECTION_EEPROM_ADDRESS, barometerCorrection);
      if(barometerCorrection == 0xFFFFFFFF)

long barometerCorrection = 0;             // The number of Pascals to add/subtract to barometer reading to match actual barometric pressure reading

float displayedBoilingPoint;              // What is currently display on screen. rounded (truncate?)

float boilingPoint;                       // computed boiling point of water from the table, not truncated.

float differential = 0.0;                 // The amount above the boiling point of water to start opening the valve

float boilingWaterCorrection = 0.0;       // The number of degrees Fahrenheit to adjust the RTD value

float targetValue = 0.0;                  // The target (temperature, brix)  identically equal to (boilingPoint + differential)

float displayedTargetValue = 0.0;         // The last displayed Target Value to see if we need to update screen

float currentValue = 0.0;                 // The current value of the sap/syrup (temperature, brix)

float temperature = 0.0;                  // The current temperature returned by the thermometer corrected for the RTD constant,                                                                  boilingWaterCorrection   Used to see if we need to update screen or can we avoid the flicker

float alarmSetpoint = 0.2;                // Temperature Alarm Setpoint above the target value

float displayedTemperature = 0.0;         // The temperature currently displayed.  Used to see if we need to update screen or can we avoid the                                                    flicker zzzzzzzzz Change to 0 for production

int   valveState;                         // Knows the state of the valve as VALVE_OPEN and VALVE_CLOSED

short int options;                        // Options enabled for this instrument 0x80 == First time since reprogramming

globalOpenTime = openTime;                // by resetting the timer and updating the global timer variable,

globalCloseTime = closeTime;                             // by resetting the timer and updating the global timer variable,




elapsedMillis openingValveTimer;          // How many milliseconds left in opening the valve
elapsedMillis closingValveTimer;          // How many milliseconds left in closing the valve
elapsedMillis upButtonTimer;              // The one degree increment timer for fast increment
elapsedMillis downButtonTimer;            // The one degree decrement timer for fast decrement
elapsedMillis optionButtonTimer;          // Timer to switch to calibration screen
elapsedMillis calibrationTimer;           // Timer to tell when calibration temperature has stabilized
elapsedMillis automaticManualTimer;       // Timer to allow the comm  buffers to be emptied before switching modes
*/
#endif
