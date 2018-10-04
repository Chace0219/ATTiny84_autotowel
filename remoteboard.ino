
/*
Alpha Phase - Remaining Changes:

"Lift to turn off" function + 20 second timer
Setting LED to stay on full brightness during grinding (auto or manual grinding) and 5 seconds afterward (fast fade on + slow fade off)
Setting LED to stay on full brightness for 10 seconds after changing settings (fast fade on + slow fade off)
Debounce settings button
Settings light turns off during manual grinding, takes a few seconds to come back on (auto LED flashes once)
Sometimes the device will only briefly grind when the manual grind button is held down
*/
#include <Wire.h>
#include "FBD.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "FiniteStateMachine.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_VL6180X.h"
#include "SoftPWM.h"

#define LIFTTURNOFF 20000
#define DEBOUNCETIME 100 // ms unit

#define NONEACTIONINTERVAL 5000 // ms unit
#define ACTIVEDELAY 5000 // ms unit
#define SETTINGDELAY 10000 // ms unit

bool motorStatus = false;
bool motorWorked = false;

//#define DEBUGSERIAL
#ifdef DEBUGSERIAL
#include <SoftwareSerial.h>
SoftwareSerial mySerial(255, 5); // RX, TX
#endif

#define RANGETHRESHOLD 50
SOFTPWM_DEFINE_CHANNEL(2, DDRA, PORTA, PORTA7);
SOFTPWM_DEFINE_CHANNEL(0, DDRB, PORTB, PORTB0);
SOFTPWM_DEFINE_CHANNEL(1, DDRB, PORTB, PORTB1);
#ifndef DEBUGSERIAL
SOFTPWM_DEFINE_CHANNEL(3, DDRA, PORTA, PORTA5);
#endif
SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(5, 100);
SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(5, 100);

Adafruit_VL6180X vl = Adafruit_VL6180X();
#define CONFIG_VERSION "ls1"
#define CONFIG_START 32
struct StoreStruct {
    char version[4];
    // The variables of your settings
    uint32_t motorActTime; // ms unit
} setting = {
    CONFIG_VERSION,
    // The default values
    2000
};

void loadConfig() {
    if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
        EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
        EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
        for (unsigned int t = 0; t<sizeof(setting); t++)
            *((char*)&setting + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
    for (unsigned int t = 0; t<sizeof(setting); t++)
        EEPROM.write(CONFIG_START + t, *((char*)&setting + t));
}

#define VBATPIN   0
#define BUTTONS   1
#define LEDSPIN   2
#define TSL_VDD   3

#define MOTOR   5  
#define REDPIN   7  
#define GREENPIN   10 
#define BLUEPIN   9 

#define OPTICALPIN 11
#define IRLED   8 


TON grindButton(DEBOUNCETIME);
TON settingButton(DEBOUNCETIME);
Rtrg grindButtonTrg, settingButtonTrg;

void activeEnter();

State idle(NULL);
State active(activeEnter, NULL, NULL);
State activeDelay(NULL);
State deactiveDelay(NULL);
State settingDelay(NULL);
FiniteStateMachine stateMachine(idle);

State fadingUp(NULL);
State fadingDown(NULL);
State fadingMax(NULL);
State ledIdle(NULL);
FiniteStateMachine ledController(fadingUp);

#define MANUAL false
#define AUTO true
bool actMode = AUTO;

#define HI 1
#define LO 0
#define HZ 2

static uint8_t rangeThres = RANGETHRESHOLD;
//static uint8_t prevDistance = 0;

static uint8_t readingBuff[6] = { 255 };
const uint8_t BUFFCOUNT = 6;

void driveLed(uint8_t upper, uint8_t red, uint8_t green, uint8_t blue)
{
    pinMode(LEDSPIN, OUTPUT);
    if (upper)
        digitalWrite(LEDSPIN, LOW);
    else
        digitalWrite(LEDSPIN, HIGH);

    if (upper)
    {
        Palatis::SoftPWM.set(0, map(red, 0, 255, 0, 100));
        Palatis::SoftPWM.set(1, map(green, 0, 255, 0, 100));
        Palatis::SoftPWM.set(2, map(blue, 0, 255, 0, 100));
    }
    else
    {
        Palatis::SoftPWM.set(0, map(red, 0, 255, 100, 0));
        Palatis::SoftPWM.set(1, map(green, 0, 255, 100, 0));
        Palatis::SoftPWM.set(2, map(blue, 0, 255, 100, 0));
    }//*/
}

// 
void powerSensor(bool turnOn)
{
    if (turnOn)
        digitalWrite(TSL_VDD, HIGH);
    else
        digitalWrite(TSL_VDD, LOW);
}

void setup()
{
#ifdef DEBUGSERIAL
    mySerial.begin(9600);
    // mySerial.println(F("release logic project"));
#endif
    pinMode(VBATPIN, INPUT);
    pinMode(BUTTONS, INPUT);

    pinMode(LEDSPIN, OUTPUT);
    pinMode(REDPIN, OUTPUT);
    pinMode(GREENPIN, OUTPUT);
    pinMode(BLUEPIN, OUTPUT);
    pinMode(TSL_VDD, OUTPUT);
    digitalWrite(TSL_VDD, LOW);

    powerSensor(true);
    Palatis::SoftPWM.begin(60);
    Palatis::SoftPWM.printInterruptLoad();

    if (!vl.begin())
    {
#ifdef DEBUGSERIAL
        mySerial.println(F("sensor failed"));
#endif
    }
    else
    {
#ifdef DEBUGSERIAL
        mySerial.println(F("snsr ok"));
#endif
    }

    uint8_t range = 100;
    uint8_t sensorStatus;
    range = vl.readRange();
    sensorStatus = vl.readRangeStatus();
    //    if (sensorStatus == VL6180X_ERROR_NONE)
    //        prevDistance = range;
    //    else
    //        prevDistance = 0;

#ifndef DEBUGSERIAL
    pinMode(MOTOR, OUTPUT);
#endif
    driveMotor(false);
}

static uint16_t avrCalibration = 100; // start at max - will settle faster
static uint16_t avrValue = 100; // start at max - will settle faster
                                //static uint16_t avrDelta = 0;

void loop()
{
    static uint32_t lastCheckVLTime = millis();

    // read adc valuye from button pins
    static uint16_t buttonADC = 0;
    buttonADC = analogRead(BUTTONS);

    // check SW1 action 
    grindButton.IN = buttonADC > 400;
    grindButton.update();
    grindButtonTrg.IN = grindButton.Q;
    grindButtonTrg.update();

    // check SW2 switch
    settingButton.IN = buttonADC < 300; // that part
    settingButton.update();
    settingButtonTrg.IN = settingButton.Q;
    settingButtonTrg.update();

    uint8_t pwmValue;
    if (ledController.isInState(fadingUp))
    {
        if (ledController.timeInCurrentState() > 2500)
            ledController.transitionTo(fadingDown);
        pwmValue = map(ledController.timeInCurrentState(), 0, 2500, 0, 250);
    }
    else if (ledController.isInState(fadingDown))
    {
        if (ledController.timeInCurrentState() > 2500)
            ledController.transitionTo(fadingUp);
        pwmValue = map(ledController.timeInCurrentState(), 0, 2500, 250, 0);
    }
    else if (ledController.isInState(fadingMax))
        pwmValue = 250;
    else if (ledController.isInState(ledIdle))
        pwmValue = 0;

    // led fading control as auto/manual mode
    if (actMode == AUTO)
        driveLed(true, 0, 0, pwmValue);
    else
        driveLed(false, pwmValue, 0, 0);

    // every 200ms checking sensor range 
    if (millis() - lastCheckVLTime > 50) // faster here - could you add averaged value so glitches less dominant?
                                         // Are you thinking it is really reason of current issue?, I thikn sensor value should be pretty accurate
    {
        static uint8_t range = 0;
        static uint8_t last_range = 0;
        static uint8_t sensorStatus;
        const uint8_t SAMPLECNT = 10; // this is essentially calibration
        const uint8_t  THRESHOLD = 10;
        //        const uint8_t THRESHOLDSPEED = 32;

#ifdef DEBUGSERIAL
        if (isMotorActivated())
            mySerial.print(F("RUNNING "));
#endif // DEBUGSERIAL

        //SENSE//
        range = vl.readRange();
        sensorStatus = vl.readRangeStatus();

        if (sensorStatus != VL6180X_ERROR_NONE)
            range = 200;
#ifdef DEBUGSERIAL
        //            mySerial.print(F(" R:"));
        mySerial.print(range);
#endif // DEBUGSERIAL

        avrValue = ((avrValue * 2) + range) / (2 + 1);

        if (stateMachine.isInState(idle)) // only recalibrate while idle;
        {
            avrCalibration = ((avrCalibration * SAMPLECNT) + range) / (SAMPLECNT + 1);
            avrCalibration = max(avrCalibration, avrValue); // pushes calibration to max quickly.
        }

        //       avrDelta = (avrDelta * 3 + (range - last_range)) / (3+1);
        //       avrDelta = 100 + (range - last_range);

#ifdef DEBUGSERIAL
        mySerial.print(F(" AVG: "));
        mySerial.print(avrValue);
        mySerial.print(F(" CAL: "));
        mySerial.println(avrCalibration);
#endif // DEBUGSERIAL
        if ((stateMachine.isInState(active) && actMode == AUTO) && ((avrCalibration - avrValue) <= THRESHOLD))// || range > 190 this will trigger on single bad reading.
        {
#ifdef DEBUGSERIAL
            mySerial.println(F("OFF(gone)"));
#endif // DEBUGSERIAL
            stateMachine.transitionTo(idle);
            ledController.transitionTo(fadingUp);
        }
        else if (stateMachine.isInState(deactiveDelay))
        {// confirm operator, change value is smaller than Threshold, correct?
            driveMotor(false);
            if (stateMachine.timeInCurrentState() > 1000 && (avrCalibration - avrValue) <= THRESHOLD) // might work, this means cleared.
            {
#ifdef DEBUGSERIAL
                mySerial.println(F("RESET(clear)"));
#endif // DEBUGSERIAL
                stateMachine.transitionTo(idle);
            }
        }
        //            stateMachine.transitionTo(idle);

        //DO//
        if (stateMachine.isInState(idle) && stateMachine.timeInCurrentState() > 500) // ignore movement on insertion
        {
            if (avrValue < avrCalibration && (avrCalibration - avrValue) >= THRESHOLD)
            {
#ifdef DEBUGSERIAL
                mySerial.println(F("ON(Detect)"));
#endif // DEBUGSERIAL
                stateMachine.transitionTo(active);
                ledController.transitionTo(ledIdle); // turn off led if sensor activated
            }
        }
        else if (stateMachine.isInState(active))
        {
            // deactivating part, after 500ms from active, if
            // decrasing speed is greater than 32, it will be deactivated, in this part, already 500ms delay
            if (stateMachine.timeInCurrentState() > 650 && (avrValue < last_range - 12))
            { // is this lifting correct?

                if (actMode == AUTO)
                {
#ifdef DEBUGSERIAL
                    mySerial.println(F("OFF(Motion)"));
#endif // DEBUGSERIAL
                    stateMachine.transitionTo(deactiveDelay);
                    ledController.transitionTo(fadingUp);
                }
            }
        }

        last_range = avrValue;
        lastCheckVLTime = millis();
    }


    if (settingButtonTrg.Q)
    {
#ifdef DEBUGSERIAL
        mySerial.println(F("sw2(chg mode)"));
#endif // DEBUGSERIAL
        if (actMode == AUTO)
            actMode = MANUAL;
        else
            actMode = AUTO;
        ledController.transitionTo(fadingMax);
        stateMachine.transitionTo(settingDelay);
    }

    if (stateMachine.isInState(active))
    {
        if (actMode == MANUAL)
        {
            if (grindButton.Q)
            {
                driveMotor(true);
                ledController.transitionTo(fadingMax);
                stateMachine.resetTime();
                motorWorked = true;
            }
            else
                driveMotor(false);

            if (stateMachine.timeInCurrentState() > NONEACTIONINTERVAL)
            {
                stateMachine.transitionTo(idle);
                ledController.transitionTo(fadingUp);
                /*
                if (motorWorked)
                stateMachine.transitionTo(activeDelay);
                else
                {
                stateMachine.transitionTo(idle);
                ledController.transitionTo(fadingUp);
                }
                */
            }
        }
        else if (actMode == AUTO)
        {
            driveMotor(true); // motor turn on
            ledController.transitionTo(fadingMax);
            if (stateMachine.timeInCurrentState() > NONEACTIONINTERVAL)
            {
                stateMachine.transitionTo(idle);
                ledController.transitionTo(fadingUp);
            }
        }
    }
    else if (stateMachine.isInState(activeDelay))
    {
        driveMotor(false);
        if (stateMachine.timeInCurrentState() > ACTIVEDELAY)
        {
            stateMachine.transitionTo(idle);
            ledController.transitionTo(fadingDown);
        }
    }
    else if (stateMachine.isInState(settingDelay))
    {
        driveMotor(false);
        if (stateMachine.timeInCurrentState() > SETTINGDELAY)
        {
            stateMachine.transitionTo(idle);
            ledController.transitionTo(fadingDown);
        }
    }
    else if (stateMachine.isInState(idle))
    {
        driveMotor(false);
    }

    //
    stateMachine.update();
    ledController.update();
}

void driveMotor(bool action)
{
    motorStatus = action;
#ifndef DEBUGSERIAL
    if(action)
        Palatis::SoftPWM.set(3, 125);
    else
        Palatis::SoftPWM.set(3, 0);
    // digitalWrite(MOTOR, action);
#endif
}

bool isMotorActivated()
{
    return motorStatus;
}

void activeEnter()
{
    motorWorked = false;
}