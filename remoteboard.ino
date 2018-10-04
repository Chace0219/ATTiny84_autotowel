
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

#define LIFTTURNOFF 60 // mm lift until shutoff
#define DEBOUNCETIME 50 // ms unit

#define NONEACTIONINTERVAL 5000 // ms unit
#define ACTIVEDELAY 5000 // ms unit
#define SETTINGDELAY 2000 // ms unit

bool motorStatus = false;
bool motorWorked = false;

#define DEBUGSERIAL
#ifdef DEBUGSERIAL
#include <SoftwareSerial.h>
SoftwareSerial mySerial(255, 5); // RX, TX
#endif

#define RANGETHRESHOLD 50
SOFTPWM_DEFINE_CHANNEL(2, DDRA, PORTA, PORTA7);
SOFTPWM_DEFINE_CHANNEL(0, DDRB, PORTB, PORTB0);
SOFTPWM_DEFINE_CHANNEL(1, DDRB, PORTB, PORTB1);
SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(5, 100);
SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(5, 100);

Adafruit_VL6180X vl = Adafruit_VL6180X();

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

int StartRange = 0; // range when motor goes active (entry point)

TON grindButton(DEBOUNCETIME);
TON settingButton(DEBOUNCETIME);
Rtrg grindButtonTrg, settingButtonTrg;

void activeEnter();

State idle(NULL);
State active(activeEnter, NULL, NULL);
State activeDelay(NULL);
State deactiveDelay(NULL);
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
    // mySerial.println(F("V1.4"));
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
        // debugPrintln(F("Sensor failed"));
    }
    else
    {
        // debugPrintln(F("Sensor ok"));
    }

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
    grindButton.IN = buttonADC > 375;
    grindButton.update();
    grindButtonTrg.IN = grindButton.Q;
    grindButtonTrg.update();
#ifdef DEBUGSERIAL
    if (grindButtonTrg.Q)
    {
        mySerial.print(F("GRIND KEY "));
        mySerial.print(millis());
        mySerial.print(F(" ADC:"));
        mySerial.println(buttonADC); // */
    }
#endif

    // check SW2 switch
    settingButton.IN = buttonADC < 30; // that part
    settingButton.update();
    settingButtonTrg.IN = settingButton.Q;
    settingButtonTrg.update();
#ifdef DEBUGSERIAL
    if (settingButtonTrg.Q)
        mySerial.println(F("SETTING KEY"));
#endif

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
    {
        static uint8_t range = 0;
        static uint8_t last_range = 0;
        static uint8_t sensorStatus;
        const uint8_t SAMPLECNT = 10; // this is essentially calibration
        const uint8_t  THRESHOLD = 10;

#ifdef DEBUGSERIAL
        if (isMotorActivated())
            mySerial.println(F("RUNNING "));
#endif
        //SENSE//
        range = vl.readRange();
        sensorStatus = vl.readRangeStatus();
        if (sensorStatus != VL6180X_ERROR_NONE)
            range = 200;

#ifdef DEBUGSERIAL
        if (actMode)
        {
            mySerial.print(F(" R: "));
            mySerial.print(range);
        }
#endif
        avrValue = ((avrValue * 2) + range) / (2 + 1);
        if (stateMachine.isInState(idle)) // only recalibrate while idle;
        {
            avrCalibration = ((avrCalibration * SAMPLECNT) + range) / (SAMPLECNT + 1);
            avrCalibration = max(avrCalibration, avrValue); // pushes calibration to max quickly.
        }

#ifdef DEBUGSERIAL
        if (actMode)
        {
            mySerial.print(F(" AVG: "));
            mySerial.print(avrValue);
            mySerial.print(F(" CAL: "));
            mySerial.println(avrCalibration);
        }
#endif

        if ((stateMachine.isInState(active) && actMode == AUTO) && ((avrCalibration - avrValue) <= THRESHOLD))// || range > 190 this will trigger on single bad reading.
        {
#ifdef DEBUGSERIAL
            mySerial.println(F("OFF(gone)"));
#endif
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
#endif
                stateMachine.transitionTo(idle);
            }
        }

        if (stateMachine.isInState(idle) && stateMachine.timeInCurrentState() > 500) // ignore movement on insertion
        {
            if (avrValue < avrCalibration && (avrCalibration - avrValue) >= THRESHOLD)
            {
#ifdef DEBUGSERIAL
                mySerial.println(F("ON(Detect)"));
#endif
                StartRange = avrValue;//record height at which object appears // HERE

                stateMachine.transitionTo(active);
                ledController.transitionTo(ledIdle); // turn off led if sensor activated
            }
        }
        else if (stateMachine.isInState(active))
        {
            // deactivating part, after 500ms from active, if
            // decrasing speed is greater than 32, it will be deactivated, in this part, already 500ms delay
            if (stateMachine.timeInCurrentState() > 650 && (avrValue < StartRange - LIFTTURNOFF)) // HERE
            { // is this lifting correct?
                if (actMode == AUTO)
                {
#ifdef DEBUGSERIAL
                    mySerial.println(F("OFF(Motion)"));
#endif
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
        if (actMode == AUTO)
            actMode = MANUAL;
        else
            actMode = AUTO;
#ifdef DEBUGSERIAL
        mySerial.print(F("MODE "));
        if (actMode)
            mySerial.println(F("AUTO"));
        else
            mySerial.println(F("MANUAL"));
#endif
        // ledController.transitionTo(fadingMax);
        stateMachine.transitionTo(idle);
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
            {
                driveMotor(false);
                stateMachine.transitionTo(idle);
                ledController.transitionTo(fadingDown);
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
    else if (stateMachine.isInState(idle))
    {
        driveMotor(false);
        if (actMode == MANUAL)
        {
            if (grindButton.Q)
            {
#ifdef DEBUGSERIAL
                mySerial.println(F("GRIND "));
                // mySerial.println(millis());
#endif
                stateMachine.transitionTo(active);
            }
        }
    }

    //
    stateMachine.update();
    ledController.update();
}

void driveMotor(bool action)
{
    motorStatus = action;
#ifndef DEBUGSERIAL
    digitalWrite(MOTOR, action);
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