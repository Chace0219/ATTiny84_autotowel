#include <Wire.h>
#include "FBD.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "FiniteStateMachine.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();

// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 32
// Example settings structure
struct StoreStruct {
    // This is for mere detection if they are your settings
    char version[4];
    // The variables of your settings
    uint32_t motorActTime; // ms unit
} setting = {
    CONFIG_VERSION,
    // The default values
    2000
};

void loadConfig() {
    // To make sure there are settings, and they are YOURS!
    // If nothing is found it will use the default settings.
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

TON outerShort(250);
TON centerShort(250);
TON centerLong(2000);
Rtrg outerShortTrg, centerShortTrg, centerLongTrg;

State idle(NULL);
State active(NULL);
FiniteStateMachine stateMachine(idle);

#define MANUAL false
#define AUTO true
bool actMode = MANUAL;

void triggerLed(bool red, bool green, bool blue)
{
    /*
    digitalWrite(LEDSPIN, HIGH);
    if (red)
        digitalWrite(REDPIN, LOW);
    else
        digitalWrite(REDPIN, HIGH);

    if (green)
        digitalWrite(GREENPIN, LOW);
    else
        digitalWrite(GREENPIN, HIGH);

    if (blue)
        digitalWrite(BLUEPIN, LOW);
    else
        digitalWrite(BLUEPIN, HIGH);
        */
    
    digitalWrite(LEDSPIN, LOW);
    if(red)
        digitalWrite(REDPIN, HIGH);
    else
        digitalWrite(REDPIN, LOW);

    if (green)
        digitalWrite(GREENPIN, HIGH);
    else
        digitalWrite(GREENPIN, LOW);

    if (blue)
        digitalWrite(BLUEPIN, HIGH);
    else
        digitalWrite(BLUEPIN, LOW);// */
}

// 
void powerSensor(bool turnOn)
{
    if(turnOn)
        digitalWrite(TSL_VDD, HIGH);
    else
        digitalWrite(TSL_VDD, LOW);
}

void setup()
{
    pinMode(VBATPIN, INPUT);
    pinMode(BUTTONS, INPUT);

    pinMode(IRLED, OUTPUT);
    pinMode(OPTICALPIN, INPUT);

    pinMode(MOTOR, OUTPUT);
    digitalWrite(MOTOR, LOW); // motor drive pin as LOW, PA5

    pinMode(LEDSPIN, OUTPUT);
    pinMode(REDPIN, OUTPUT);
    pinMode(GREENPIN, OUTPUT);
    pinMode(BLUEPIN, OUTPUT);

    pinMode(TSL_VDD, OUTPUT);
    digitalWrite(TSL_VDD, LOW);

    powerSensor(true);
    delay(100);
    if (!vl.begin()) 
    {
    }
}

bool lastIRStatus = false;

void loop()
{
    triggerLed(true, true, true);
    // button proc
    static uint16_t buttonADC = 0;
    buttonADC = analogRead(BUTTONS);

    digitalWrite(LEDSPIN, LOW);
    digitalWrite(GREENPIN, LOW);
    outerShort.IN = buttonADC > 450 && buttonADC < 750;
    outerShort.update();
    outerShortTrg.IN = outerShort.Q;
    outerShortTrg.update();
    if (outerShortTrg.Q)
    {
        // digitalWrite(LEDSPIN, LOW);
        // digitalWrite(GREENPIN, HIGH);
    }
    /*
    if ((outerShort.Q && stateMachine.isInState(active)) && actMode == MANUAL)
    {
        driveMotor(true);
        stateMachine.resetTime();
    }
    else
        driveMotor(false);
    */

    centerShort.IN = buttonADC < 200;
    centerShort.update();
    centerShortTrg.IN = centerShort.Q;
    centerShortTrg.update();
    if (centerShortTrg.Q)
    {
        digitalWrite(LEDSPIN, HIGH);
        digitalWrite(GREENPIN, LOW);

        // CENTER BUTTON
        // stateMachine.transitionTo(active);
    }

    centerLong.IN = buttonADC < 100;
    centerLong.update();
    centerLongTrg.IN = centerLong.Q;
    centerLongTrg.update();
    if (centerLongTrg.Q)
    {
        // CENTER button long pressed
        
    }

    /*
    if (stateMachine.isInState(active))
    {
        if (actMode == AUTO)
            triggerLed(false, true, true);
        else // manual mode
            triggerLed(true, false, false);
        
        if (stateMachine.timeInCurrentState() > 4000)
            stateMachine.transitionTo(idle);
    }
    else if(stateMachine.isInState(idle))
    {
        triggerLed(false, false, false);
    }
    */

    // 
    stateMachine.update();
}

void driveMotor(bool action)
{
    digitalWrite(MOTOR, action);
}