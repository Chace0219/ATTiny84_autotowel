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


uint8_t currLevel = 0;
TON sw1Ton(250);
TON sw2Ton(250);
Rtrg sw1Trg, sw2Trg;

State idle(NULL);
State tslActive(NULL);
State motorActive(NULL);
FiniteStateMachine controlMachine(idle);

void driveLed(uint8_t index, bool bTurnOn)
{
    if (!bTurnOn)
    {
        digitalWrite(LEDSPIN, LOW);
        digitalWrite(REDPIN, LOW);
        digitalWrite(GREENPIN, LOW);
        digitalWrite(BLUEPIN, LOW);
    }
    else
    {
        switch (index)
        {

        case 0:
        {
            digitalWrite(LEDSPIN, HIGH);
            digitalWrite(REDPIN, LOW);
            digitalWrite(GREENPIN, HIGH);
            digitalWrite(BLUEPIN, HIGH);
        }
        break;

        case 1:
        {
            digitalWrite(LEDSPIN, HIGH);
            digitalWrite(REDPIN, HIGH);
            digitalWrite(GREENPIN, LOW);
            digitalWrite(BLUEPIN, HIGH);
        }
        break;

        case 2:
        {
            digitalWrite(LEDSPIN, HIGH);
            digitalWrite(REDPIN, HIGH);
            digitalWrite(GREENPIN, HIGH);
            digitalWrite(BLUEPIN, LOW);
        }
        break;

        case 3:
        {
            digitalWrite(LEDSPIN, LOW);
            digitalWrite(REDPIN, HIGH);
            digitalWrite(GREENPIN, LOW);
            digitalWrite(BLUEPIN, LOW);
        }
        break;

        case 4:
        {
            digitalWrite(LEDSPIN, LOW);
            digitalWrite(REDPIN, LOW);
            digitalWrite(GREENPIN, HIGH);
            digitalWrite(BLUEPIN, LOW);
        }
        break;

        case 5:
        {
            digitalWrite(LEDSPIN, LOW);
            digitalWrite(REDPIN, LOW);
            digitalWrite(GREENPIN, LOW);
            digitalWrite(BLUEPIN, HIGH);
        }
        break;
        }
    }
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
uint8_t ledIdx = 0;

void loop()
{
    /*
    static uint32_t lastIRLedTime = millis();
    if (millis() - lastIRLedTime > 500)
    {
    lastIRLedTime = millis();
    if (lastIRStatus)
    {
    // driveLed(1, LOW);
    lastIRStatus = false;
    }
    else
    {
    // driveLed(1, HIGH);
    lastIRStatus = true;
    }
    }

    digitalWrite(IRLED, lastIRStatus);
    driveLed(1, HIGH);
    */

    /*
    if (digitalRead(OPTICALPIN) == HIGH)
    driveLed(1, HIGH);
    else
    driveLed(1, LOW);
    */
    // */


    /*
    driveLed(currLevel, HIGH);

    // button proc
    static uint16_t buttonADC = 0;
    buttonADC = analogRead(BUTTONS);
    sw1Ton.IN = buttonADC > 450 && buttonADC < 750;
    sw1Ton.update();
    sw1Trg.IN = sw1Ton.Q;
    sw1Trg.update();
    if (sw1Trg.Q)
    {
    if(currLevel < 2)
    currLevel++;
    }

    sw2Ton.IN = buttonADC < 200;
    sw2Ton.update();
    sw2Trg.IN = sw2Ton.Q;
    sw2Trg.update();
    if (sw2Trg.Q)
    {
    if (currLevel > 0)
    currLevel--;
    }

    if (controlMachine.isInState(idle))
    {
    digitalWrite(TSL_VDD, LOW);
    pinMode(IRLED, OUTPUT);
    digitalWrite(IRLED, HIGH);
    digitalWrite(MOTOR, LOW);


    if (controlMachine.timeInCurrentState() > 250)
    {
    digitalWrite(TSL_VDD, HIGH);
    digitalWrite(IRLED, LOW);
    pinMode(IRLED, INPUT);
    delay(50);
    configureSensor();
    controlMachine.transitionTo(tslActive);
    }
    }
    else if (controlMachine.isInState(tslActive))
    {
    digitalWrite(TSL_VDD, HIGH);
    digitalWrite(MOTOR, LOW);
    if (digitalRead(IRLED))
    {
    controlMachine.transitionTo(motorActive);
    }
    else if (controlMachine.timeInCurrentState() > 250)
    {
    digitalWrite(TSL_VDD, LOW);
    pinMode(IRLED, OUTPUT);
    digitalWrite(IRLED, HIGH);
    controlMachine.transitionTo(idle);
    }
    }
    else if (controlMachine.isInState(motorActive))
    {
    digitalWrite(TSL_VDD, LOW);
    pinMode(IRLED, OUTPUT);
    digitalWrite(IRLED, LOW);
    digitalWrite(MOTOR, HIGH);
    }
    */

    static uint32_t checkSensorTime = millis();
    if (millis() - checkSensorTime > 10)
    {
        uint8_t range = vl.readRange();
        uint8_t status = vl.readRangeStatus();

        digitalWrite(LEDSPIN, LOW);
        digitalWrite(REDPIN, LOW);
        digitalWrite(BLUEPIN, LOW);
        if (status == VL6180X_ERROR_NONE)
        {
            uint8_t pwmValue = map(range, 30, 150, 0, 255);
            analogWrite(GREENPIN, pwmValue);
            if (range < 100)
            {
                if (controlMachine.isInState(idle))
                    controlMachine.transitionTo(motorActive);
            }
        }
        else
        {
            digitalWrite(GREENPIN, LOW);
            if (controlMachine.isInState(motorActive))
                controlMachine.transitionTo(idle);
        }
        checkSensorTime = millis();
    }

    if (controlMachine.isInState(motorActive))
    {
        if (controlMachine.timeInCurrentState() < 2000)
            digitalWrite(MOTOR, HIGH);
        else
            digitalWrite(MOTOR, LOW);
    }
    else
    {
        digitalWrite(MOTOR, LOW);
    }
    controlMachine.update();
}

