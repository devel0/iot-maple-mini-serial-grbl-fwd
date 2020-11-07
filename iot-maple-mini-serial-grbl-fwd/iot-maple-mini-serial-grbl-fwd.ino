#include "config.h"

#include <limits.h>
#include <SPI.h>

#include "MemoryInfo.h"
#include "Utils.h"
#include "Global.h"
#include "InputControls.h"
#include "SDCARD.h"

void setup()
{
    state = StateEnum::Setup;

    // setup pin modes
    pinMode(RESET_PIN, INPUT_PULLUP);
    pinMode(PAUSE_RESUME_PIN, INPUT_PULLUP);
    pinMode(SPEED_UP_PIN, INPUT_PULLUP);
    pinMode(SPEED_DOWN_PIN, INPUT_PULLUP);

    pinMode(RESETOUT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // reset if any
    unsigned long mm = micros();
    digitalWrite(RESETOUT_PIN, LOW);
    while (micros() - mm < 20)
        ;
    digitalWrite(RESETOUT_PIN, HIGH);

    // setup serials
    Serial.begin(BAUD_RATE);
    while (!Serial)
    {
    }

    Serial2.begin(BAUD_RATE);
    while (!Serial2)
    {
    }

    // setup builtin led
    digitalWrite(LED_BUILTIN, LOW);
    
    // attach interrupt for input controls
    attachInterrupt(digitalPinToInterrupt(RESET_PIN), reset_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PAUSE_RESUME_PIN), pause_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SPEED_UP_PIN), speedUp_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SPEED_DOWN_PIN), speedDown_ISR, CHANGE);

    doSetup();
}

void loop()
{
    mainLoop();
    
}
