#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "config.h"
#include <Arduino.h>

#include "SerialHelper.h"

// states
typedef enum StateEnum
{
    Setup = 0,
    Normal = 1,
    SendSD = 2,
    SendSDPaused = 3,
    Aborting = 4,

    ExecutingScript = 100,

    Error = 255
};

extern StateEnum state;

extern HardwareSerial Serial2;

extern int speedUpReqCnt;
extern int speedDownReqCnt;

void doSetup();

void mainLoop();

/** reset fwd so then grbl controller due to RSTOUT connection */
void doReset();

void doPauseResume();
void doSpeedUp();
void doSpeedDown();

void toggleLed();
void ledOn();
void ledOff();

#endif