#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "config.h"
#include <Arduino.h>

// states
typedef enum StateEnum
{
    Setup = 0,
    Normal = 1,
    Aborting = 3,

    SendSDStartingQueryLocalPos = 20,
    SendSDStartingQueryLocalPosSent = 21,
    SendSDBegin = 22,
    SendSD = 23,    
    SendSDPausingQueryCurrentLocalPos = 24,
    SendSDPausingQueryCurrentLocalPosSent = 25,
    SendSDPausingQueryCurrentGlobalPos = 26,
    SendSDPausingQueryCurrentGlobalPosSent = 27,    
    SendSDPaused = 28,

    ResumeHoming = 50,
    ResumeStartPositionAsk = 51,
    ResumePosition = 52,
    ResumeStartSDFileAsk = 53,
    ResumeStartSDFile = 54
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