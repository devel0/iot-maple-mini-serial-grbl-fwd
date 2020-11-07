#ifndef _INPUT_CONTROLS_H
#define _INPUT_CONTROLS_H

#include "config.h"
#include "Global.h"
#include "DebouncedButton.h"

extern DebouncedButton resetBtn;
extern DebouncedButton pauseBtn;
extern DebouncedButton speedUpBtn;
extern DebouncedButton speedDownBtn;

void reset_ISR();
void pause_ISR();
void speedUp_ISR();
void speedDown_ISR();

#endif