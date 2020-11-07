#ifndef _DEBOUNCED_BUTTON_H
#define _DEBOUNCED_BUTTON_H

/**
 * Prerequisites:
 * - pinMode(BTN_PIN, INPUT_PULLUP);
 * - attachInterrupt(digitalPinToInterrupt(BTN_PIN), fwdISR, CHANGE);
 * - void fwdISR() { debouncedBtn.ISRHandler(); }
 * Check change by comparing debouncedBtn.getPressCount()
 */
class DebouncedButton
{
    int pin;
    int debounceMs;

    bool pressed = false;
    unsigned long pressedBegin = 0;
    bool releasing = false;
    unsigned long releasedBegin = 0;

    volatile int pressCount = 0;
public:
    DebouncedButton(int _pin, int _debounceMs = 50);

    void ISRHandler();    
    int getPressCount() const;
};

#endif