#include "DebouncedButton.h"

#include "Utils.h"

DebouncedButton::DebouncedButton(int _pin, int _debounceMs)
{
    pin = _pin;
    debounceMs = _debounceMs;
}

void DebouncedButton::ISRHandler()
{
    int state = digitalRead(pin);
    unsigned long m = millis();

    if (state == HIGH && pressed) // signal HIGH
    {
        if (releasing)
        {
            if (timeDiff(m, releasedBegin) >= debounceMs)
            {
                pressed = releasing = false;
            }
        }
        else
        {
            releasing = true;
            releasedBegin = m;
        }
    }
    else // signal LOW
    {
        if (releasing)
        {
            if (timeDiff(m, releasedBegin) >= debounceMs)
            {
                pressed = releasing = false;
            }
        }

        if (pressed)
        {
            releasing = false;
        }
        else
        {
            pressed = true;
            pressedBegin = m;
            ++pressCount;            
        }
    }
}

int DebouncedButton::getPressCount() const
{
    return pressCount;
}