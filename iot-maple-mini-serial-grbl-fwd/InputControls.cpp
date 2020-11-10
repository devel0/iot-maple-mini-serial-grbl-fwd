#include "InputControls.h"

#include "Utils.h"

DebouncedButton resetBtn(RESET_PIN, DEBOUNCE_RESET_BTN_MS);
DebouncedButton pauseBtn(PAUSE_RESUME_PIN, DEBOUNCE_BTN_MS);
DebouncedButton speedUpBtn(SPEED_UP_PIN, DEBOUNCE_BTN_MS);
DebouncedButton speedDownBtn(SPEED_DOWN_PIN, DEBOUNCE_BTN_MS);

int resetBtnPressCountPrev = 0;
void reset_ISR()
{
    resetBtn.ISRHandler();

    if (resetBtn.getPressCount() != resetBtnPressCountPrev)
    {
        resetBtnPressCountPrev = resetBtn.getPressCount();
        doReset();
    }
}

int pauseBtnPressCountPrev = 0;
void pause_ISR()
{
    pauseBtn.ISRHandler();

    if (pauseBtn.getPressCount() != pauseBtnPressCountPrev)
    {
        pauseBtnPressCountPrev = pauseBtn.getPressCount();
        doPauseResume();
    }
}

int speedUpBtnPressCountPrev = 0;
void speedUp_ISR()
{
    speedUpBtn.ISRHandler();

    if (speedUpBtn.getPressCount() != speedUpBtnPressCountPrev)
    {
        speedUpBtnPressCountPrev = speedUpBtn.getPressCount();
        doSpeedUp();
    }
}

int speedDownBtnPressCountPrev = 0;
void speedDown_ISR()
{
    speedDownBtn.ISRHandler();

    if (speedDownBtn.getPressCount() != speedDownBtnPressCountPrev)
    {
        speedDownBtnPressCountPrev = speedDownBtn.getPressCount();
        doSpeedDown();
    }
}
