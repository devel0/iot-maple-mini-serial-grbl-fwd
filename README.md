# iot-maple-mini-serial-grbl-fwd

## description

- forward gcode to marlin controller
- manage sdcard, list, send file
- manage grbl controller
    - reset
    - pause/unpause streaming sdcard file to grbl controller
    - change speed up/down (10%)
- save job and resume even if device turned off
- special commands starts with slash

![](data/doc/WIRINGS.png)

## syntax

```
/ver           display version
/ls            list sdcard content
/home          do homing G28, go to safe zone and switch to G54 working
/zero          set zero 0,0,0 here
/send <file>   load sdcard file and send to gcode controller
/more <file>   view file content
/scripts       reload scripts from sdcard
/reset         reset gcode controller
/pause         pause/resume gcode controller print
/save          save paused printing job
/resume        resume saved printing job
/abort         cancel current sd print
/info          sys info
```

## usage demo

[![asciicast](https://asciinema.org/a/371243.svg)](https://asciinema.org/a/371243)

## develop stm32f103 breakout board

- [PCB pdf](data/doc/BREAKOUT_BOARD.pdf)

- [EASYEDA project](https://easyeda.com/lorenzo.delana/mini-maple-stm32f103-breakout)

![](data/doc/breakout-board-top.png)

![](data/doc/breakout-board-bottom.png)

## notes on vscode debugging

- prerequisites
    - https://github.com/stm32duino/Arduino_Core_STM32
    - https://www.st.com/en/development-tools/stm32cubeprog.html
    - https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases
- update `.vscode` json files paths accordingly to your tools path
- connect STLINK V2 ( 4 wires 3.3-swdio-swclk-gnd )
