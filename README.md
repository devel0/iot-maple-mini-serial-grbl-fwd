# iot-maple-mini-serial-grbl-fwd

## description

- forward gcode to grbl controller
- manage sdcard, list, send file
- manage grbl controller
    - reset
    - pause streaming sdcard file to grbl controller
    - change speed up/down (10%)

![](data/doc/WIRINGS.png)

## todo

- allow to interleave gcode commands during sdcard print (ie. `M114` current coord)

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
