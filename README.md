# iot-maple-mini-serial-grbl-fwd

## introduction

this forwarder is developed to allow stream and eventual preprocess some of gcode files to the marlin controller; the purpose of this project is to be used within a cnc machine and not with fdm printers for those marlin already provide everything needed.

## description

- forward gcode to [marlin](https://github.com/MarlinFirmware/Marlin) controller
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

## custom homing / zero scripts

If no scripts `fwdscr/homing.nc` and `fwdscr/zero.nc` found or sdcard not present on boot, then follow default scripts associated to `/home` and `/zero` commands:

*home*
```gcode
G28
```

*zero*
```gcode
G92X0Y0Z0
```

For delta robots its suggested to customize homing script in order to go in a safe zone where XY movements in the allowable radius doesn't make affectors exit their guidelines, for example:

```gcode
; homing
G28
; go to safe zone
G90
G0 Z200
```

## absolute and relative positioning respect to save/resume

when system start its default to absolute positioning G90, but during script G91 may used then each command sent to marlin controller keep tracked automatically to check current axis mode in order to restore the same situation if /resume command used after a /save and a power off.

## pausing job

a default queue of 4 cmds is used, this way planner can work having some prediction of subsequent commands but pause not take too much to receive ack of queued commands and stop the job.

## resuming job

the resume process can be called using `/resume` if a `/fwdstate.txt`, previously created by a `/save` command, exists. This file contains information about the filename and offset that was in execution when pause requested keeping track of the offset for the next command to execute if a print continue is requested.

this process executes follow actions in order:
- homing script ( or default if not present )
- ask user y/n to continue in the resume positioning ( first position XY then position Z )
- ask user y/n to continue in job execution

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
