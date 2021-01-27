#ifndef _CONFIG_H
#define _CONFIG_H

#define VERSION_NR_STR "0.4.0"
#define VERSION_NAME_STR "maple-mini-serial-grbl-fwd"
#define VERSION_NAME_SHORT_STR "GRBL FWD"

//
// SCRIPTS
//
#define HOMING_SCRIPT "M220S100\nG1F2000\nG28\nG90\nG0Z530\n"
#define ZERO_SCRIPT "G92X0Y0Z0\n"

// ------------------------------------------------------------

#define BAUD_RATE 115200
//
// retrieve value from BUFSIZE of Marlin Configuration_adv.h
//
#define MARLIN_BUFSIZE 4
#define SERIAL_READ_TIMEOUT_MS 100
#define UART_FIFO_SIZE 2048
#define RX_LINE_SIZE 100
#define F_BUFFER_SIZE 64

// grbl
#define MIN_SPEED_PERCENT 10

// report print progress and estimation each % ( 10, 20, ... )
#define F_PERCENT_PRINT_BASE 10
#define F_PERCENT_STAT_MINSEC 10

// utils
#define MORE_LINES_MAX 25

// sdcard
#define SDCARD_MOSI_PIN PA7
#define SDCARD_MISO_PIN PA6
#define SDCARD_SCK_PIN PA5
#define SDCARD_CHIPSEL PA4

// buttons
#define RESET_PIN PB3
#define PAUSE_RESUME_PIN PB4
#define SPEED_UP_PIN PB5
#define SPEED_DOWN_PIN PB6

// output pin to reset grbl controller board
#define RESETOUT_PIN PB12

#define DEBOUNCE_ROT_US 1500
#define DEBOUNCE_BTN_MS 200
#define DEBOUNCE_RESET_BTN_MS 1000

// uart attached to grbl controller
#define UART_SECOND_RX PA3
#define UART_SECOND_TX PA2

#endif