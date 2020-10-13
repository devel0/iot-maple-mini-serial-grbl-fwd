#ifndef _CONFIG_H
#define _CONFIG_H

#define VERSION_STR "maple-mini-serial-grbl-fwd 0.2"

#define BAUD_RATE 115200

// sdcard
#define SDCARD_MOSI_PIN PA7
#define SDCARD_MISO_PIN PA6
#define SDCARD_SCK_PIN PA5
#define SDCARD_CHIPSEL PA4

// uart attached to grbl controller
#define UART_SECOND_RX PA3
#define UART_SECOND_TX PA2
#define BACKSPACE 8

// buttons
#define RESET_PIN PB3
#define PAUSE_RESUME_PIN PB4
#define SPEED_UP_PIN PB5
#define SPEED_DOWN_PIN PB6
#define BTN_DEBOUNCE_FAST_MS 50
#define BTN_DEBOUNCE_MID_MS 200
#define BTN_DEBOUNCE_SLOW_MS 300
#define BTN_DEBOUNCE_SLOWEST_MS 1000

// output pin to reset grbl controller board
#define RESETOUT_PIN PB12

// states
#define STATE_NORMAL 0
#define STATE_WRITE_SD 1
#define STATE_SEND_SD 2

#endif