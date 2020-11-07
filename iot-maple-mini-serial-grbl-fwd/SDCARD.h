#ifndef _SDCARD_H
#define _SDCARD_H

#include "config.h"
#include "Global.h"
#include <Arduino.h>
#include <SD.h>

bool init_sd_card();

void sdcard_ls();
int sdcard_send(String filename);
void sdcard_rm(String filename);
File sdcard_open(String filename);

#endif