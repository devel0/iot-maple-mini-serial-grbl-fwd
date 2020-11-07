#ifndef _UTILS_H
#define _UTILS_H

#include <Arduino.h>

unsigned long timeDiff(unsigned long now, unsigned long start);
int atoin(const char *str, int len);
void printHumanSize(Print& stream, uint32_t size);
void printHumanSeconds(Print& stream, uint32_t seconds);
float strPtrGetFloatWhileDigits(const char **ptr);

#endif