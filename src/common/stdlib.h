#ifndef __MEMORY_H__
#define __MEMORY_H__

#include <stddef.h>

void ltoa(char *buf, unsigned long i, int base);
char *ftoa(float f);
double atof(const char *s);
char *itoa(int value, char *str);

int atoi(char *s);

int sprintf(char *str, const char *format, ...);

#endif
