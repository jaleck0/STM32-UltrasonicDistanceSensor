#ifndef PIN_H
#define PIN_H

#include "stm32f303xe.h"

typedef enum {
    OUTPUT,
    INPUT_PULLUP
} PINMODE;

typedef enum {
    LOW,
    HIGH,
    INVERT
} DIGI_VALUE;

extern int PinMode(GPIO_TypeDef * pCol, int pinNr, PINMODE pinMode);

extern int PinDWrite(GPIO_TypeDef * pCol, int pinNr, DIGI_VALUE value);

extern int PinDRead(GPIO_TypeDef * pCol, int pinNr);

#endif