#include "pin.h"
#include "stm32f303xe.h"

extern int PinMode(GPIO_TypeDef * pCol, int pinNr, PINMODE pinMode) {

    if (pinNr > 15 ) {
        return -1;
    }

    switch (pinMode)
    {
    case OUTPUT:
        pCol -> MODER |= (0x1 << pinNr*0x2);
        break;
    case INPUT_PULLUP:
        pCol -> MODER | (0x1 << pinNr*0x2);
        pCol -> PUPDR | (0x1 << pinNr*0x2);
        break;
    default:
        break;
    }
    return 0;
}

extern int PinDWrite(GPIO_TypeDef * pCol, int pinNr, DIGI_VALUE value) {
    if (pinNr > 15 ) {
        return -1;
    }

    switch (value)
    {
    case LOW:
        pCol->ODR &= ~(0x1 << pinNr);
        break;
    case HIGH:
        pCol->ODR |= (0x1 << pinNr);
        break;
    case INVERT:
        pCol->ODR ^= (0x1 << pinNr);
        break;
    default:
        break;
    }
    return 0;
}

extern int PinDRead(GPIO_TypeDef * pCol, int pinNr) {

    if (pinNr > 15 ) {
        return -1;
    }

    if (pCol->IDR & (0x1 << pinNr)) {
        return 1;
    } else {
        return 0;
    }

}
