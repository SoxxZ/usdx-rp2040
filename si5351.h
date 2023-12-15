#ifndef SI5351_H_
#define SI5351_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef uint8_t byte;

#define i2caddr 0x60 

void TX_ON ();
void TX_OFF ();
void SetFrequency (unsigned long frequency);
void SetPower (byte power);
void SetParkMode ();
void Si5351a_Write_Reg (byte regist, byte value);


#endif
