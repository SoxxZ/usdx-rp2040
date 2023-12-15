#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include "si5351.h"


//void setup() {
//  Wire.begin();                        // Initialize I2C-communication as master
//                                       //    SDA on pin ADC04
//                                       //    SCL on pin ADC05
//  SetFrequency (10140000);             // Set TX-Frequency [10,14 MHz]
//  SetParkMode ();                      // Intialize park mode
//}

//void loop() {
//   TX_ON();                            // Switches transmitter on
//   SetPower(4);                        // Nothing usefull, just changing between different power levels
//   delay(10000);                       
//   SetPower(3);
//   delay(10000);
//   SetPower(2);
//   delay(10000);
//   SetPower(1);
//   delay(10000);
//   TX_OFF();                           // Switches transmitter off
//}



void TX_ON () {                        // Enables output on CLK0 and disables Park Mode on CLK1
  Si5351a_Write_Reg (17, 128);         // Disable output CLK1
  Si5351a_Write_Reg (16, 79);          // Enable output CLK0, set crystal as source and Integer Mode on PLLA
}

void TX_OFF () {                       // Disables output on CLK0 and enters Park Mode on CLK1
  Si5351a_Write_Reg (16, 128);         // Disable output CLK0
  Si5351a_Write_Reg (17, 111);         // Enable output CLK1, set crystal as source and Integer Mode on PLLB
}

void SetFrequency (unsigned long frequency) { // Frequency in Hz; must be within [7810 Hz to 200 Mhz]
  #define F_XTAL 25000000;             // Frequency of Quartz-Oszillator
  #define c 1048574;                   // "c" part of Feedback-Multiplier from XTAL to PLL
  unsigned long fvco;                  // VCO frequency (600-900 MHz) of PLL
  unsigned long outdivider;            // Output divider in range [4,6,8-900], even numbers preferred
  byte R = 1;                          // Additional Output Divider in range [1,2,4,...128]
  byte a;                              // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  unsigned long b;                     // "b" part of Feedback-Multiplier from XTAL to PLL
  float f;                             // floating variable, needed in calculation
  unsigned long MS0_P1;                // Si5351a Output Divider register MS0_P1, P2 and P3 are hardcoded below
  unsigned long MSNA_P1;               // Si5351a Feedback Multisynth register MSNA_P1
  unsigned long MSNA_P2;               // Si5351a Feedback Multisynth register MSNA_P2
  unsigned long MSNA_P3;               // Si5351a Feedback Multisynth register MSNA_P3

  outdivider = 900000000 / frequency;  // With 900 MHz beeing the maximum internal PLL-Frequency
  
  while (outdivider > 900){            // If output divider out of range (>900) use additional Output divider
    R = R * 2;
    outdivider = outdivider / 2;
  }
  if (outdivider % 2) outdivider--;    // finds the even divider which delivers the intended Frequency

  fvco = outdivider * R * frequency;   // Calculate the PLL-Frequency (given the even divider)

  switch (R){                          // Convert the Output Divider to the bit-setting required in register 44
    case 1: R = 0; break;              // Bits [6:4] = 000
    case 2: R = 16; break;             // Bits [6:4] = 001
    case 4: R = 32; break;             // Bits [6:4] = 010
    case 8: R = 48; break;             // Bits [6:4] = 011
    case 16: R = 64; break;            // Bits [6:4] = 100
    case 32: R = 80; break;            // Bits [6:4] = 101
    case 64: R = 96; break;            // Bits [6:4] = 110
    case 128: R = 112; break;          // Bits [6:4] = 111
  }

  a = fvco / F_XTAL;                   // Multiplier to get from Quartz-Oscillator Freq. to PLL-Freq.
  f = fvco - a * F_XTAL;               // Multiplier = a+b/c
  f = f * c;                           // this is just "int" and "float" mathematics
  f = f / F_XTAL;
  b = f;

  MS0_P1 = 128 * outdivider - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                       // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below

  f = 128 * b / c;                     // Calculation of Feedback Multisynth registers MSNA_P1 to MSNA_P3
  MSNA_P1 = 128 * a + f - 512;
  MSNA_P2 = f;
  MSNA_P2 = 128 * b - MSNA_P2 * c; 
  MSNA_P3 = c;

  //Si5351a_Write_Reg (16, 128);                      // Disable output during the following register settings 
  Si5351a_Write_Reg (26, (MSNA_P3 & 65280) >> 8);   // Bits [15:8] of MSNA_P3 in register 26
  Si5351a_Write_Reg (27, MSNA_P3 & 255);            // Bits [7:0]  of MSNA_P3 in register 27
  Si5351a_Write_Reg (28, (MSNA_P1 & 196608) >> 16); // Bits [17:16] of MSNA_P1 in bits [1:0] of register 28
  Si5351a_Write_Reg (29, (MSNA_P1 & 65280) >> 8);   // Bits [15:8]  of MSNA_P1 in register 29
  Si5351a_Write_Reg (30, MSNA_P1 & 255);            // Bits [7:0]  of MSNA_P1 in register 30
  Si5351a_Write_Reg (31, ((MSNA_P3 & 983040) >> 12) | ((MSNA_P2 & 983040) >> 16)); // Parts of MSNA_P3 und MSNA_P1
  Si5351a_Write_Reg (32, (MSNA_P2 & 65280) >> 8);   // Bits [15:8]  of MSNA_P2 in register 32
  Si5351a_Write_Reg (33, MSNA_P2 & 255);            // Bits [7:0]  of MSNA_P2 in register 33
  Si5351a_Write_Reg (42, 0);                        // Bits [15:8] of MS0_P3 (always 0) in register 42
  Si5351a_Write_Reg (43, 1);                        // Bits [7:0]  of MS0_P3 (always 1) in register 43
  Si5351a_Write_Reg (44, ((MS0_P1 & 196608) >> 16) | R);  // Bits [17:16] of MS0_P1 in bits [1:0] and R in [7:4]
  Si5351a_Write_Reg (45, (MS0_P1 & 65280) >> 8);    // Bits [15:8]  of MS0_P1 in register 45
  Si5351a_Write_Reg (46, MS0_P1 & 255);             // Bits [7:0]  of MS0_P1 in register 46
  Si5351a_Write_Reg (47, 0);                        // Bits [19:16] of MS0_P2 and MS0_P3 are always 0
  Si5351a_Write_Reg (48, 0);                        // Bits [15:8]  of MS0_P2 are always 0
  Si5351a_Write_Reg (49, 0);                        // Bits [7:0]   of MS0_P2 are always 0
  if (outdivider == 4){
    Si5351a_Write_Reg (44, 12 | R);                 // Special settings for R = 4 (see datasheet)
    Si5351a_Write_Reg (45, 0);                      // Bits [15:8]  of MS0_P1 must be 0
    Si5351a_Write_Reg (46, 0);                      // Bits [7:0]  of MS0_P1 must be 0
  } 
  //Si5351a_Write_Reg (177, 32);                      // This resets PLL A
}

void SetParkMode () {                               // Sets CLK1 to the Park Mode frequency of 150 MHz to keep the Si5351a warm during key-up
  //Si5351a_Write_Reg (17, 128);                      // Disable output during the following register settings
  Si5351a_Write_Reg (34, 255);                      // Bits [15:8] of MSNB_P3
  Si5351a_Write_Reg (35, 254);                      // Bits [7:0]  of MSNB_P3
  Si5351a_Write_Reg (36, 0);                        // Bits [17:16] of MSNB_P1 in bits [1:0]
  Si5351a_Write_Reg (37, 14);                       // Bits [15:8]  of MSNB_P1
  Si5351a_Write_Reg (38, 169);                      // Bits [7:0]  of MSNB_P1
  Si5351a_Write_Reg (39, 252);                      // Parts of MSNB_P3 und MSNB_P1
  Si5351a_Write_Reg (40, 130);                      // Bits [15:8]  of MSNB_P2
  Si5351a_Write_Reg (41, 82);                       // Bits [7:0]  of MSNB_P2
  Si5351a_Write_Reg (50, 0);                        // Bits [15:8] of MS1_P3
  Si5351a_Write_Reg (51, 1);                        // Bits [7:0]  of MS1_P3
  Si5351a_Write_Reg (52, 0);                        // Bits [17:16] of MS1_P1 in bits [1:0] and R in [7:4]
  Si5351a_Write_Reg (53, 1);                        // Bits [15:8]  of MS1_P1
  Si5351a_Write_Reg (54, 0);                        // Bits [7:0]  of MS1_P1
  Si5351a_Write_Reg (55, 0);                        // Bits [19:16] of MS1_P2 and MS1_P3
  Si5351a_Write_Reg (56, 0);                        // Bits [15:8]  of MS1_P2
  Si5351a_Write_Reg (57, 0);                        // Bits [7:0]   of MS1_P2
  //Si5351a_Write_Reg (177, 128);                     // This resets PLL B
}

void SetPower (byte power){                         // Sets the output power level
  if (power == 0 || power > 4){power = 0;}          // valid power values are 0 (25%), 1 (50%), 2 (75%) or 3 (100%)
  switch (power){
    case 1:
      Si5351a_Write_Reg (16, 76);                   // CLK0 drive strength = 2mA; power level ~ -8dB
      break;
    case 2:
      Si5351a_Write_Reg (16, 77);                   // CLK0 drive strength = 4mA; power level ~ -3dB
      break;
    case 3:
      Si5351a_Write_Reg (16, 78);                   // CLK0 drive strength = 6mA; power level ~ -1dB
      break;
    case 4:
      Si5351a_Write_Reg (16, 79);                   // CLK0 drive strength = 8mA; power level := 0dB
      break;
  }
}

void Si5351a_Write_Reg (byte regist, byte value){   // Writes "byte" into "regist" of Si5351a via I2C
  int retorno = 0;
  const uint8_t* reg  = &regist;
  const uint8_t* val = &value;
  uint8_t i2cdata[2] = {regist,value};
  
  printf("Writing to i2c 1");
  
  retorno = i2c_write_blocking(i2c1, i2caddr, i2cdata, 2, true);
  //sleep_us(200);
  printf("Done writing to i2c, status = %i \n", retorno);

}
