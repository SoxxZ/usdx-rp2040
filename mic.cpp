#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>
#include <hardware/pwm.h>
#include "picopwm.h"

#define F_MCU 125000000

#define MORE_MIC_GAIN   1       // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#ifdef MORE_MIC_GAIN
volatile uint8_t vox_thresh = (1 << 2);
#else
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);
#endif

#define CARRIER_COMPLETELY_OFF_ON_LOW  1

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

volatile uint8_t tx = 0;
volatile uint8_t filt = 0;

#define MAX_DP  ((filt == 0) ? _UA : (filt == 3) ? _UA/4 : _UA/2) 

uint8_t lut[256];
volatile uint8_t amp;

int amplitude;

#define F_SAMP_TX 4800 //4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#if(F_MCU != 20000000)
const int16_t _F_SAMP_TX = (F_MCU * 4800LL / 20000000);  // Actual ADC sample-rate; used for phase calculations
#else
#define _F_SAMP_TX  F_SAMP_TX
#endif

#define _UA  600 //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision

volatile uint8_t mode = 0; //0 for lsb, 1 for usb

#define MULTI_ADC  1 

const uint LED_PIN = 25;
const uint ADC_PIN = 26;
const uint PWM_PIN = 15;
const uint BUTTON_GPIO = 1;

static int xv[45];

bool txbutton = false;

int maxvalue = -20000;
int minvalue = 20000;

int printxv = 0;

float map(float xmap, float in_min, float in_max, float out_min, float out_max)
{
  return (xmap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(){

    float freqssb = 0;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    stdio_init_all();
    printf("ADC Example, measuring GPIO26\n");


    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    PicoPwm pwm0 = PicoPwm(PWM_PIN);
    int slice_num = pwm0.getSlice();

    //pwm_set_enabled(slice_num, false);
    //pwm0.setDutyPercentage(50);
    //pwm_set_enabled(slice_num, true);

    adc_init();
    adc_set_clkdiv(11.3378684807);
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);
    
    int16_t data = adc_read();


    while(true) {

      data = adc_read();
      data = map(data, 0, 4095, 0, 64);
      txbutton = gpio_get(BUTTON_GPIO);
      if(txbutton == false){
        gpio_put(LED_PIN, 1);
        //pwm_set_enabled(slice_num, false);
        
        pwm_set_gpio_level(PWM_PIN, data); 

        pwm_set_enabled(slice_num, true);
      }else{
        pwm_set_enabled(slice_num, false);
        gpio_put(LED_PIN, 0);
      }
    }

  return 0;

} 