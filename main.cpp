#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>
//#include <hardware/pwm.h>
//#include "picopwm.h"
#include <hardware/i2c.h>
#include "si5351.h"
//#include "si5351mcu.h"


//Si5351mcu Si;


#define ln(x) (log(x)/log(2.718281828459045235f))

const uint LED_PIN = 25;
const uint ADC_PIN = 26;
const uint PWM_PIN = 15;
const uint BUTTON_GPIO = 1;
const uint AUDIO_SR = 8000;
uint16_t adc_raw;
int slice_num = 0;

static int xv[45];

bool txbutton = false;
float amplitude = 0;

float map(float xmap, float in_min, float in_max, float out_min, float out_max)
{
  return (xmap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void adc_handler() {
    adc_raw = adc_fifo_get();
    pwm_set(adc_raw);
}

void pwm_set(uint16_t sample){
  float df, amp;
  int samplerate = AUDIO_SR; // set sample-rate of microphone input here, use 4800 to 8000
  float adcdiv = 500000/AUDIO_SR; //divisor do adc
  int notvi = 1;   // set no TVI to true if you want constant carrier
  float freqssb = 0;

  sample = map(sample, 0, 4095, 0, 65535);
  ssb((float)sample/32767.0, samplerate, &amp, &df);

  freqssb = 7200000 + (df);
  txbutton = gpio_get(BUTTON_GPIO);
  if(txbutton == false){
    gpio_put(LED_PIN, 1);
    //pwm_set_enabled(slice_num, false);
    //pwm_set_gpio_level(PWM_PIN, amplitude); 
    //pwm0.setFrequency(freqssb);
    //pwm_set_enabled(slice_num, true);


    printf("SET FREQ \n");

    SetFrequency(freqssb);
    TX_ON();
    //sleep_ms(0.208);

    //Si.enable(0);
    //Si.setFreq(0, 7200000);

    printf("SET FREQ DONE , freq = %i \n", freqssb);

  }else{
    //pwm_set_enabled(slice_num, false);

      TX_OFF();
      // Si.disable(0);

      gpio_put(LED_PIN, 0);

      }
}


void filter(int val, int* i, int* q)
{
    int j;

    for (j = 0; j < 44; j++) {
        xv[j] = xv[j+1];
    }
    xv[44] = val;

    *i = xv[22];

    int _q = (xv[1] + xv[3] + xv[5] + xv[7]+xv[7] + 4*xv[9] + 6*xv[11] \
        + 9*xv[13] + 14*xv[15] + 23*xv[17] + 41*xv[19] + 127*xv[21] \
        - (127*xv[23] + 41*xv[25] + 23*xv[27] + 14*xv[29] + 9*xv[31] \
        + 6*xv[33] + 4*xv[35] + xv[37]+xv[37] + xv[39] + xv[41] + xv[43]) ) / 202;
 

  *q = _q;
}

int arctan2(int y, int x)
{
   int abs_y = abs(y);
   int angle;
   if(x >= 0){
      angle = 45 - 45 * (x - abs_y) / ((x + abs_y)==0?1:(x + abs_y));
   } else {
      angle = 135 - 45 * (x + abs_y) / ((abs_y - x)==0?1:(abs_y - x));
   }
   return (y < 0) ? -angle : angle; // negate if in quad III or IV
}

static float t = 0;
static float prev_f = 0;
static float prev_phase = 0;
static float acc = 0;

void ssb(float in, float fsamp, float* amp, float* df)
{
   int i, q;
   float phase;

   t++;
   filter(in * 128, &i, &q);
   *amp = sqrt( i*i + q*q) / 128.0f;
   amplitude = *amp;
   if(*amp > 1.0){
     printf("amp overflow %f\n", *amp);
     *amp = 1.0;
   }
   phase = M_PI + ((float)arctan2(q,i)) * M_PI/180.0f;
   float dp = phase - prev_phase;
   if(dp < 0) dp = dp + 2*M_PI;
   prev_phase = phase;

   *df = dp*fsamp/(2.0f*M_PI);
}

int main(){
    int samplerate = AUDIO_SR; // set sample-rate of microphone input here, use 4800 to 8000
    float adcdiv = 500000/AUDIO_SR; //divisor do adc
    int notvi = 1;   // set no TVI to true if you want constant carrier
    float freqssb = 0;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    stdio_init_all();

    i2c_init(i2c1, 800000);

    gpio_set_function(6, GPIO_FUNC_I2C);
    gpio_set_function(7, GPIO_FUNC_I2C);
    gpio_pull_up(6);
    gpio_pull_up(7);
  
    SetFrequency(7200000);
    SetParkMode();
    TX_ON();
    SetPower(4);

    //Si.init();

    //Si.correction(-1250);
    
    // set max power
    //Si.setPower(0, SIOUT_8mA);
    //Si.setFreq(0, 7200000);
    //Si.enable(0);

    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    //PicoPwm pwm0 = PicoPwm(PWM_PIN);
    //int slice_num = pwm0.getSlice();

    adc_init();
    adc_gpio_init(ADC_PIN);

    // Calculate the correct clock divider for the desired sample rate
    adc_set_clkdiv(adcdiv);

    adc_irq_set_enabled(true);
    adc_fifo_setup(
        true,
        false,
        1,
        false,
        true
    );

    sleep_ms(1000);

    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_handler);
    irq_set_priority(ADC_IRQ_FIFO, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    adc_run(true);



    while(true) {
      
    }

  return 0;

}