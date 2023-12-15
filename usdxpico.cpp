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

volatile uint8_t vox = 0;
volatile uint8_t tx = 0;
volatile uint8_t filt = 0;
volatile uint8_t drive = 2;
volatile uint8_t amp;
uint8_t lut[256];

#define _F_SAMP_TX  4800
#define _UA  600 //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
#define CARRIER_COMPLETELY_OFF_ON_LOW  1    // disable oscillator on low amplitudes, to prevent potential unwanted biasing/leakage through PA circuit
#define MAX_DP  ((filt == 0) ? _UA : (filt == 3) ? _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define MIC_ATTEN  0

#define MORE_MIC_GAIN   1       // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#ifdef MORE_MIC_GAIN
volatile uint8_t vox_thresh = (1 << 2);
#else
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);
#endif




inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z  //derived from (5) [1]   note that atan2 can overflow easily so keep _UA low
//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}


inline void _vox(bool trigger)
{
  if(trigger){
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  } else {
    if(tx) tx--;
  }
}


#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];
#ifdef MORE_MIC_GAIN
//#define DIG_MODE  // optimization for digital modes: for super flat TX spectrum, (only down < 100Hz to cut-off DC components)
#ifdef DIG_MODE
  int16_t ac = in;
  dc = (ac + (7) * dc) / (7 + 1);  // hpf: slow average
  v[15] = (ac - dc) / 2;           // hpf (dc decoupling)  (-6dB gain to compensate for DC-noise)
#else
  int16_t ac = in * 2;             //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)
#endif //DIG_MODE
  i = v[7] * 2;  // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i / 2, q / 2);  // -6dB gain (correction)
#else  // !MORE_MIC_GAIN
  //dc += (in - dc) / 2;       // fast moving average
  dc = (in + dc) / 2;        // average
  int16_t ac = (in - dc);   // DC decoupling
  //v[15] = ac;// - z1;        // high-pass (emphasis) filter
  v[15] = (ac + z1);// / 2;           // low-pass filter with notch at Fs/2
  z1 = ac;

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
#endif  // MORE_MIC_GAIN

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if(vox) _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef QUAD
  if(dp >= (_UA/2)){ dp = dp - _UA/2; quad = !quad; }
#endif

#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif

  return dp * ( _F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference

}


const uint LED_PIN = 25;
const uint ADC_PIN = 26;
const uint PWM_PIN = 15;
const uint BUTTON_GPIO = 1;

bool txbutton = false;

volatile int16_t adc_result[3]; 

#define ADC_RANGE 4096
#define ADC_BIAS  (ADC_RANGE/2)

float map(float xmap, float in_min, float in_max, float out_min, float out_max)
{
  return (xmap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void dsp_tx_ssb(){
  int16_t dfreq = 0;
  int freqssb = 7200000;

  dfreq = ssb(adc_result[2]);
  freqssb = freqssb + dfreq;
  SetFrequency(freqssb);
  TX_ON();
}



void dsp_tx_fm()
{ // jitter dependent things first      // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  int16_t dfreq = 0;
  int freqfm = 7200000;
  
  int16_t adc = adc_result[2] - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  dfreq = in;

  freqfm = freqfm + dfreq;
  SetFrequency(freqfm);
  TX_ON();
  
}


int main(){
  float adcdiv = 500000/_F_SAMP_TX; //divisor do adc

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
  TX_OFF();
  SetPower(4);

  gpio_init(BUTTON_GPIO);
  gpio_set_dir(BUTTON_GPIO, GPIO_IN);
  gpio_pull_up(BUTTON_GPIO);

    //PicoPwm pwm0 = PicoPwm(PWM_PIN);
    //int slice_num = pwm0.getSlice();

  adc_init();
  adc_set_clkdiv(adcdiv);
  adc_gpio_init(ADC_PIN);
  adc_select_input(0);
    
  uint16_t data = adc_read();

  while(true){
    txbutton = gpio_get(BUTTON_GPIO);
    if (txbutton == false)   // TX
    {

      //read MIC  8x  (similar to main software)
      adc_result[2] = ((int16_t)adc_read() - ADC_BIAS);  //ADC MIC
      for(uint16_t i=1; i<10; i++)
      {
        adc_result[2] += ((int16_t)adc_read() - ADC_BIAS);  //ADC MIC
      }
      adc_result[2] >>= 3;  //MIC average
 
      gpio_put(LED_PIN, 1);
      dsp_tx_ssb();
      //dsp_tx_cw();
      //dsp_tx_am();
      //dsp_tx_fm();

    }else{

      TX_OFF();
      gpio_put(LED_PIN, 0);

    }



  }


}
