/*
 * File:        Lab 5
 * Author:      Caroline Chu, Ashley He, Mira Kim
 * 
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_2_2a.h"
// threading library
#include "pt_cornell_1_2_2a.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
////////////////////////////////////

// #define EnablePullDownA(bits) \
//   CNPUACLR = bits;            \
//   CNPDASET = bits;
// #define DisablePullDownA(bits) CNPDACLR = bits;
// #define EnablePullUpA(bits) \
//   CNPDACLR = bits;          \
//   CNPUASET = bits;
// #define DisablePullUpA(bits) CNPUACLR = bits;

// string buffer
char buffer[60];
char buffer1[60];

//DAC config
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define DEMO
//ISR stuff
// volatile int adc_val = 0;
// volatile int desired_angle = 505; //need to be set
// volatile int adc_angle = 255;
// volatile int motor_ctrl = 0;
// volatile int DAC_val = 0;
// volatile int i = 0;


// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_adc, pt_demo;
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

// system 1 second interval tick
int sys_time_seconds;
// ============== ISR =========================================================

// void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
// {
//   //ISR to update PID terms and write new motor control values
//   //Also outputs motor control and current angle to DAC
//   mT2ClearIntFlag();

//   // Read ADC
//   int i;
//   adc_val = 0;
//   for (i = 0; i < 5; i++)
//   {
//     adc_val = (ReadADC10(1) + adc_val) / 2; //265 - 800
//   }
//   adc_angle = adc_val;
//   // Write angle to DAC
//   mPORTBClearBits(BIT_4); // start transaction
//   // test for ready
//   while (TxBufFullSPI2())
//     ;
//   // write to spi2
//   WriteSPI2(DAC_config_chan_A | (adc_angle << 2));
//   while (SPI2STATbits.SPIBUSY)
//     ; // wait for end of transaction
//   // CS high
//   mPORTBSetBits(BIT_4); // end transaction
//   for (i = 1; i < 5; i++)
//   {
//     error[i] = error[i - 1];
//   }
//   //calculate current error
//   error[0] = desired_angle - adc_angle;

//   //control values
//   proportional_ctrl = Kp * error[0];
//   differential_ctrl = Kd * (error[0] - error[4]);
//   integral_ctrl += error[0];

//   if (((integral_ctrl > 0) && (error[0] < 0)) || ((integral_ctrl < 0) && (error[0] > 0)))
//     integral_ctrl = integral_ctrl - (integral_ctrl >> 3);
//             //(int)(0.9 * integral_ctrl);

//   motor_ctrl = proportional_ctrl + differential_ctrl + (integral_ctrl >> Ki);

//   if (motor_ctrl < 0)
//     motor_ctrl = 0;
//   if (motor_ctrl > 39999)
//     motor_ctrl = 39999;
//   SetDCOC3PWM(motor_ctrl); //motor_ctrl (pwm on time)

//   // Write motor value to DAC
//   //Low pass filter
//   DAC_val = DAC_val + ((motor_ctrl - DAC_val) >> 6); // 64 samples
//   mPORTBClearBits(BIT_4); // start transaction
//   // test for ready
//   while (TxBufFullSPI2())
//     ;
//   // write to spi2
//   WriteSPI2(DAC_config_chan_B | DAC_val); // /10
//   while (SPI2STATbits.SPIBUSY)
//     ; // wait for end of transaction
//   // CS high
//   mPORTBSetBits(BIT_4); // end transaction
// }

// ============== end ISR =========================================================

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD(protothread_timer(struct pt *pt))
{
  PT_BEGIN(pt);
  while (1)
  {
    // yield time 1 second
    PT_YIELD_TIME_msec(1000);
    sys_time_seconds++;

    // draw sys_time
    sprintf(buffer, "%d", sys_time_seconds);
    printLine2(1, buffer, ILI9340_YELLOW, ILI9340_BLACK);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // timer thread

volatile int i = 0;
volatile int j;
// === ADC Thread =============================================
//thread 1
static PT_THREAD(protothread_adc(struct pt *pt))
{
  PT_BEGIN(pt);
  while (1) {
      // rotate CW
    mPORTAClearBits(BIT_0);
    PT_YIELD_TIME_msec(1000);
    mPORTASetBits(BIT_1);
    PT_YIELD_TIME_msec(1000);
    mPORTBSetBits(BIT_0);
    PT_YIELD_TIME_msec(1000);
    mPORTBSetBits(BIT_1);
    PT_YIELD_TIME_msec(1000);


  }


//   } // END WHILE(1)
  PT_END(pt);
} // animation thread


// === Main  ======================================================
void main(void)
{
  //SYSTEMConfigPerformance(PBCLK);

  ANSELA = 0;
  ANSELB = 0;

  // timer interrupt //////////////////////////
  // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
  // 400 is 100 ksamples/sec at 30 MHz clock
  // 200 is 200 ksamples/sec
  // === Config timer and output compare to make PWM ========
  // set up timer2 to generate the wave period -- SET this to 1 mSec!
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 40000);
  // Need ISR to compute PID controller
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag
  // set up compare3 for PWM mode
  OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, motor_ctrl, motor_ctrl); //
  // OC3 is PPS group 4, map to RPB9 (pin 18)
  PPSOutput(4, RPB9, OC3);

  // the ADC ///////////////////////////////////////
  // configure and enable the ADC
  CloseADC10(); // ensure the ADC is off before setting the configuration

  // define setup parameters for OpenADC10
  // Turn module on | ouput in integer | trigger mode auto | enable autosample
  // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
  // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
  // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
#define PARAM1 ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

// define setup parameters for OpenADC10
// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
#define PARAM2 ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
      //
  // Define setup parameters for OpenADC10
  // use peripherial bus clock | set sample time | set ADC clock divider
  // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
  // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
#define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy

// define setup parameters for OpenADC10
// set AN11 and  as analog inputs
#define PARAM4 ENABLE_AN11_ANA | ENABLE_AN5_ANA //

// define setup parameters for OpenADC10
// DO not skip the channels you want to scan
// do not specify channels  5 and 11
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

  // use ground as neg ref for A
  // actual channel number is specified by the scan list
  SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);           //
  OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using the parameters defined above

  EnableADC10(); // Enable the ADC
  ///////////////////////////////////////////////////////

  /// SPI setup //////////////////////////////////////////
  // SCK2 is pin 26
  // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
//   PPSOutput(2, RPB5, SDO2);
  // control CS for DAC
//   mPORTBSetPinsDigitalOut(BIT_4);
//   mPORTBSetBits(BIT_4);
  // divide Fpb by 2, configure the I/O ports. Not using SS in this example
  // 16 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripherial, you will need to match these
  SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV, 2);

  // GPIO Setup ///////////////////////////////////////////
  // Set button inputs
  //mPORTASetPinsDigitalIn(BIT_4 | BIT_3);
  //EnablePullDownA(BIT_4);
  //EnablePullDownA(BIT_3);

  mPORTASetPinsDigitalOut(BIT_0);
  mPORTASetPinsDigitalOut(BIT_1);
  mPORTBSetPinsDigitalOut(BIT_0);
  mPORTBSetPinsDigitalOut(BIT_1);
  mPORTAClearBits(BIT_0);
  mPORTAClearBits(BIT_1);
  mPORTBClearBits(BIT_0);
  mPORTBClearBits(BIT_1);
  

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_adc);

  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);

  // round-robin scheduler for threads
  while (1)
  {
    PT_SCHEDULE(protothread_timer(&pt_timer));
    PT_SCHEDULE(protothread_adc(&pt_adc));
  }
} // main

// === end  ======================================================