/*
 * File:        lab5.c
 * Author:      Caroline Chu, Ashley He, Mira Kim
 * For use with Sean Carroll's Big Board
 * http://people.ece.cornell.edu/land/courses/ece4760/PIC32/target_board.html
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config_1_2_3.h"
// threading library
#include "pt_cornell_1_2_3.h"
// yup, the expander
#include "port_expander_brl4.h"

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// need for sin function
#include <math.h>
////////////////////////////////////

// lock out timer 2 interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)


////////////////////////////////////
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000
// DDS constant
#define two32 4294967296.0 // 2^32
#define Fs 100000

///////////////////////////////////////////
#define EnablePullUpB(bits) \
  CNPDBCLR = bits; \
  CNPDBSET = bits;

//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data;            // output value
volatile SpiChannel spiChn = SPI_CHANNEL2; // the SPI channel to use
volatile int spiClkDiv = 4;                // 10 MHz max speed for port expander!!
// the DDS units:
volatile unsigned int phase_accum_main, phase_incr_main = 400.0 * two32 / Fs; //
// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
  int junk;

  mT2ClearIntFlag();

  // main DDS phase and sine table lookup
  phase_accum_main += phase_incr_main;
  DAC_data = sin_table[phase_accum_main >> 24];

    // test for ready
     while (TxBufFullSPI2());
    // reset spi mode to avoid conflict with expander
    SPI_Mode16();
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction 
     // write to spi2
     WriteSPI2(DAC_config_chan_A | DAC_data);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // MUST read to clear buffer for port expander elsewhere in code
    junk = ReadSPI2(); 
   // CS high
   mPORTBSetBits(BIT_4); // end transaction
   //
}

// === print a line on TFT =====================================================
// print a line on the TFT
// string buffer
char buffer[60];
char buffer2[60];
char buffer3[60];
void printLine2(int line_number, char *print_buffer, short text_color, short back_color)
{

  int v_pos;
  v_pos = line_number * 20;
  // erase the pixels
  tft_fillRoundRect(0, v_pos, 239, 16, 1, back_color); // x,y,w,h,radius,color
  tft_setTextColor(text_color);
  tft_setCursor(0, v_pos);
  tft_setTextSize(2);
  tft_writeString(print_buffer);
}

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_color, pt_key, pt_serial;
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

// system 1 second interval tick
int sys_time_seconds;

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD(protothread_timer(struct pt *pt))
{
  PT_BEGIN(pt);
  // inside cs
   // port expander
  start_spi2_critical_section;
  initPE();
  // key 0
  mPortZSetPinsIn(BIT_0);
  mPortZEnablePullUp(BIT_0);
  // key 1
  mPortZSetPinsIn(BIT_1);
  mPortZEnablePullUp(BIT_1);
  // key 2
  mPortZSetPinsIn(BIT_2);
  mPortZEnablePullUp(BIT_2);
  // key 3
  mPortZSetPinsIn(BIT_3);
  mPortZEnablePullUp(BIT_3);
  // key 4
  mPortZSetPinsIn(BIT_4);
  mPortZEnablePullUp(BIT_4);
  // key 5
  mPortZSetPinsIn(BIT_5);
  mPortZEnablePullUp(BIT_5);
  // key 6
  mPortZSetPinsIn(BIT_6);
  mPortZEnablePullUp(BIT_6);

  mPortYSetPinsOut(BIT_0);
  mPortYSetPinsOut(BIT_1);
  mPortYSetPinsOut(BIT_2);
  mPortYSetPinsOut(BIT_3);
  clearBits(GPIOY,BIT_0);
  clearBits(GPIOY,BIT_1);
  clearBits(GPIOY,BIT_2);
  clearBits(GPIOY,BIT_3);
  

  end_spi2_critical_section;
  // without pullup, threshold is approx vdd/2, but with pullup
  // need external voltage divider
  // test input with a push button
  // another option is to put photoresistor through transistor - make it digital signal

  // turn off cs

  // FINAL PROJECT THINGS
  // mPORTASetPinsDigitalOut(BIT_2);
  // mPORTASetPinsDigitalOut(BIT_3);
  // mPORTBSetPinsDigitalOut(BIT_4);
  // mPORTBSetPinsDigitalOut(BIT_3);

  // mPORTAClearBits(BIT_2);
  // mPORTAClearBits(BIT_3);
  // mPORTBClearBits(BIT_4);
  // mPORTBClearBits(BIT_3);

  // setBits(GPIOZ, BIT_0);

  // testing pin
  // mPORTBSetPinsDigitalIn(BIT_7);
  // EnablePullUpB(BIT_7);


  while (1)
  {
    // yield time 1 second
    PT_YIELD_TIME_msec(1000);
    sys_time_seconds++;
    // draw sys_time
    sprintf(buffer, "Time=%d", sys_time_seconds);
    printLine2(0, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    // in cs
    start_spi2_critical_section;
    sprintf(buffer, "Input Z0=%d ",  readBits(GPIOZ, BIT_0));
    printLine2(5, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    sprintf(buffer, "Input Z1=%d",  readBits(GPIOZ, BIT_1)>>1);
    printLine2(6, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    sprintf(buffer, "Input Z2=%d ",  readBits(GPIOZ, BIT_2)>>2);
    printLine2(7, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    sprintf(buffer, "Input Z3=%d",  readBits(GPIOZ, BIT_3)>>3);
    printLine2(8, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    sprintf(buffer, "Input Z4=%d ",  readBits(GPIOZ, BIT_4)>>4);
    printLine2(9, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    sprintf(buffer, "Input Z5=%d",  readBits(GPIOZ, BIT_5)>>5);
    printLine2(10, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    sprintf(buffer, "Input Z6=%d ",  readBits(GPIOZ, BIT_6)>>6);
    printLine2(11, buffer, ILI9340_BLACK, ILI9340_YELLOW);

    // end cs
    end_spi2_critical_section;
    printLine2(5, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    printLine2(8, buffer2, ILI9340_BLACK, ILI9340_YELLOW);
    printLine2(11, buffer3, ILI9340_BLACK, ILI9340_YELLOW);
    // NEVER exit while
  } // END WHILE(1)
  
  PT_END(pt);
} // timer thread

volatile int i = 0;

//=== Serial terminal thread =================================================

static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  //   static char cmd[30];
  //   static float value;
  while (1)
  {

    //step 7 different times
    while (i < 5)
     {

    //   // rotate CW one step
        // mPORTASetBits(BIT_2);
        // PT_YIELD_TIME_msec(100);
        // mPORTAClearBits(BIT_2);
        // PT_YIELD_TIME_msec(100);
        // mPORTASetBits(BIT_3);
        // PT_YIELD_TIME_msec(100);
        // mPORTAClearBits(BIT_3);
        // PT_YIELD_TIME_msec(100);

        // mPORTBSetBits(BIT_4);
        // PT_YIELD_TIME_msec(100);
        // mPORTBClearBits(BIT_4);
        // PT_YIELD_TIME_msec(100);
        // mPORTBSetBits(BIT_3);
        // PT_YIELD_TIME_msec(100);
        // mPORTBClearBits(BIT_3);
        // PT_YIELD_TIME_msec(100);
        setBits(GPIOY, BIT_0);
        PT_YIELD_TIME_msec(100);
        clearBits(GPIOY, BIT_0);
        PT_YIELD_TIME_msec(100);
        sprintf(buffer, "step0");
        printLine2(12, buffer, ILI9340_BLACK, ILI9340_YELLOW);
        setBits(GPIOY, BIT_1);
        PT_YIELD_TIME_msec(100);
        clearBits(GPIOY, BIT_1);
        PT_YIELD_TIME_msec(100);
        sprintf(buffer, "step1");
        printLine2(13, buffer, ILI9340_BLACK, ILI9340_YELLOW);
        setBits(GPIOY, BIT_2);
        PT_YIELD_TIME_msec(100);
        clearBits(GPIOY, BIT_2);
        PT_YIELD_TIME_msec(100);
        sprintf(buffer, "step2");
        printLine2(14, buffer, ILI9340_BLACK, ILI9340_YELLOW);
        setBits(GPIOY, BIT_3);
        PT_YIELD_TIME_msec(100);
        clearBits(GPIOY, BIT_3);
        PT_YIELD_TIME_msec(100);
        sprintf(buffer, "step3");
        printLine2(14, buffer, ILI9340_BLACK, ILI9340_YELLOW);
        i++;
     }

    // while (i < 10)
    // {
    //   //rotate CCW one step
    //     // mPORTBSetBits(BIT_3);
    //     // PT_YIELD_TIME_msec(100);
    //     // mPORTBClearBits(BIT_3);
    //     // PT_YIELD_TIME_msec(100);
    //     // mPORTBSetBits(BIT_4);
    //     // PT_YIELD_TIME_msec(100);
    //     // mPORTBClearBits(BIT_4);
    //     // PT_YIELD_TIME_msec(100);

    //     // mPORTASetBits(BIT_3);
    //     // PT_YIELD_TIME_msec(100);
    //     // mPORTAClearBits(BIT_3);
    //     // PT_YIELD_TIME_msec(100);
    //     // mPORTASetBits(BIT_2);
    //     // PT_YIELD_TIME_msec(100);
    //     // mPORTAClearBits(BIT_2);
    //     // PT_YIELD_TIME_msec(100);
    //     setBits(GPIOY, BIT_3);
    //     PT_YIELD_TIME_msec(100);
    //     clearBits(GPIOY, BIT_3);
    //     PT_YIELD_TIME_msec(100);
    //     setBits(GPIOY, BIT_2);
    //     PT_YIELD_TIME_msec(100);
    //     clearBits(GPIOY, BIT_2);
    //     PT_YIELD_TIME_msec(100);
    //     setBits(GPIOY, BIT_1);
    //     PT_YIELD_TIME_msec(100);
    //     clearBits(GPIOY, BIT_1);
    //     PT_YIELD_TIME_msec(100);
    //     setBits(GPIOY, BIT_0);
    //     PT_YIELD_TIME_msec(100);
    //     clearBits(GPIOY, BIT_0);
    //     PT_YIELD_TIME_msec(100);
    //     i++;
    // }

    i = 0;
    // never exit while
  } // END WHILE(1)
  PT_END(pt);
} // thread 3

// === Main  ======================================================
void main(void)
{
  //SYSTEMConfigPerformance(PBCLK);

  ANSELA = 0;
  ANSELB = 0;

  // set up DAC on big board
  // timer interrupt //////////////////////////
  // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
  // at 30 MHz PB clock 60 counts is two microsec
  // 400 is 100 ksamples/sec
  // 2000 is 20 ksamp/sec
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 400);

  // set up the timer interrupt with a priority of 2
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag

  // control CS for DAC
  mPORTBSetPinsDigitalOut(BIT_4);
  mPORTBSetBits(BIT_4);
  // SCK2 is pin 26
  // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
  PPSOutput(2, RPB5, SDO2);
  // 16 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripherial, you will need to match these
  // NOTE!! IF you are using the port expander THEN
  // -- clk divider must be set to 4 for 10 MHz
  SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV, 4);
  // end DAC setup

  // === build the sine lookup table =======
  // scaled to produce values between 0 and 4096
  int ii;
  for (ii = 0; ii < sine_table_size; ii++)
  {
    sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
  }

 // moved pin setup to thread

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_serial);
  

  // init the display
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);

  // round-robin scheduler for threads
  while (1)
  {
    PT_SCHEDULE(protothread_timer(&pt_timer));
    PT_SCHEDULE(protothread_serial(&pt_serial));
  }
} // main

// === end  ======================================================
