/*
 * File:        lablab.c
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
#include <string.h>
////////////////////////////////////

// lock out timer 2 interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)

// string buffer
char buffer[60];

////////////////////////////////////
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000
// DDS constant
#define two32 4294967296.0 // 2^32
#define Fs 25000.0         // Nyquist: higher than 1477*16 (hiehest frequency)(at least 8 samples)

//== Timer 2 interrupt handler ===========================================
volatile SpiChannel spiChn = SPI_CHANNEL2; // the SPI channel to use
volatile int spiClkDiv = 2;                // 10 MHz max speed for port expander!!
// the DDS units:
#define sine_table_size 256
volatile int sin_table[sine_table_size];

// ==================================================================
// initialize output signals
static float Fout = 400.0;
static float Fout2 = 500.0;
static float Fout1 = 600.0;

//== Timer 2 interrupt handler ===========================================
// actual scaled DAC
volatile int DAC_data;
// the DDS units:
volatile unsigned int phase_accum_main, phase_accum_main2, phase_incr_main, phase_incr_main2;
volatile unsigned int phase_accum_test, phase_incr_test;
volatile int playingnote = 0;

// ==================================================================

////////////////////////////////////
// pullup/down macros for keypad
// PORT B
#define EnablePullDownB(bits) \
  CNPUBCLR = bits;            \
  CNPDBSET = bits;
#define DisablePullDownB(bits) CNPDBCLR = bits;
#define EnablePullUpB(bits) \
  CNPDBCLR = bits;          \
  CNPUBSET = bits;
#define DisablePullUpB(bits) CNPUBCLR = bits;
//PORT A
#define EnablePullDownA(bits) \
  CNPUACLR = bits;            \
  CNPDASET = bits;
#define DisablePullDownA(bits) CNPDACLR = bits;
#define EnablePullUpA(bits) \
  CNPDACLR = bits;          \
  CNPUASET = bits;
#define DisablePullUpA(bits) CNPUACLR = bits;
////////////////////////////////////


void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
  int junk;

  mT2ClearIntFlag();
  // main DDS phase and sine table lookup
  phase_accum_main += phase_incr_main;
  DAC_data = sin_table[phase_accum_main >> 24];

  // === Channel A =============
  // wait for possible port expander transactions to complete
  while (TxBufFullSPI2());
  // reset spi mode to avoid conflict with port expander
  SPI_Mode16();
  // CS low to start transaction
  mPORTBClearBits(BIT_4); // start transaction
  // write to spi2
  WriteSPI2(DAC_config_chan_A | ((DAC_data + 2048) & 0xfff));
  while (SPI2STATbits.SPIBUSY); // wait for end of transaction
                        // CS high
  mPORTBSetBits(BIT_4); // end transaction
  // need to read SPI channel to avoid confusing port expander
  junk = ReadSPI2();
  //
}

// === print a line on TFT =====================================================
// print a line on the TFT
// string buffer
char buffer[60];

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

  while (1)
  {
    // yield time 1 second
    PT_YIELD_TIME_msec(1000);
    sys_time_seconds++;
    // draw sys_time
    sprintf(buffer, "Time=%d", sys_time_seconds);
    printLine2(0, buffer, ILI9340_BLACK, ILI9340_YELLOW);
  } // END WHILE(1)
  PT_END(pt);
} // timer thread

// ============================== Read Photoresistor Thread =============================================

static PT_THREAD(protothread_key(struct pt *pt))
{
  PT_BEGIN(pt);

  static int playback;
  static int bit0;
  static int bit1;
  static int bit2;
  static int bit3;
  static int bit4;
  static int bit5;
  static int bit6;
  static int prevbit0 = 0;
  static int prevbit1 = 0;
  static int prevbit2 = 0;
  static int prevbit3 = 0;
  static int prevbit4 = 0;
  static int prevbit5 = 0;
  static int prevbit6 = 0;

  static int prev2bit0 = 0;
  static int prev2bit1 = 0;
  static int prev2bit2 = 0;
  static int prev2bit3 = 0;
  static int prev2bit4 = 0;
  static int prev2bit5 = 0;
  static int prev2bit6 = 0;

  static int prev3bit0 = 0;
  static int prev3bit1 = 0;
  static int prev3bit2 = 0;
  static int prev3bit3 = 0;
  static int prev3bit4 = 0;
  static int prev3bit5 = 0;
  static int prev3bit6 = 0;

  static int prev4bit0 = 0;
  static int prev4bit1 = 0;
  static int prev4bit2 = 0;
  static int prev4bit3 = 0;
  static int prev4bit4 = 0;
  static int prev4bit5 = 0;
  static int prev4bit6 = 0;

  static int prev5bit0 = 0;
  static int prev5bit1 = 0;
  static int prev5bit2 = 0;
  static int prev5bit3 = 0;
  static int prev5bit4 = 0;
  static int prev5bit5 = 0;
  static int prev5bit6 = 0;

  static int prev6bit0 = 0;
  static int prev6bit1 = 0;
  static int prev6bit2 = 0;
  static int prev6bit3 = 0;
  static int prev6bit4 = 0;
  static int prev6bit5 = 0;
  static int prev6bit6 = 0;

  static int prev7bit0 = 0;
  static int prev7bit1 = 0;
  static int prev7bit2 = 0;
  static int prev7bit3 = 0;
  static int prev7bit4 = 0;
  static int prev7bit5 = 0;
  static int prev7bit6 = 0;

  static int prev8bit0 = 0;
  static int prev8bit1 = 0;
  static int prev8bit2 = 0;
  static int prev8bit3 = 0;
  static int prev8bit4 = 0;
  static int prev8bit5 = 0;
  static int prev8bit6 = 0;

  static int prev9bit6 = 0;
  static int prev10bit6 = 0;

  static int readbit0 = 0;
  static int readbit1 = 0;
  static int readbit2 = 0;
  static int readbit3 = 0;
  static int readbit4 = 0;
  static int readbit5 = 0;
  static int readbit6 = 0;

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
  // without pullup, threshold is approx vdd/2, but with pullup
  // need external voltage divider

  end_spi2_critical_section;

  // ====== SET UP INPUTS AND PULL UPS ================================
  while (1)
  {

    // yield time
    PT_YIELD_TIME_msec(30);

    // turn on cs
    start_spi2_critical_section;
    readbit0 = readBits(GPIOZ, BIT_0);
    readbit1 = readBits(GPIOZ, BIT_1);
    readbit2 = readBits(GPIOZ, BIT_2);
    readbit3 = readBits(GPIOZ, BIT_3);
    readbit4 = readBits(GPIOZ, BIT_4);
    readbit5 = readBits(GPIOZ, BIT_5);
    readbit6 = readBits(GPIOZ, BIT_6);

    if (readbit0 || prevbit0 || prev2bit0 || prev3bit0 || prev4bit0 || prev5bit0 || prev6bit0 || prev7bit0 || prev8bit0)
      bit0 = 0;
    else
      bit0 = 1;
    if (readbit1 || prevbit1 || prev2bit1 || prev3bit1 || prev4bit1 || prev5bit1 || prev6bit1 || prev7bit1 || prev8bit1)
      bit1 = 0;
    else
      bit1 = 1;
    if (readbit2 || prevbit2 || prev2bit2 || prev3bit2 || prev4bit2 || prev5bit2 || prev6bit2 || prev7bit2 || prev8bit2)
      bit2 = 0;
    else
      bit2 = 1;
    if (readbit3 || prevbit3 || prev2bit3 || prev3bit3 || prev4bit3 || prev5bit3 || prev6bit3 || prev7bit3 || prev8bit3)
      bit3 = 0;
    else
      bit3 = 1;
    if (readbit4 || prevbit4 || prev2bit4 || prev3bit4 || prev4bit4 || prev5bit4 || prev6bit4 || prev7bit4 || prev8bit4)
      bit4 = 0;
    else
      bit4 = 1;
    if (readbit5 || prevbit5 || prev2bit5 || prev3bit5 || prev4bit5 || prev5bit5 || prev6bit5 || prev7bit5 || prev8bit5)
      bit5 = 0;
    else
      bit5 = 1;
    if (readbit6 || prevbit6 || prev2bit6 || prev3bit6 || prev4bit6 || prev5bit6 || prev6bit6 || prev7bit6 || prev8bit6 || prev9bit6 || prev10bit6)
      bit6 = 0;
    else
      bit6 = 1;

    prev10bit6 = prev9bit6;
    prev9bit6 = prev8bit6;

    prev8bit0 = prev7bit0;
    prev8bit1 = prev7bit1;
    prev8bit2 = prev7bit2;
    prev8bit3 = prev7bit3;
    prev8bit4 = prev7bit4;
    prev8bit5 = prev7bit5;
    prev8bit6 = prev7bit6;

    prev7bit0 = prev6bit0;
    prev7bit1 = prev6bit1;
    prev7bit2 = prev6bit2;
    prev7bit3 = prev6bit3;
    prev7bit4 = prev6bit4;
    prev7bit5 = prev6bit5;
    prev7bit6 = prev6bit6;

    prev6bit0 = prev5bit0;
    prev6bit1 = prev5bit1;
    prev6bit2 = prev5bit2;
    prev6bit3 = prev5bit3;
    prev6bit4 = prev5bit4;
    prev6bit5 = prev5bit5;
    prev6bit6 = prev5bit6;

    prev5bit0 = prev4bit0;
    prev5bit1 = prev4bit1;
    prev5bit2 = prev4bit2;
    prev5bit3 = prev4bit3;
    prev5bit4 = prev4bit4;
    prev5bit5 = prev4bit5;
    prev5bit6 = prev4bit6;

    prev4bit0 = prev3bit0;
    prev4bit1 = prev3bit1;
    prev4bit2 = prev3bit2;
    prev4bit3 = prev3bit3;
    prev4bit4 = prev3bit4;
    prev4bit5 = prev3bit5;
    prev4bit6 = prev3bit6;

    prev3bit0 = prev2bit0;
    prev3bit1 = prev2bit1;
    prev3bit2 = prev2bit2;
    prev3bit3 = prev2bit3;
    prev3bit4 = prev2bit4;
    prev3bit5 = prev2bit5;
    prev3bit6 = prev2bit6;

    prev2bit0 = prevbit0;
    prev2bit1 = prevbit1;
    prev2bit2 = prevbit2;
    prev2bit3 = prevbit3;
    prev2bit4 = prevbit4;
    prev2bit5 = prevbit5;
    prev2bit6 = prevbit6;

    prevbit0 = readbit0;
    prevbit1 = readbit1;
    prevbit2 = readbit2;
    prevbit3 = readbit3;
    prevbit4 = readbit4;
    prevbit5 = readbit5;
    prevbit6 = readbit6;

    end_spi2_critical_section;

    playingnote = bit0 + bit1 + bit2 + bit3 + bit4 + bit5 + bit6;

    //one note playing
    if (playingnote == 1)
    {
      if (bit0 == 1)
      { // F note
        Fout = 698;
        Fout2 = 0;
      }
      if (bit1 == 1)
      { // G note
        Fout = 784;
        Fout2 = 0;
      }
      if (bit2 == 1)
      { // A note
        Fout = 880;
        Fout2 = 0;
      }
      if (bit3 == 1)
      { //B
        Fout = 988;
        Fout2 = 0;
      }
      if (bit4 == 1)
      { //C
        Fout = 1047;
        Fout2 = 0;
      }
      if (bit5 == 1)
      { //D
        Fout = 1175;
        Fout2 = 0;
      }
      if (bit6 == 1)
      { //E
        Fout = 1319;
        Fout2 = 0;
      }
    }
    else if (playingnote == 2)
    {
      if (bit0 == 1)
      {
        Fout = 698;
        if (bit1 == 1)
          Fout2 = 784;
        if (bit2 == 1)
          Fout2 = 880;
        if (bit3 == 1)
          Fout2 = 988;
        if (bit4 == 1)
          Fout2 = 1047;
        if (bit5 == 1)
          Fout2 = 1175;
        if (bit6 == 1)
          Fout2 = 1319;
      }
      if (bit1 == 1)
      {
        Fout = 784;
        if (bit0 == 1)
          Fout2 = 698;
        if (bit2 == 1)
          Fout2 = 880;
        if (bit3 == 1)
          Fout2 = 988;
        if (bit4 == 1)
          Fout2 = 1047;
        if (bit5 == 1)
          Fout2 = 1175;
        if (bit6 == 1)
          Fout2 = 1319;
      }
      if (bit2 == 1)
      {
        Fout = 880;
        if (bit0 == 1)
          Fout2 = 698;
        if (bit1 == 1)
          Fout2 = 784;
        if (bit3 == 1)
          Fout2 = 988;
        if (bit4 == 1)
          Fout2 = 1047;
        if (bit5 == 1)
          Fout2 = 1175;
        if (bit6 == 1)
          Fout2 = 1319;
      }
      if (bit3 == 1)
      {
        Fout = 988;
        if (bit0 == 1)
          Fout2 = 698;
        if (bit1 == 1)
          Fout2 = 784;
        if (bit2 == 1)
          Fout2 = 880;
        if (bit4 == 1)
          Fout2 = 1047;
        if (bit5 == 1)
          Fout2 = 1175;
        if (bit6 == 1)
          Fout2 = 1319;
      }
      if (bit4 == 1)
      {
        Fout = 1047;
        if (bit0 == 1)
          Fout2 = 698;
        if (bit1 == 1)
          Fout2 = 784;
        if (bit2 == 1)
          Fout2 = 880;
        if (bit3 == 1)
          Fout2 = 988;
        if (bit5 == 1)
          Fout2 = 1175;
        if (bit6 == 1)
          Fout2 = 1319;
      }
      if (bit5 == 1)
      {
        Fout = 1175;
        if (bit0 == 1)
          Fout2 = 698;
        if (bit1 == 1)
          Fout2 = 784;
        if (bit2 == 1)
          Fout2 = 880;
        if (bit3 == 1)
          Fout2 = 988;
        if (bit4 == 1)
          Fout2 = 1047;
        if (bit6 == 1)
          Fout2 = 1319;
      }
      if (bit6 == 1)
      {
        Fout = 1319;
        if (bit0 == 1)
          Fout2 = 698;
        if (bit1 == 1)
          Fout2 = 784;
        if (bit2 == 1)
          Fout2 = 880;
        if (bit3 == 1)
          Fout2 = 988;
        if (bit4 == 1)
          Fout2 = 1047;
        if (bit5 == 1)
          Fout2 = 1175;
      }
    }
    else
    {
      Fout = 0;
      Fout2 = 0;
    }

    phase_incr_main = (int)((Fout) * (float)two32 / Fs);
    phase_incr_main2 = (int)((Fout2) * (float)two32 / Fs);
    //sprintf(buffer, "Fout=%f ", Fout);
    //printLine2(12, buffer, ILI9340_BLACK, ILI9340_YELLOW);
    //sprintf(buffer, "Fout2=%d ", Fout2);
    //printLine2(13, buffer, ILI9340_BLACK, ILI9340_YELLOW);

  } // END WHILE(1)

  PT_END(pt);
} // =============== END PHOTORESISTOR THREAD ============================

volatile int i = 0;

//============================================ Stepper Motor thread =================================================

static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);

  while (1)
  {

    // half step 7 different times
    // each bit set and reset is one half step
    while (i < 2)
    {
      // rotate CW one half step
      mPORTBSetBits(BIT_3);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_3);
      PT_YIELD_TIME_msec(20);
      // rotate CW one half step
      mPORTBSetBits(BIT_7);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_7);
      PT_YIELD_TIME_msec(20);
      // rotate CW one half step
      mPORTBSetBits(BIT_8);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_8);
      PT_YIELD_TIME_msec(20);
      // rotate CW one half step
      mPORTBSetBits(BIT_13);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_13);
      PT_YIELD_TIME_msec(20);
      i++;
    }

    // rotate CCW
    while (i < 4)
    {
      // rotate CCW one half step
      mPORTBSetBits(BIT_13);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_13);
      PT_YIELD_TIME_msec(20);
      // rotate CCW one half step
      mPORTBSetBits(BIT_8);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_8);
      PT_YIELD_TIME_msec(20);
      // rotate CCW one half step
      mPORTBSetBits(BIT_7);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_7);
      PT_YIELD_TIME_msec(20);
      // rotate CCW one half step
      mPORTBSetBits(BIT_3);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_3);
      PT_YIELD_TIME_msec(20);
      i++;
    }

    i = 0;
    // never exit while
  } // END WHILE(1)
  PT_END(pt);
} 
//============================================ END Stepper Motor thread =================================================

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
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 1600);

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

  // FINAL PROJECT THINGS
  mPORTBSetPinsDigitalOut(BIT_3);
  mPORTBSetPinsDigitalOut(BIT_7);
  mPORTBSetPinsDigitalOut(BIT_8);
  mPORTBSetPinsDigitalOut(BIT_13);

  mPORTBClearBits(BIT_3);
  mPORTBClearBits(BIT_7);
  mPORTBClearBits(BIT_8);
  mPORTBClearBits(BIT_13);

  // END FINAL PROJECT THINGS

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_serial);
  PT_INIT(&pt_key);

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
    PT_SCHEDULE(protothread_key(&pt_key));
  }
} // main

// === end  ======================================================