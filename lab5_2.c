/*
 * File:        DTMF Dialer
 * Author:      Caroline Chu, Mira Kim, Ashley He
 * For use with Sean Carroll's Big Board
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define Fs 25000.0 // Nyquist: higher than 1477*16 (hiehest frequency)(at least 8 samples)
#define two32 4294967296.0 // 2^32 
#include "config_1_2_3.h"
#include "pt_cornell_1_2_3.h"
#include "port_expander_brl4.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ==================================================================
// array to track the buttons pressed during normal mode
volatile int digits[12];
// counts the number of keys hit during recording mode
volatile int count = 0; 
// initialize output signals
static float Fout = 400.0;
static float Fout2= 500.0;
static float Fout1= 600.0;
volatile int ramp = 0;
// status helps track the different states during playback and recording
volatile int status = 0;
// tracks whether status has been set to 4 (ramp up during playback cycle)
volatile int check = 0;
// index for playback array
volatile int count2 = 0;
// track if button is released or not in test mode
volatile int released = 1;
// check whether the current mode is test or not. Normal mode: 0, Test mode: 1
volatile int testmode = 0;

volatile int i = 0;

// state machine
enum states {
    release,
    maybePushed,
    stillPushed,
    maybeReleased
};

// initialize the current state to be an unpressed key on the keypad
enum states PushState = release;
// ==================================================================

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_key, pt_motor;

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];     // sin_table for normal mode
volatile int sin_table2[sine_table_size];    // sin_table for test mode

//== Timer 2 interrupt handler ===========================================
// actual scaled DAC 
volatile  int DAC_data;
// the DDS units:
volatile unsigned int phase_accum_main, phase_accum_main2, phase_incr_main, phase_incr_main2;
volatile unsigned int phase_accum_test, phase_incr_test;

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include "stdlib.h"
////////////////////////////////////

////////////////////////////////////
// pullup/down macros for keypad
// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
////////////////////////////////////

////////////////////////////////////
// some precise, fixed, short delays
// to use for extending pulse durations on the keypad
// if behavior is erratic
#define NOP asm("nop");
// 1/2 microsec
#define wait20 NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// one microsec
#define wait40 wait20;wait20;
////////////////////////////////////

// string buffer
char buffer[60];

////////////////////////////////////
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000


// system 1 second interval tick
int sys_time_seconds ;


// =================================================================================
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // 74 cycles to get to this point from timer event
    mT2ClearIntFlag();
    
    // ramp up before playback
    if(!testmode) {
        if (status == 1) {
            if (ramp < 255) {
                ramp += 2; // this ensures ramp up in 3ms
            }
            if (ramp >= 255){
            status = 2;
            }
        }
        
        // ramp up during playback
        if (status == 4) {
            if (ramp < 255) {
                ramp += 2;
            }
            if (ramp >= 255){
            status = 5; // hold for 65ms
            }
        }
        
        // ramp down before playback
        if (status == 3){
            if (ramp > 0){
                ramp -= 2;
            }
            if (ramp <= 1){
                if(ramp==0)
                    status = 0;
                if(ramp==1)
                    ramp--;
            }
        }

        // ramp down during playback
        if (status == 6){
            if (ramp > 0){
                ramp -= 2;
            }
            if(ramp <= 1){
                if(ramp==0)
                    status = 7;
                if(ramp==1)
                ramp--;
            }
        }
    }
    // ================= TESTMODE RAMP ====================
    else if (testmode) {
        // if the button is pressed, ramp up to 255
        if (released == 0) {
            ramp = 255;
        }
        // if button is released, ramp down to 0
        else if (released == 1) {
            ramp = 0;
        }
    }
    // ================= END TESTMODE RAMP =================

    // main DDS phase
        phase_accum_test += phase_incr_test;    
        phase_accum_main += phase_incr_main;
        phase_accum_main2 += phase_incr_main2;
   
    if (testmode) {
        // if in test mode, play single frequency
        DAC_data = ramp*(sin_table[phase_accum_test>>24]);
    }
    else {
        // in normal mode, sum two sine waves
        DAC_data = ramp*(sin_table[phase_accum_main>>24] + sin_table[phase_accum_main2>>24])  ;
    }
    // now the 90 degree data
    
    // === Channel A =============
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // test for ready
    // write to spi2 
    WriteSPI2( DAC_config_chan_A | ((DAC_data>>8) + 2048));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
    mPORTBSetBits(BIT_4); // end transaction

} // =================== end ISR TIMER2 ==================================================


// ======================== Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
     tft_writeString("Time in seconds since boot\n");
     // set up LED to blink
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000);
        sys_time_seconds++ ;
        // draw sys_time
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", sys_time_seconds);
        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread


// ============================== Keypad Thread =============================================
// connections:
// A0 -- row 1 -- thru 300 ohm resistor -- avoid short when two buttons pushed
// A1 -- row 2 -- thru 300 ohm resistor
// A2 -- row 3 -- thru 300 ohm resistor
// A3 -- row 4 -- thru 300 ohm resistor
// B7 -- col 1 -- internal pulldown resistor -- avoid open circuit input when no button pushed
// B8 -- col 2 -- internal pulldown resistor
// B9 -- col 3 -- internal pulldown resistor

static PT_THREAD (protothread_key(struct pt *pt))
{
    PT_BEGIN(pt);
    // keypad > 0 if button pressed, i = keypad number pressed
    static int keypad, i, pattern;
    // to be used to update the x position on the TFT (testing purposes only)
    static int increment = 0;
    // keycode indicates the number on the keypad pressed
    // possible indicates if there was a key possibly pressed
    static int keycode, possible;
    static int playback;

    // order is 0 thru 9 then * ==10 and # ==11
    // no press = -1
    // table is decoded to natural digit order (except for * and #)
    // 0x80 for col 1 ; 0x100 for col 2 ; 0x200 for col 3
    // 0x01 for row 1 ; 0x02 for row 2; etc
    static int keytable[12]={0x108, 0x81, 0x101, 0x201, 0x82, 0x102, 0x202, 0x84, 0x104, 0x204, 0x88, 0x208};
    // init the keypad pins A0-A3 and B7-B9
    // PortA ports as digital outputs
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3);    //Set port as output
    mPORTASetPinsDigitalIn(BIT_4);
    // PortB as inputs
    mPORTBSetPinsDigitalIn(BIT_7 | BIT_8 | BIT_9);    //Set port as input
    // and turn on pull-down on inputs
    EnablePullDownB( BIT_7 | BIT_8 | BIT_9 );
    EnablePullDownA( BIT_4 );

      while(1) {

        // yield time
        PT_YIELD_TIME_msec(30);
        
        // ====== TOGGLE TESTMODE ================================
        if(mPORTAReadBits(BIT_4)){
            testmode = 1;
        }
        else {
            testmode = 0;
        }
        // ====== TOGGLE TESTMODE ================================
        
        if(status == 2){
            // change how long the tone is played
            PT_YIELD_TIME_msec(200);
            status = 3;
        }
        
        // read each row sequentially
        mPORTAClearBits(BIT_0 | BIT_1 | BIT_2 | BIT_3);
        pattern = 1; mPORTASetBits(pattern);
   
        for (i = 0; i < 4; i++) {
            wait40 ;
            keypad  = mPORTBReadBits(BIT_7 | BIT_8 | BIT_9);
            if(keypad!=0) {keypad |= pattern ; break;}
            mPORTAClearBits(pattern);
            pattern <<= 1;
            mPORTASetBits(pattern);
        }

        // search for keycode
        if (keypad > 0){ // then button is pushed
            for (i = 0; i < 12; i++){
                if (keytable[i] == keypad) break;
            }
            // if invalid, two button push, set to -1
            if (i == 12) i = -1;
        }
        else i = -1; // no button pushed

        keycode = i;

        switch (PushState) {
            case release:
                released = 1;
                if (keycode == -1) PushState = release;
                else {
                    PushState = maybePushed;
                    possible = keycode;
                }
                break;

            case maybePushed:
                if (keycode == possible) {
                    PushState = stillPushed;
                    released = 0;
                    // if the pound key is pressed, enter playback mode
                    if (i == 11 && !testmode) {
                        // while the index during playback isn't the same as number of keys pressed
                        // or if the frequency isn't currently ramped up
                        while ((count2 != count) || (check != 0)) {
                            if (check == 0) {
                                // ramp up 
                                status = 4;
                                // determine key # pressed
                                playback = digits[count2];
                                // currently ramped up
                                check = 1;
                                // move to next key in playback array
                                count2++;
                            }
                            if (status == 5) {
                                // if in playback mode and ramped up, wait 65ms
                                // before ramping back down
                                PT_YIELD_TIME_msec(65);
                                // move to ramp down phase
                                status = 6;
                            }

                            if (status == 7) {
                                // if in playback mode and ramped down, wait 65ms
                                PT_YIELD_TIME_msec(65);
                                // move to just before ramp up phase (status 4 not set yet)
                                check = 0;                                
                            }

                            // frequencies to be used during playback, depending on key press
                            if (playback == 1) {
                                Fout = 697;
                                Fout2 = 1209;
                            }

                            if (playback == 2) {
                                Fout = 697;
                                Fout2 = 1336;
                            }

                            if (playback == 3) {
                                Fout = 697;
                                Fout2 = 1477;
                            }

                            if (playback == 4) {
                                Fout = 770;
                                Fout2 = 1209;
                            }

                            if (playback == 5) {
                                Fout = 770;
                                Fout2 = 1336;
                            }

                            if (playback == 6) {
                                Fout = 770;
                                Fout2 = 1477;
                            }

                            if (playback == 7) {
                                Fout = 852;
                                Fout2 = 1209;
                            }

                            if (playback == 8) {
                                Fout = 852;
                                Fout2 = 1336;
                            }

                            if (playback == 9) {
                                Fout = 852;
                                Fout2 = 1477;
                            }

                            if (playback == 0) {
                                Fout = 941;
                                Fout2 = 1336;
                            }

                            if (playback > -1) {
                                phase_incr_main = (int)((Fout)*(float)two32/Fs);
                                phase_incr_main2 = (int)((Fout2)*(float)two32/Fs);
                            }
                            
                        }  
                        // playback has finished, reset for next playback
                        count2 = 0;
                    }
                    // if the number of keypresses is less than 12, record into playback array
                    // also play the 2 frequencies as the keys are pressed
                    else if (count < 12) {

                        if (i == 1) {
                            Fout = 697;
                            Fout2 = 1209;
                        }

                        if (i == 2) {
                            Fout = 697;
                            Fout2 = 1336;
                        }

                        if (i == 3) {
                            Fout = 697;
                            Fout2 = 1477;
                        }

                        if (i == 4) {
                            Fout = 770;
                            Fout2 = 1209;
                        }

                        if (i == 5) {
                            Fout = 770;
                            Fout2 = 1336;
                        }

                        if (i == 6) {
                            Fout = 770;
                            Fout2 = 1477;
                        }

                        if (i == 7) {
                            Fout = 852;
                            Fout2 = 1209;
                        }

                        if (i == 8) {
                            Fout = 852;
                            Fout2 = 1336;
                        }

                        if (i == 9) {
                            Fout = 852;
                            Fout2 = 1477;
                        }

                        if (i == 0) {
                            Fout = 941;
                            Fout2 = 1336;
                        }

                        // ====================== TESTMODE =====================================================
                        if (testmode) {
                            // set keys to single frequencies for test mode
                            if (i == 1) {
                                Fout1 = 697;
                            }

                            if (i == 2) {
                                Fout1 = 770;
                            }

                            if (i == 3) {
                                Fout1 = 852;
                            }

                            if (i == 4) {
                                Fout1 = 941;
                            }

                            if (i == 5) {
                                Fout1 = 1209;
                            }

                            if (i == 6) {
                                Fout1 = 1336;
                            }

                            if (i == 7) {
                                Fout1 = 1477;
                            }
                            phase_incr_test = (int)((Fout1)*(float)two32/Fs); 
                        }

                        // =============== END TESTMODE ========================================================

                        if (i > -1) {
                            phase_incr_main = (int)((Fout)*(float)two32/Fs);
                            phase_incr_main2 = (int)((Fout2)*(float)two32/Fs);                            
                        }

                        // if * is pressed, clear the numbers 
                        if (i == 10) {
                            count = 0;
                            status = 0;
                            tft_setCursor(10, 220);
                            tft_fillRoundRect(10,220, 200, 14, 1, ILI9340_BLACK); // x,y,w,h,radius,color
                            // to reset position on TFT (testing purposes only)
                            increment = 0;
                        }

                        else {
                            tft_setCursor(10+increment, 220);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            sprintf(buffer, "%d", i);
                            if (i == 11) {
                                sprintf(buffer,"#");
                            }
                            tft_writeString(buffer);
                            status = 1;
                            digits[count] = i;
                            // to set position on TFT (testing purposes only)
                            increment += 15;
                            count++;
                        }
                    }

                    else if (i == 10){
                        count = 0;
                        status = 0;
                        tft_setCursor(10, 220);
                        tft_fillRoundRect(10,220, 200, 14, 1, ILI9340_BLACK); // x,y,w,h,radius,color
                        // to reset position on TFT (testing purposes only)
                        increment = 0;
                    }
                }
                else PushState = release;
                break;

            case stillPushed:
                if (keycode == possible) {
                    PushState = stillPushed;
                    released = 0;
                }
                else PushState = maybeReleased;
                break;
            
            case maybeReleased:
                released = 1;
                if (keycode == possible) PushState = stillPushed;
                else PushState = release;
                break;

        }
      } // END WHILE(1)

  PT_END(pt);
} // =============== END KEYPAD THREAD ============================

//================= Motor Thread ============================
static PT_THREAD(protothread_motor(struct pt *pt))
{
  PT_BEGIN(pt);
  //   static char cmd[30];
  //   static float value;  


    //step 7 different times
    while (i < 5)
    {

      // rotate CW one step
      mPORTBSetBits(BIT_0);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_0);
      PT_YIELD_TIME_msec(10);
      mPORTBSetBits(BIT_1);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_1);
      PT_YIELD_TIME_msec(10);

      mPORTBSetBits(BIT_2);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_2);
      PT_YIELD_TIME_msec(10);
      mPORTBSetBits(BIT_3);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_3);
      PT_YIELD_TIME_msec(10);
      i++;
    }

    while (i < 10)
    {
      //rotate CCW one step
      mPORTBSetBits(BIT_3);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_3);
      PT_YIELD_TIME_msec(10);
      mPORTBSetBits(BIT_2);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_2);
      PT_YIELD_TIME_msec(10);

      mPORTBSetBits(BIT_1);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_1);
      PT_YIELD_TIME_msec(10);
      mPORTBSetBits(BIT_0);
      PT_YIELD_TIME_msec(10);
      mPORTBClearBits(BIT_0);
      PT_YIELD_TIME_msec(10);
      i++;
    }

    i = 0;
    // never exit while
   // END WHILE(1)
  PT_END(pt);
} // thread 3

// === Main  ======================================================
void main(void) {
  
  ANSELA = 0; ANSELB = 0; 

    // timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // at 30 MHz PB clock 60 counts is two microsec
    // 400 is 100 ksamples/sec
    // 2000 is 20 ksamp/sec
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 1600);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);

    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);

  mPORTBSetPinsDigitalOut(BIT_0);
  mPORTBSetPinsDigitalOut(BIT_1);
  mPORTBSetPinsDigitalOut(BIT_2);
  mPORTBSetPinsDigitalOut(BIT_3);

  mPORTBClearBits(BIT_0);
  mPORTBClearBits(BIT_1);
  mPORTBClearBits(BIT_2);
  mPORTBClearBits(BIT_3);


    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // clk divider set to 2 for 20 MHz
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 2);
    // end DAC setup
        
    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

    // init the threads
    PT_INIT(&pt_timer);
    PT_INIT(&pt_key);
    PT_INIT(&pt_motor);

    // init the display
    // NOTE that this init assumes SPI channel 1 connections
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240

    // seed random color
    srand(1);

   // build the sine lookup table
   // scaled to produce values between 0 and 4096
   int i;
   for (i = 0; i < sine_table_size; i++){
        sin_table[i] = (int)(1023*sin((float)i*6.283/(float)sine_table_size));
    }

    int j;
    for (j = 0; j < sine_table_size; j++){
        sin_table2[j] = (int)(2047*sin((float)j*6.283/(float)sine_table_size));
    }
  
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      //PT_SCHEDULE(protothread_key(&pt_key));
      PT_SCHEDULE(protothread_key(&pt_motor));
      }
    
  } // main

// === end  ======================================================

