
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
 // lock out timer 2 interrupt during spi comm to port expander
// This is necessary if you use the SPI2 channel in an ISR.
// The ISR below runs the DAC using SPI2
#define start_spi2_critical_section INTEnable(INT_T2, 0)
#define end_spi2_critical_section INTEnable(INT_T2, 1)
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
 // ==================================================================
 volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock
 // === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_key;
 // DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];     // sin_table for normal mode
 //== Timer 2 interrupt handler ===========================================
// actual scaled DAC 
volatile  int DAC_data;
// the DDS units:
volatile unsigned int phase_accum_main, phase_accum_main2, phase_incr_main, phase_incr_main2;
volatile unsigned int phase_accum_test, phase_incr_test;
volatile  int playingnote = 0;
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
    int junk;
    mT2ClearIntFlag();
    
        // if the button is pressed, ramp up to 255
        //if (playingnote == 1 || playingnote == 2) {
            ramp = 255;
        //}
        // if button is released, ramp down to 0
        //else if (playingnote == 0) {
        //    ramp = 0;
        //}
        //else {ramp = 0;}
    // ================= END TESTMODE RAMP =================
     // main DDS phase  
        phase_accum_main += phase_incr_main;
        phase_accum_main2 += phase_incr_main2;
   
    
        DAC_data = ramp*(sin_table[phase_accum_main>>24] + sin_table[phase_accum_main2>>24]);
    // now the 90 degree data
    
    // === Channel A =============
        // wait for possible port expander transactions to complete
    while (TxBufFullSPI2());
    // reset spi mode to avoid conflict with port expander
    SPI_Mode16();
    //while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS low to start transaction
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // test for ready
    // write to spi2 
    WriteSPI2( DAC_config_chan_A | ((DAC_data>>8) + 2048));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
    mPORTBSetBits(BIT_4); // end transaction
    // need to read SPI channel to avoid confusing port expander
    //junk = ReadSPI2(); 
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
    static int bit0;
    static int bit1;
    static int bit2;
    static int bit3;
    static int bit4;
    static int bit5;
    static int bit6;
 
// ====== SET UP INPUTS AND PULL UPS ================================
      while(1) {
         // yield time
        PT_YIELD_TIME_msec(10);
        
        // ====== TOGGLE TESTMODE ================================
        
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
  		// test input with a push button
  		// another option is to put photoresistor through transistor - make it digital signal
   		// turn off cs
        if(readBits(GPIOZ, BIT_0) == 0) bit0 = 1;
        else bit0 = 0;
        if(readBits(GPIOZ, BIT_1) == 0) bit1 = 1;
        else bit1 = 0;
        if(readBits(GPIOZ, BIT_2) == 0) bit2 = 1;
        else bit2 = 0;
        if(readBits(GPIOZ, BIT_3) == 0) bit3 = 1;
        else bit3 = 0;
        if(readBits(GPIOZ, BIT_4) == 0) bit4 = 1;
        else bit4 = 0;
        if(readBits(GPIOZ, BIT_5) == 0) bit5 = 1;
        else bit5 = 0;
        if(readBits(GPIOZ, BIT_6) == 0) bit6 = 1;
        else bit6 = 0;
        /*sprintf(buffer, "Input Z0=%d ",  readBits(GPIOZ, BIT_0));
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
    	printLine2(11, buffer, ILI9340_BLACK, ILI9340_YELLOW);*/
         end_spi2_critical_section;
         playingnote = bit0 + bit1 + bit2 + bit3 + bit4 + bit5 + bit6;
         //one note playing
        if(playingnote == 1){
        	if(bit0 == 1){ // F note
        		Fout = 698;
        		Fout2 = 0;
        		//sprintf(buffer, "route");
    			//printLine2(14, buffer, ILI9340_BLACK, ILI9340_YELLOW);
        	}
        	if(bit1 == 1){ // G note
        		Fout = 784;
        		Fout2 = 0;
        	}
        	if(bit2 == 1){ // A note
        		Fout = 880;
        		Fout2 = 0;
        	}
        	if(bit3 == 1){//B
        		Fout = 988;
        		Fout2 = 0;
        	}
        	if(bit4 == 1){//C
        		Fout = 1047;
        		Fout2 = 0;
        	}
        	if(bit5 == 1){//D
        		Fout = 1175;
        		Fout2 = 0;
        	}
        	if(bit6 == 1){ //E
        		Fout = 1319;
        		Fout2 = 0;
        	}
        }
        else if(playingnote == 2){
        	if(bit0 == 1){
				Fout = 698;
        		if(bit1 == 1) Fout2 = 784;
        		if(bit2 == 1) Fout2 = 880;
        		if(bit3 == 1) Fout2 = 988;
        		if(bit4 == 1) Fout2 = 1047;
        		if(bit5 == 1) Fout2 = 1175;
        		if(bit6 == 1) Fout2 = 1319;
        	}
        	if(bit1 == 1){
				Fout = 784;
        		if(bit0 == 1) Fout2 = 698;
        		if(bit2 == 1) Fout2 = 880;
        		if(bit3 == 1) Fout2 = 988;
        		if(bit4 == 1) Fout2 = 1047;
        		if(bit5 == 1) Fout2 = 1175;
        		if(bit6 == 1) Fout2 = 1319;
        	}
        	if(bit2 == 1){
				Fout = 880;
        		if(bit0 == 1) Fout2 = 698;
        		if(bit1 == 1) Fout2 = 784;
        		if(bit3 == 1) Fout2 = 988;
        		if(bit4 == 1) Fout2 = 1047;
        		if(bit5 == 1) Fout2 = 1175;
        		if(bit6 == 1) Fout2 = 1319;
        	}
        	if(bit3 == 1){
				Fout = 988;
        		if(bit0 == 1) Fout2 = 698;
        		if(bit1 == 1) Fout2 = 784;
        		if(bit2 == 1) Fout2 = 880;
        		if(bit4 == 1) Fout2 = 1047;
        		if(bit5 == 1) Fout2 = 1175;
        		if(bit6 == 1) Fout2 = 1319;
        	}
        	if(bit4 == 1){
				Fout = 1047;
        		if(bit0 == 1) Fout2 = 698;
        		if(bit1 == 1) Fout2 = 784;
        		if(bit2 == 1) Fout2 = 880;
        		if(bit3 == 1) Fout2 = 988;
        		if(bit5 == 1) Fout2 = 1175;
        		if(bit6 == 1) Fout2 = 1319;
        	}
        	if(bit5 == 1){
				Fout = 1175;
        		if(bit0 == 1) Fout2 = 698;
        		if(bit1 == 1) Fout2 = 784;
        		if(bit2 == 1) Fout2 = 880;
        		if(bit3 == 1) Fout2 = 988;
        		if(bit4 == 1) Fout2 = 1047;
        		if(bit6 == 1) Fout2 = 1319;
        	}
        	if(bit6 == 1){
				Fout = 1319;
        		if(bit0 == 1) Fout2 = 698;
        		if(bit1 == 1) Fout2 = 784;
        		if(bit2 == 1) Fout2 = 880;
        		if(bit3 == 1) Fout2 = 988;
        		if(bit4 == 1) Fout2 = 1047;
        		if(bit5 == 1) Fout2 = 1175;
        	}
        }else{
        	Fout = 0;
        	Fout2 = 0;
        }
        phase_incr_main = (int)((Fout)*(float)two32/Fs);
        phase_incr_main2 = (int)((Fout2)*(float)two32/Fs);
        //sprintf(buffer, "Fout=%f ", Fout);
    	//printLine2(12, buffer, ILI9340_BLACK, ILI9340_YELLOW); 
    	//sprintf(buffer, "Fout2=%d ", Fout2);
    	//printLine2(13, buffer, ILI9340_BLACK, ILI9340_YELLOW);                            
        
      } // END WHILE(1)
   PT_END(pt);
} // =============== END KEYPAD THREAD ============================
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
   // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_key(&pt_key));
      }
  } // main
 // === end  ======================================================