/*
Copyright (c) 2022 Konomu Abe
 
Inspired by Adafruit example
   https://github.com/Seeed-Studio/ArduinoCore-samd/tree/master/libraries/Adafruit_ZeroDMA/examples/zerodma_spi2/zerodma_spi2.ino
  https://github.com/Seeed-Studio/ArduinoCore-samd/blob/master/libraries/Adafruit_ZeroDMA/LICENSE
*/

/*******************************
The MIT License (MIT)

Copyright (c) 2016 Adafruit Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <TimerTC3.h>
#include "TimerTC4.h"
#include <Adafruit_ZeroDMA.h>
#include <math.h>

#define SAMPLE_FREQ (100000)

#define uq32 uint32_t // PI/2 = 2^32, unsigned
#define THETA_LSB (1.0/4294967296.0) /* 1.0[Right angle](= 90 degree) */

uq32 delta_theta;
float signal_freq = 10.0;

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions
DmacDescriptor *desc; // DMA descriptor address (so we can change contents)

bool isLEDOn = false;

uq32 theta = 0, prev_theta = 0;

#define DATA_LENGTH 1024
uint16_t source_memory[2][DATA_LENGTH];
uint8_t buffer_being_filled = 0, // Index of 'filling' buffer
        buffer_to_write = 1 ;

int buf_idx = 0;

uint16_t sin_table[512];
int sin_phase = 0;

// Callback for end-of-DMA-transfer
void dma_callback(Adafruit_ZeroDMA *dma) {
  buffer_being_filled = 1 - buffer_being_filled;
  myDMA.changeDescriptor(desc,           // DMA descriptor address
    source_memory[buffer_being_filled]); // New src; dst & count don't change
  stat = myDMA.startJob();
}

int16_t fix_sin(uq32 theta,int phase) /* theta[Right Angle] */
{
  unsigned int idx = (theta>>23);
  if(phase%2 == 1){
    idx = 511-idx;
  }
  return((int16_t)(512 +(phase>1?-1:1)*sin_table[idx]));
}

void setup() 
{
    Serial.begin(115200);
    Serial.println("Hello.\n");

    pinMode(LED_BUILTIN, OUTPUT);    
    pinMode(PIN_A1, OUTPUT);    

    TimerTc3.initialize(100000);
    TimerTc3.attachInterrupt(timerIsr);
    TimerTc4.initialize(10);

// packages\Seeeduino\tools\CMSIS-Atmel\1.2.1\CMSIS-Atmel\CMSIS\Device\ATMEL\samd21\include\instance\tc4.h
    myDMA.setTrigger(TC4_DMAC_ID_OVF);
    myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);
  
    Serial.println("Allocating DMA channel...");
    stat = myDMA.allocate();
    myDMA.printStatus(stat);

    desc = myDMA.addDescriptor(
      source_memory[buffer_being_filled], // move data from here
      (void *)(&DAC->DATABUF.reg),   // to here
      DATA_LENGTH,                        // this many...
      DMA_BEAT_SIZE_HWORD,                 // bytes/hword/words
      true,                               // increment source addr?
      false);                             // increment dest addr?

    for(int i=0;i< 512;i++){
      sin_table[i] = 511*sin(HALF_PI*i/512);
    }

    for(int i=0;i< DATA_LENGTH;i++){
      source_memory[0][i] = 512;
      source_memory[1][i] = 512;
    }

    delta_theta = (uq32)(4.0*signal_freq/SAMPLE_FREQ/THETA_LSB+0.5);

    Serial.println("Adding callback");
    // register_callback() can optionally take a second argument
    // (callback type), default is DMA_CALLBACK_TRANSFER_DONE
    myDMA.setCallback(dma_callback);
    stat = myDMA.startJob(); 
    myDMA.printStatus(stat);
    analogWrite(PIN_DAC0,source_memory[0][0]);
    TimerTc4.start();
}

bool main_loop_flag = false;
uint32_t main_loop_count=0;
void loop()
{
  if(main_loop_flag){
    main_loop_flag = false;
//    Serial.println("Hello.\n");
    if(main_loop_count%10==0){
      digitalWrite(LED_BUILTIN, isLEDOn);
      isLEDOn = !isLEDOn;
//      main_loop_count = 0
    }
    digitalWrite(PIN_A1,1); // 60us
    signal_freq *= 1.02;
    if(signal_freq>20000)
      signal_freq = 10.0;
    delta_theta = (uq32)(4.0*signal_freq/SAMPLE_FREQ/THETA_LSB+0.5);
    digitalWrite(PIN_A1,0);
    main_loop_count++;
  }
  if(buf_idx<DATA_LENGTH){
//    digitalWrite(PIN_A1,1);
//    source_memory[buffer_to_write][buf_idx] = 511*sin(theta)+512; // sin() takes 100 to 150us , using table reduces it to 50us.
    source_memory[buffer_to_write][buf_idx] = fix_sin(theta,sin_phase); // Fixpointer and table takes 3.5us
    buf_idx+=1;
    prev_theta = theta;
    theta += delta_theta;
    if(prev_theta > theta){
      sin_phase = (sin_phase+1)%4;
    }
//    digitalWrite(PIN_A1,0);
  }else{
    if(buffer_being_filled == buffer_to_write){
      buffer_to_write = 1 - buffer_to_write;
      buf_idx = 0;
    }
  }
}

void timerIsr()
{   
  main_loop_flag = true;
}

void tc4_Isr() //This should never be called.
{   
//    analogWrite(PIN_DAC0,dac_val);
//    DAC->DATA.reg = dac_val & 0x3FF;
//    REG_DAC_DATA = dac_val&0x3FF;
//    REG_DAC_DATABUF = dac_val&0x3FF;
//    dac_val = (dac_val+1)%1024;    
}
