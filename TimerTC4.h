/*
 This file is modification of
  https://github.com/Seeed-Studio/ArduinoCore-samd/tree/master/libraries/TimerTC3/TimerTC3.h
 by Konomu Abe
*/
/////////////////////////////////////////////////////////////////////////////

#ifndef _TIMER_TC4_H_
#define _TIMER_TC4_H_


#define CPU_HZ 48000000

#define RESOLUTION 0xffff    // Timer TC4 is 16 bit


class TimerTC4
{
  public:

    void initialize(long microseconds = 1000000);
    
    void setPeriod(long microseconds);
    
    void start();
    void stop();
    void restart();

    void attachInterrupt(void (*isr)());
    void detachInterrupt();

    void (*isrCallback)();
};


extern TimerTC4 TimerTc4;


#endif
