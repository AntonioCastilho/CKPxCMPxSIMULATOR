/*
 * File:   main.c
 * Author: Automotive Technologist Antonio Castilho
 * 
 * It is a system to simulate a crankshaft rotation and position sensor [CKP], 
 * used in internal combustion engines and the camshaft position sensor [CMP].
 * It also has a lambda probe simulator.
 * It will be used in the development of a simulator for repairs of automotive
 * electronic control units.
 * 
 * Program environment: MPLAB X IDE v6.20, XC8 v2.50, C std C90, PIC18F455
 * Reference: Microchip PIC18F4550 Datasheet.
 * 
 * MIT License  (see: LICENSE em github)
 * Copyright (c) 2022 Antonio Aparecido Ariza Castilho
 * <https://github.com/AntonioCastilho>
 *
 * Created on 4 de Dezembro de 2024, 21:43
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <xc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"

/*******************************************************************************
 * Global variables 
 ******************************************************************************/
uint8_t interrupt_timer0 = 0, // Timebase auxiliary control.
        rpm_ckp = 0,          // Control of the pulse that the phonic wheel would generate.
        rpm_cmp = 0,          // CMP sensor position control.
        turn_ctrl = 0,        // Engine cycle control.
        count = 0;            // Control a sampling rate index for sinusoid simulation based on PWM2 duty cycle.

// senoid period
uint16_t sinewave[] = // 41 samples
    {
     0,22,49,86,132,185,246,311,380,415,450,
     415,380,311,246,185,132,86,49,22,0,22,
     49,86,132,185,246,311,380,415,450,
     415,380,311,246,185,132,86,49,22,0 
    };

/******************************************************************************/
void __interrupt() ISR()    // Interrupt function
/******************************************************************************/
{
    /***********************************************************
     * TIMER 0 Interrupt | 1 ms
     * Timebase
     ***********************************************************/
    if(INTCONbits.TMR0IF)
    { 
        INTCONbits.TMR0IF = 0;           // Clear the timer0 interrupt flag.
        timer0_write(0xFE0C);             // Reset Timer0.
        LATBbits.LATB7 = ~LATBbits.LATB7; // For function check and measurement purposes only.
        
    } // end of TIMER0 Interrupt.
    
    /**********************************************************
     * TIMER 1 Interrupt | 50ms
     * Simulation of a lambda probe signal 
     **********************************************************/
    if(PIR1bits.TMR1IF)
    {  
        // Interrupção a cada 50 ms
        // em 2 segundos -> 40 atualizações
        
        TMR1IF = 0;                     // TIMER 1 Interrupt  reset
        timer1_write(0xCF2C);
        
        LATBbits.LATB5 = ~PORTBbits.RB5;
        pwm2_setDutyPot((uint16_t)(((float)sinewave[count])*1.15));
        
        count++;
        if(count > 41) count = 0;
        
    }// end of TIMER1 Interrupt 
    
    /*********************************************************
     * TIMER 2 Interrupt
     * Synchronized CKP (60-2) and CMP (VW) signal simulation.
     *********************************************************/
    if(PIR1bits.TMR2IF)           //PWM match PR2 and TMR2.
    {
        LATBbits.LATB6 = ~PORTBbits.RB6;
        rpm_ckp++;                // Increment for a new pulse of the phonic wheel.
        PIR1bits.TMR2IF = 0;      // Clear the timer2 interrupt flag.
        
        if(rpm_ckp == 59)         // The minus two teeth gap has reached.  // One turn completed on the next pulse.
        {                        
            LATBbits.LATB0 = 0;   // Causes the CCP/PWM to shutdown.
            turn_ctrl++;          // Increases lap control.
            LATCbits.LATC2 = 0;   // Ensures PWM output remains LOW during minus 2 tooth failure.
        } // end of if "rpm_ckp == 59".
        
        if(rpm_ckp == 60)         // lap completed.
        {
            rpm_ckp = 0;          // Reset phonic wheel control variable.
            LATBbits.LATB0 = 1;   // Restarts PWM.
            
            if(turn_ctrl == 2)    // 2 complete turns, 1 complete engine cycle
            {
                turn_ctrl = 0;    // Reset lap control.
            } // end of if "turn_ctrl == 2".
        } // end of if "rpm_ckp == 60".
        
        // Synchronization for phase sensor (CMP) 
        // typical for VW line engines with 60-2 phonic wheel.
        if(rpm_cmp == 21)  LATBbits.LATB4 = 0; 
        if(rpm_cmp == 45)  LATBbits.LATB4 = 1;
        if(rpm_cmp == 51)  LATBbits.LATB4 = 0;
        if(rpm_cmp == 74)  LATBbits.LATB4 = 1;
        if(rpm_cmp == 81)  LATBbits.LATB4 = 0;
        if(rpm_cmp == 88)  LATBbits.LATB4 = 1;
        if(rpm_cmp == 111) LATBbits.LATB4 = 0;
        if(rpm_cmp == 118) LATBbits.LATB4 = 1;
        rpm_cmp++; // Increases CMP control.
        if(rpm_cmp > 119) rpm_cmp = 0; // complete engine cycle | Reset CMP control.
        
    } // end of TIMER2 Interrupt
    
} // end of "void __interrupt() ISR()" 

/******************************************************************************/
void main()     //  main function
/******************************************************************************/
{
    pic_ini();
    
    while(1)  
    {
        LATBbits.LATB3 = 1;  // On this pin you can check the execution of this part of the program.
        __delay_ms(1000);    // I want the MCU to spend time here, to check and
        LATBbits.LATB3 = 0;  // ensure that the peripherals are still operating.
        __delay_ms(100);
        NOP();               // nothing
        
    } // end of "while"
    
} // end of main function

/******************************************************************************/
