/*
 * File:   main.c
 * Author: casti
 *
 * Created on 4 de Dezembro de 2024, 21:43
 */

#include <xc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"

uint8_t interrupt_timer0 = NO; // Timebase auxiliary control.
uint8_t rpm_ckp = 0; // Control of the pulse that the phonic wheel would generate.
uint8_t rpm_cmp = 0; // CMP sensor position control.
uint8_t turn_ctrl; // Engine cycle control.

/*******************************************************************************
 * function: interrupt
 *******************************************************************************/
void __interrupt() ISR()
{
    // Timebase 1 ms | Timer0
    if(INTCONbits.TMR0IF)                 // Timer0 Interrupt
    { 
        INTCONbits.TMR0IF = NO;           // Clear the timer0 interrupt flag.
        timer0_write(0xFE0C);             // Reset Timer0.
        LATBbits.LATB7 = ~LATBbits.LATB7; // For function check and measurement purposes only.
        
    } // end of "if TIMER 0".
    
    if(PIR1bits.TMR2IF)           // Timer2 Interrupt | PWM match PR2 and TMR2.
    {
        rpm_ckp++;                // Increment for a new pulse of the phonic wheel.
        PIR1bits.TMR2IF = 0;      // Clear the timer2 interrupt flag.
        
        if(rpm_ckp == 59)         // The minus two teeth gap has reached. One turn completed on the next pulse.
        {
            LATBbits.LATB0 = 0;   // Causes the CCP/PWM to shutdown.
            turn_ctrl++;          // Increases lap control.
            LATCbits.LATC2 = LOW; // Ensures PWM output remains LOW during minus 2 tooth failure.
            
        } // end of if "rpm_ckp == 59".
        
        if(rpm_ckp == 60) // lap completed.
        {
            rpm_ckp = 0; // Reset phonic wheel control variable.
            LATBbits.LATB0 = 1; // Restarts PWM.
            
            if(turn_ctrl == 2) // 2 complete turns, 1 complete engine cycle
            {
                turn_ctrl = 0; // Reset lap control.
                
            } // end of if "turn_ctrl == 2".
        
        } // end of if "turn_ctrl ==2".
        
        // Synchronization for phase sensor (CMP) 
        // typical for VW line of engines with 60-2 phonic wheel.
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

    } // end if TMER 2
    
} // end of "void __interrupt() ISR()"

/*******************************************************************************
 * main function
 *******************************************************************************/
void main()
{
    pic_ini();
    while(1)
    {
        LATBbits.LATB5 = ~PORTBbits.RB5;
        __delay_ms(500); // Spending time checking the continuity of peripheral operation
        NOP(); // I don't know what this is or what it's for, I thought it was pretty so I put it here.
        
    }// end od "while"
    
} // end do "main function

/*******************************************************************************
 * 
 *******************************************************************************/