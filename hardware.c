/*
 * File:   hardware.c
 * Author: Antonio Castilho
 *
 * Created on 8 de Dezembro de 2024, 12:01
 */


#include <xc.h>
#include "hardware.h"
#include "main.h"

/******************************************************************************/
void pic_ini(void)
{
       /* Set Interrupt
     **************************************************************************/
    INTCONbits.GIE = ENABLE;    // Global interrupt enable | pg. 101.
    INTCONbits.PEIE_GIEL = ENABLE;
    
    /* Peripherals 
     **************************************************************************/
    lcd_ini();   lcd_cursorOff();
    adc_ini();
    timer0_ini();
    timer2_ini();
    pwm1_ini();
    
    /* Set I/O
     **************************************************************************/
    TRISCbits.RC2  =  0; // RC2 pin PWM out.
    TRISB = 0x00; // The pins of port B are configured for output.
    LATB  = 0xFF; // All port B pins start high.
    
}

/******************************************************************************/
void timer0_ini(void)
{
    T0CON = 0x90; // Timer 0 control register|Enable|16 bits|Prescaler 256| pg. 127.
    T0CONbits.TMR0ON = ENABLE;
    INTCONbits.TMR0IE = ENABLE; // Timer 0 interrupt enable | pg. 101.
    
    timer0_write(0xFE0C);
}

/******************************************************************************/
void  timer0_write( uint16_t timer_value)
{
    TMR0L = (timer_value & 0x00FF);
    TMR0H = (timer_value >> 8) & 0x00FF;
} // end of void timer0_write(uint16_t timer_value)

/*******************************************************************************
 * 
 *  
 ******************************************************************************/
void timer2_ini(void)
{
    T2CON = 0x07; // pg. 137.
    PIE1bits.TMR2IE = ENABLE; // Interrupt TIMER2 on
    PIR1bits.TMR2IF = NO;     // Timer2 Flag
    IPR1bits.TMR2IP = ENABLE; // TMR2 to PR2 Match High priority
    TMR2 = 0;
}

/******************************************************************************/
void adc_ini(void)
{
    // Configure Port A as input, according to the channels that will be needed.
    TRISA = 0x0F;  // channel in AN0.
    ADCON1 = 0x0B; // 4 channels Pg 262.
    ADCON2 = 0xBE; // Right Justified, 4Tad and Fosc/32. Pg 263.
    ADCON0bits.ADON = ON;
    ADRESH=0;	   // Flush ADC output Register. Pg 261.
    ADRESL=0;
} // end of function void adc_ini(void)

/******************************************************************************/
uint16_t adc_read(uint8_t ch)
{
    uint16_t value;    
    ADCON0bits.CHS = ch; // selects the channel to be read.
    ADCON0bits.GO = ON;  // start conversion.
    while(ADCON0bits.GO_DONE == TRUE); // wait for the conversion.
    value = (uint16_t)((ADRESH << 8) + ADRESL);
    return value;
} // end of function uint16_t adc_read(uint8_t ch)

/*******************************************************************************/
void pwm1_ini(void)
{
   /*************************************************************************
 
     *************************************************************************/
    OSCCON = 0x72;
    T2CONbits.TMR2ON = ON;  // Turn on Timer 2.     
    TMR2 = 0; // Initializes TMR2.
    // Initializes module CCP1. page 151.
    CCP1CON = 0X0F; // PWM mode: P1A, P1C active-high; P1B, P1D active-high.
    // Configure the PS2 of PWM1, to control the frequency and period of PWM1.
    uint8_t pr_var = 0; //(uint8_t)(round(((float)(_XTAL_FREQ)/
            //((float)(PWM1_FREQUENCY)*4.0*(float)TMR2PRESCALE))-1.0));
    
    pr_var = (uint8_t)(round((1/(float)PWM1_FREQUENCY)/((4/(float)_XTAL_FREQ) * (float)TMR2PRESCALE * (float)TMR2POSTCALER)));
    
    //pr_var = 62;
    PR2 = pr_var;
    // Configures Registers to control the PWM1's Duty Cycle.
    uint16_t cycle = (uint16_t)(round(((((float)DUTY_CYCLE)/
            1000.0)*4.0*((float)(pr_var)+1.0)))); // Calculates value for CCPRL.
    CCP1CONbits.DC1B0 = cycle; // Bit Lsb
    CCP1CONbits.DC1B1 = cycle >> 1; // Bit LSb
    CCPR1L = cycle >> 2; // Bit Msb.
    
    /* Auto-shutdown
     **************************************************************************/
    TRISBbits.TRISB0 = 0;
    LATBbits.LATB0 = 1;
    ECCP1DEL = 0b10000000;
    ECCP1AS  = 0b11010000;
    
    
    
}
/******************************************************************************/
void pwm1_setDutyPot(uint16_t ccpr1_aux)
{
    CCP1CONbits.DC1B0 = ccpr1_aux; // Bit 0 - Lsb
    CCP1CONbits.DC1B1 = ccpr1_aux >> 1; // Bit 1 - Lsb
    CCPR1L = ccpr1_aux >> 2; // Bit 8 - Msb.
}

/******************************************************************************/
void pwm1_setPeriod(uint8_t period)
{
    T2CON = 0x04;
    PR2 = period;
    
}