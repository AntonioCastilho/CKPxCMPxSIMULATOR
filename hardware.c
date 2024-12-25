/*
 * File:   hardware.c
 * Author: Automotive Technologist Antonio Castilho
 * 
 * Program environment: MPLAB X IDE v6.20, XC8 v2.50, C std C90, PIC18F455
 * Reference: Microchip PIC18F4550 Datasheet.
 * 
 * MIT License  (see: LICENSE em github)
 * Copyright (c) 2022 Antonio Aparecido Ariza Castilho
 * <https://github.com/AntonioCastilho>
 *
 * Created on 8 de Dezembro de 2024, 12:01
 */

/*******************************************************************************
 * Includes 
 ******************************************************************************/
#include <xc.h>
#include "hardware.h"
#include "main.h"

/******************************************************************************/
void pic_ini(void)
/******************************************************************************/
{
    /* Set Interrupt
    **************************************************************************/
    INTCONbits.GIE = 1;    // Global interrupt enable | pg. 101.
    INTCONbits.PEIE_GIEL = 1;
    
    /* Peripherals 
    **************************************************************************/
    lcd_ini();  
    lcd_cursorOff(); // Starts a 2 X16 LCD and turns off its cursor.
    adc_ini();       // Starts the Analog Digital converter.   
    timer0_ini();    // Starts Timer 0.
    timer1_ini();    // Starts Timer 1.
    timer2_ini();    // Starts Timer 2.
    pwm1_ini();      // Starts PWM 1.
    pwm2_ini();      // Starts PWM 2.
    
    
    /* Set I/O
    ***************************************************************************/
    TRISB = 0x00; // The pins of port B are configured for output.
    LATB  = 0xFF; // All port B pins start high.
    
} // end of pic_ini())

/******************************************************************************/
void timer0_ini(void)
/******************************************************************************/
{
    T0CON = 0x90; // Timer 0 control register|Enable|16 bits|Prescaler 256| pg. 127.
    T0CONbits.TMR0ON = 1;
    INTCONbits.TMR0IE = 1; // Timer 0 interrupt enable | pg. 101.
    
    timer0_write(0xFE0C);
    
} // end timer0_ini()

/******************************************************************************/
void  timer0_write( uint16_t timer_value)
/******************************************************************************/
{
    TMR0L = (timer_value & 0x00FF);
    TMR0H = (timer_value >> 8) & 0x00FF;
    
} // end of void timer0_write(uint16_t timer_value)

/******************************************************************************/
void timer1_ini(void)
/******************************************************************************/
{
    T1CON = 0xB1; //timer1 on| pg. 131.
    TMR1L = 0;
    TMR1H = 0;
    PIE1bits.TMR1IE = 1;
    
    TMR1IF = 0;
   
    timer1_write(0xCF2C); 
    
} //end timer1_ini

/******************************************************************************/
void timer1_write(uint16_t timer_value)
/******************************************************************************/
{
    TMR1L = (timer_value & 0x00FF);
    TMR1H = (timer_value >> 8) & 0x00FF;
} 
// end of void timer1_write(uint16_t timer_value)

 /*****************************************************************************/
void timer2_ini(void)
/******************************************************************************/
{
    T2CON = 0x07; // pg. 137.
    PIE1bits.TMR2IE = 1; // Interrupt TIMER2 on
    PIR1bits.TMR2IF = 0;     // Timer2 Flag
    IPR1bits.TMR2IP = 1; // TMR2 to PR2 Match High priority
    TMR2 = 0;
    
} // end of timer2()

/******************************************************************************/
void adc_ini(void)
/******************************************************************************/
{
    // Configure Port A as input, according to the channels that will be needed.
    TRISA = 0x0F;  // channel in AN0.
    ADCON1 = 0x0B; // 4 channels Pg 262.
    ADCON2 = 0xBE; // Right Justified, 4Tad and Fosc/32. Pg 263.
    ADCON0bits.ADON = 1;
    ADRESH=0;	   // Flush ADC output Register. Pg 261.
    ADRESL=0;
    
} // end of function void adc_ini(void)

/******************************************************************************/
uint16_t adc_read(uint8_t ch)
/******************************************************************************/
{
    uint16_t value;    
    ADCON0bits.CHS = ch; // selects the channel to be read.
    ADCON0bits.GO = 1;  // start conversion.
    while(ADCON0bits.GO_DONE == 1); // wait for the conversion.
    value = (uint16_t)((ADRESH << 8) + ADRESL);
    return value;
    
} // end of function uint16_t adc_read(uint8_t ch)

/******************************************************************************/
void pwm1_ini(void)
/******************************************************************************/
{
    // I/O Set
    TRISCbits.RC2  =  0; // RC2 pin PWM1 out.
    
    OSCCON = 0x72;
    T2CONbits.TMR2ON = 1;  // Enables the operation of Timer0. 
    TMR2 = 0; // Initialize the timer2 counter.
    
    // Initializes module CCP1. page 151.
    CCP1CON = 0X0F; // PWM mode: P1A, P1C active-high; P1B, P1D active-high.
    
    // Configure the PS2 of PWM1, to control the frequency and period of PWM1.
    uint8_t pr_var = 0; 
    pr_var = 
        (uint8_t)(round((1/(float)PWM1_FREQUENCY)/((4/(float)_XTAL_FREQ) * 
        (float)TMR2PRESCALE * (float)TMR2POSTCALER)));
    
    PR2 = pr_var;
    
    // Configures Registers to control the PWM1's Duty Cycle.
    uint16_t cycle =
        (uint16_t)(round(((((float)DUTY_CYCLE)/1000.0)*4.0*(
        (float)(pr_var)+1.0)))); // Calculates value for CCPRL.
    
    CCP1CONbits.DC1B0 = cycle; // Bit Lsb
    CCP1CONbits.DC1B1 = cycle >> 1; // Bit LSb
    CCPR1L = cycle >> 2; // Bit Msb.
    
    /* Auto-shutdown
    ***************************************************************************/
    // I/O set
    TRISBbits.TRISB0 = 0;
    LATBbits.LATB0 = 1;
    
    ECCP1DEL = 0b10000000;
    ECCP1AS  = 0b11010000;
    
} // end of pwm1_ini()

/******************************************************************************/
void pwm2_ini(void)
/******************************************************************************/
{
    // I/O Set
    TRISCbits.RC1  =  0; // RC1 pin PWM 2 out.
    
    //OSCCON = 0x72;
    //T2CONbits.TMR2ON = 1;  // Enables the operation of Timer0. 
    //TMR2 = 0; // Initialize the timer2 counter.
    
    // Initializes module CCP2. page 151.
    CCP2CON = 0X0F;

    // Configure the PS2 of PWM2, to control the frequency and period of PWM2.
    uint8_t pr_var = PR2; 
    // pr_var = 
    //    (uint8_t)(round((1/(float)PWM1_FREQUENCY)/((4/(float)_XTAL_FREQ) * 
    //    (float)TMR2PRESCALE * (float)TMR2POSTCALER)));
    
    // PR2 = pr_var;
    
    // Configures Registers to control the PWM2's Duty Cycle.
    uint16_t cycle2 =
        (uint16_t)(round(((((float)DUTY_CYCLE)/1000.0)*4.0*(
        (float)(pr_var)+1.0)))); // Calculates value for CCPRL.
    
    CCP2CONbits.DC2B0 = cycle2; // Bit Lsb
    CCP2CONbits.DC2B1 = cycle2 >> 1; // Bit LSb
    CCPR2L = cycle2 >> 2; // Bit Msb.
    
}

/******************************************************************************/
void pwm1_setDutyPot(uint16_t ccpr1_aux)
/******************************************************************************/
{
    CCP1CONbits.DC1B0 = ccpr1_aux; // Bit 0 - Lsb
    CCP1CONbits.DC1B1 = ccpr1_aux >> 1; // Bit 1 - Lsb
    CCPR1L = ccpr1_aux >> 2; // Bit 8 - Msb.
    
} // end of pwm1_setDutyPot()

/******************************************************************************/
void pwm2_setDutyPot(uint16_t ccpr2_aux)
/******************************************************************************/
{
    CCP2CONbits.DC2B0 = ccpr2_aux; // Bit 0 - Lsb
    CCP2CONbits.DC2B1 = ccpr2_aux >> 1; // Bit 1 - Lsb
    CCPR2L = ccpr2_aux >> 2; // Bit 8 - Msb.
    
} // end of pwm2_setDutyPot()

/******************************************************************************/
void pwm1_setPeriod(uint8_t period)
/******************************************************************************/
{
    T2CON = 0x04;
    PR2 = period;
    
} // end of pwm1_setPeriod()

/******************************************************************************/