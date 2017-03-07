/*
 * File:   main.c
 * Author: dkg
 *
 * Created on March 2, 2017, 10:48 PM
 */

#pragma config FOSC = INTOSCIO // Oscillator Selection bits (INTOSC oscillator: 

//CLKOUT function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF     // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Detect (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)


#define PWM_ON CCP1CON=CCP1CON &  0b11111100
#define PWM_OFF CCP1CON=CCP1CON & 0b11110000
#define US_TRIG 0x20 //GPIO 5
#define T1GP 0x10 // GPIO 4


#define _XTAL_FREQ 4000000

#include <xc.h>

// Function declarations.
void init(void);
int pwmGenerator(int freq);
void err(void);

void main(void) {
    init();
    /*
        PR2 = 30;
        T2CON = 0b00000101;
        CCPR1L = 0b00001111;// 0b10101110;    
        CCP1CON = 0b00011100;//0b00111100;
     */

    //  if (!pwmGenerator(300)) {
    //     err();
    // }

    static bit t1gp_high, got_dist, trig;
    unsigned int pulse_width, dist = 0;
    t1gp_high = 0;
    got_dist = 0;
    trig = 0;


    while (1) {

        // Send a Trigger pulse to ultrasonic sensor.
        if (!trig && t1gp_high == 0 && !(GPIO & T1GP)) {
            GPIO |= US_TRIG;
            __delay_us(10);
            GPIO &= ~US_TRIG;
            trig = 1;
        }

        // Measure return pulse.
        //Check to see if T1GP goes HIGH
        if (t1gp_high == 0 && (GPIO & T1GP)) {
            t1gp_high = 1;
            pulse_width = 0;
        }

        // Check to see if T1GP went LOW after going high 
        if (t1gp_high == 1 && !(GPIO & T1GP)) {
            pulse_width = TMR1H; // Read the value of timer
            pulse_width = (pulse_width << 8) | TMR1L;
            TMR1L = 0;
            TMR1H = 0;
            t1gp_high = 0;
            got_dist = 1;
        }

        if (got_dist) {
            got_dist = 0;
            trig = 0;
            dist = pulse_width / 58.82;
            pwmGenerator(dist*10);
            __delay_ms(100);
        }


    }

    return;
}

void init(void) {
    // Basic Pic init.
    OSCCON = 0x70;
    GPIO = 0x00;
    ANSEL = 0x00;
    TRISIO = 0x00;
    ADCON0 = 0x00; // Disable AD convertors.
    CMCON0 = 0x7;
    WPU = 0x00;


    // Initialize Timer 1 in Gating Mode to calculate pulse width.
    TRISIO |= T1GP;
    T1CON = 0b11010001;
    T1GSS = 1; //Set Gate Source as T1g
    TMR1L = 0;
    TMR1H = 0;

}

// pwmGenerator generates the pwm wave with frequency freq.It returns zero
// if its unable to calculate the values.

int pwmGenerator(int freq) {
    float period, tosc = 0;
    int pr2, scaler, ccp;
    float duty = 0.5;

    period = 1 / (float) freq;
    tosc = 1 / (float) _XTAL_FREQ;

    // Selecting a prescalar value  (1,4,16) for TMR2.
    for (scaler = 1; scaler <= 16; scaler = scaler * 4) {
        pr2 = period / (4 * tosc * scaler) - 1;
        ccp = duty * 4 * (pr2 + 1);
        // check for bounds.
        if (pr2 <= 255 && ccp <= 1023) {
            break;
        }
        pr2 = 0;
        ccp = 0;
    }

    if (pr2) {
        // Setup PWM params.
        PR2 = pr2;

        // Setup TMR2 prescalar.    
        switch (scaler) {
            case 1:
                T2CON = 0b00000100;
                break;
            case 4:
                T2CON = 0b00000101;
                break;
            case 16:
                T2CON = 0b00000110;
                break;
            default:
                T2CON = 0b00000100;
                break;
        }

        int lsb = 0;
        // TODO: PWM duty cycle needs to be calculated for prescalar change..
        CCPR1L = ccp >> 2;
        lsb = (ccp & 11) << 4;


        CCP1CON = 0b00001100 | lsb;
        return 1;
    }

    return 0;
}

void err(void) {
    PR2 = 0b01111100;
    T2CON = 0b00000101;
    CCPR1L = 0b00111110;
    CCP1CON = 0b00011100;
    __delay_ms(500);
    __delay_ms(500);
    __delay_ms(500);

    PR2 = 0b00110001;
    T2CON = 0b00000101;
    CCPR1L = 0b00011000;
    CCP1CON = 0b00111100;
    __delay_ms(500);
    __delay_ms(500);
    __delay_ms(500);
    __delay_ms(500);

    PR2 = 0b01111100;
    T2CON = 0b00000100;
    CCPR1L = 0b00111110;
    CCP1CON = 0b00011100;
    __delay_ms(500);
    __delay_ms(500);

}

