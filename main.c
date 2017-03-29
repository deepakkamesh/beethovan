/*
 * File:   main.c
 * Author: dkg
 * PIC12F683 - ultrasonic music composer. 
 * Buzzer range 300Hz to 6000Hz (3000Hz effective)
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
#define TOSC (1 / (float) _XTAL_FREQ)
#define DUTY 0.8 // PWM Duty Cycle.
#define STEP 3 // Dist in cm when the note changes.
#define P(x) pwmGenerator(x)
#define S(x) __delay_ms(x*400)

#include <xc.h>

// Frequency Table of notes.
#define C 2441
#define D 2741
#define E 3048
#define F 3255
#define G 3654
#define A 4058
#define B 4562
#define C2 4882

// Function declarations.
void init(void);
int pwmGenerator(int freq);
void err(void);
void playIntro(void);

void main(void) {
    unsigned int pulse_width = 0;
    unsigned char dist = 0;

    init();
    playIntro();

    while (1) {
        // Send a Trigger pulse to ultrasonic sensor.
        GPIO |= US_TRIG;
        __delay_us(10);
        GPIO &= ~US_TRIG;

        // Wait for line to go high.
        while (!(GPIO & T1GP));

        // Wait for line to go low after going high.
        while (GPIO & T1GP);

        pulse_width = TMR1H;
        pulse_width = (pulse_width << 8) | TMR1L;
        TMR1L = 0;
        TMR1H = 0;
        dist = pulse_width / 58.82;

        // Keep the frequency between 300Hz to 3000Hz 
        // and reduce sensitivity to 2 cm.
        if (dist > 5 && dist < 64) {
            //  pwmGenerator((dist / 2 - 5)*100 + 300);
            if ((dist - 5) / STEP == 0) {
                P(C);
                S(1);
            }
            if ((dist - 5) / STEP == 1) {
                P(D);
                S(1);
            }
            if ((dist - 5) / STEP == 2) {
                P(E);
                S(1);
            }
            if ((dist - 5) / STEP == 3) {
                P(F);
                S(1);
            }
            if ((dist - 5) / STEP == 4) {
                P(G);
                S(1);
            }
            if ((dist - 5) / STEP == 5) {
                P(A);
                S(1);
            }
            if ((dist - 5) / STEP == 6) {
                P(B);
                S(1);
            }
            if ((dist - 5) / STEP == 7) {
                P(C2);
                S(1);
            }
            continue;
        }
        pwmGenerator(20000);
    }

    return;
}


// playIntro plays a song.

void playIntro(void) {
    P(C);
    S(1);
    P(C);
    S(1);
    P(G);
    S(1);
    P(G);
    S(1);
    P(A);
    S(1);
    P(A);
    S(1);
    P(G);
    S(2);
    P(F);
    S(1);
    P(F);
    S(1);
    P(E);
    S(1);
    P(E);
    S(1);
    P(D);
    S(1);
    P(D);
    S(1);
    P(C);
    S(1);
}

// init initializes the pic.

void init(void) {
    // Basic Pic initialization.
    OSCCON = 0x70;
    GPIO = 0x00;
    ANSEL = 0x00;
    TRISIO = 0x00; // Setup GPIO as output.
    ADCON0 = 0x00; // Disable AD.
    CMCON0 = 0x7;
    WPU = 0x00;

    // Initialize Timer 1 in Gating Mode to calculate pulse width.
    TRISIO |= T1GP;
    T1CON = 0b11010001;
    T1GSS = 1; //Set Gate Source as T1g
    TMR1L = 0;
    TMR1H = 0;
}

// pwmGenerator generates the pwm wave with frequency freq. It returns zero
// if its unable to calculate the values.

int pwmGenerator(int freq) {
    int pr2, scaler, ccp = 0;
    static bit got = 0;
    freq = freq / 2;

    // Selecting a prescalar value  (1,4,16) for TMR2.
    for (scaler = 1; scaler <= 16; scaler = scaler * 4) {
        pr2 = (1 / (float) freq) / (4 * TOSC * scaler) - 1; // Calculate pwm period.
        ccp = DUTY * 4 * (pr2 + 1); // Calculate pwm duty.

        // check for bounds. PR2 is 8 bit and CCP1RL:CCP1CON<5:4> is 10 bit.
        if (pr2 <= 255 && ccp <= 1023) {
            // Setup PWM parameters. PR2, T2CON, CCPR1L & CCP1CON.    
            PR2 = pr2;
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
            CCPR1L = ccp >> 2;
            int lsb = (ccp & 11) << 4;
            CCP1CON = 0b00001100 | lsb;
            return 1;
        }
    }
    return 0;
}

