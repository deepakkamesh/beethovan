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

#include <xc.h>

// Function declarations.
void init(void);
int pwmGenerator(int freq);
void err(void);

void main(void) {
    unsigned int pulse_width, dist = 0;

    init();

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
            pwmGenerator((dist/2 - 5)*100 + 300 );
            continue;
        }
        
        pwmGenerator(10000);

    }

    return;
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
    float period, tosc = 0, duty = 0.4;
    int pr2, scaler, ccp = 0;
    static bit got = 0;

    period = 1 / (float) freq;
    tosc = 1 / (float) _XTAL_FREQ;

    // Selecting a prescalar value  (1,4,16) for TMR2.
    for (scaler = 1; scaler <= 16; scaler = scaler * 4) {
        pr2 = period / (4 * tosc * scaler) - 1; // Calculate pwm period.
        ccp = duty * 4 * (pr2 + 1); // Calculate pwm duty.

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

// errTone generates a tone for error notification.

void errTone(void) {
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

