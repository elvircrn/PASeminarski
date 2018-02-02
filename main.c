#pragma config FOSC = HS, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF,       \
                      CPD = OFF, BOREN = OFF, CLKOUTEN = OFF
#pragma config IESO = OFF, FCMEN = OFF, WRT = OFF, VCAPEN = OFF, PLLEN = OFF,  \
                      STVREN = OFF, LVP = OFF

#define _XTAL_FREQ 8000000
#define F_PWM 19610.0f
#define T_PWM 1 / 19610.0f
#define T_PWM_MS ((1 / 19610.0f) * 1000.0f)
/*
   RA0 -> potenciometar
   RA1 -> senzor
   RA2 -> output
   C   -> output
   */
#include <xc.h>

#define POTEN 0
#define SENSOR 1

#define __deyal_ms(x) _delay((unsigned long)((x) * (_XTAL_FREQ / 4000.0)))

void interruptSetup() {
  //  ADCON1 = 0x82; // AN0->AN4 selected as analog input
  //  ADCON0 = 0b11000001; // Configue analog mode
  GIE = 1; //Enable global interrupt
  PEIE = 1; //Enable peripheral interrupt
  ADIE = 1; //Enable ADC interrupt 
  ADIF = 0;
  ADGO=1;
  TMR0IF=0;
  TMR0IE=1;
  TMR0CS = 0;
  PSA = 0;
}

int analog_selected = 0;

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

volatile unsigned char turnOn, d;
void interrupt interruptHandler(void){
//  if (TMR0IE && TMR0IF) {
//    ADIF = 0;
//    TMR0IF = 0;
//    //PORTB = 0xff;
//
//    if (turnOn) {
//        PS2 = 1;
//        PS1 = 1;
//        PS0 = 1;
//        PORTB = 0xff;
//    }
//    else {
//        PS2 = 1;
//        PS1 = 1;
//        PS0 = 0;
//        PORTB = 0;
//    }
//    turnOn ^= 1;
//  }
  
  if (ADIE && ADIF) {
//    volatile unsigned char lock = (unsigned char)((ADRESH / 255.0f) * 51.0f);
    PORTD = ~reverse(ADRESH);
//    PORTBbits.RB0 = 1;
//    __delay_us(lock);
//    PORTBbits.RB0 = 0;
//    __delay_us(51 - lock);
    ADGO = 1;
    ADIF = 0;
  }
}


void initialize() {
  ANSELA = 0xff;
  TRISA = 0xff;
  TRISD = 0;
  PORTA = 0;
  TRISB = 0;
  ANSELB = 0;
  PORTB=0;
  PORTD = 0;
  TRISD = 0;
  ANSELD = 0;


  ADCON1bits.ADFM = 0;

  ADCON1bits.ADCS2 = 1;
  ADCON1bits.ADCS1 = 1;
  ADCON1bits.ADCS0 = 1;

  ADCON1bits.ADNREF = 0;
  ADCON1bits.ADPREF1 = 0;
  ADCON1bits.ADPREF0 = 0;

  ADCON0bits.ADON = 1;

  ADCON0bits.CHS4 = 0;
  ADCON0bits.CHS3 = 0;
  ADCON0bits.CHS2 = 0;
  ADCON0bits.CHS1 = 0;
  ADCON0bits.CHS0 = 0;

}

unsigned char getConversionRes(char id) {
  ADCON0bits.CHS0 = id;
  ADCON0bits.ADGO = 1;
  while (ADCON0bits.ADGO) ;
  return ADRESH;
}

float getVoltage(char id) { return 5.0f * (getConversionRes(id) / 255.0f); }

#define KI -8998.71f
#define KP 9001.28f
#define MAX 5.0f
#define MIN 0.0f
#define SET_POINT 0.3f
#define SAMPLE_RATE 0.5f

void _main(void) {
  initialize();
  float uprev = 0, error;
  float integral = 0;
  float setPoint = getVoltage(POTEN);
  float output = 0.0f;
  while (1) {
    error = getVoltage(POTEN) - getVoltage(SENSOR);
    integral += error * SAMPLE_RATE;
    output = KP * error + KI * integral;

    if (output > MAX)
      output = MAX;
    if (output < MIN)
      output = MIN;

    PORTBbits.RB0 = 0;
    __delay_ms(T_PWM_MS * 0.3);
    PORTBbits.RB0 = 1;
    __delay_ms(T_PWM_MS * 0.7);

    __delay_ms(SAMPLE_RATE * 1000.0f);
  }
  return;
}

void setup() {
  ANSELB = 0;
  TRISB = 0;
  PR2 = 0b01100101;
  T2CON = 0b00000100;
  CCPR1L = 0b01100101;
  CCP1CON = 0b00111100;
}

void main() {
  //  setup();
  initialize();
  interruptSetup();
  unsigned char id = 1;
  for (;;) {
//    PORTB = (d & (0xff - 1)) + (PORTB & 1);;
//    PORTB = getConversionRes(POTEN); 
//    PORTB = (lock & (0xff - 1)) + (PORTB & 1); 
//    PORTBbits.RB0 = 1;
//    __delay_ms(lock / 10);
//    PORTBbits.RB0 = 0;
//    __delay_ms((255 - lock) / 10);
    //    for (dc = 0; dc < 128; dc++) {
    //      CCPR1L = dc;
    //      CCPR2L = 128 - dc;
    //      __delay_ms(10);
    //    }
    //    for (dc = 127; dc > 0; dc--) {
    //      CCPR1L = dc;
    //      CCPR2L = 128 - dc;
    //      __delay_ms(10);
    //    }
  };
}

