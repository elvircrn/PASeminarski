#pragma config FOSC = HS, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF,CPD = OFF, BOREN = OFF, CLKOUTEN = OFF
#pragma config IESO = OFF, FCMEN = OFF, WRT = OFF, VCAPEN = OFF, PLLEN = OFF, STVREN = OFF, LVP = OFF
#include <xc.h>

/**
   RA0 -> potenciometar
   RA1 -> senzor
   RA2 -> output
   C   -> output
*/

#define _XTAL_FREQ 8000000
#define F_PWM 19610.0f
#define T_PWM 1 / 19610.0f
#define T_PWM_MS ((1 / 19610.0f) * 1000.0f)
// #define __deyal_ms(x) _delay((unsigned long)((x) * (_XTAL_FREQ / 4000.0)))
#define KI -8999.36f
#define KP 9000.64f
#define MAX 5.0f
#define MIN 0.0f
#define SET_POINT 0.3f
#define SAMPLE_RATE 0.5f

// Za 7-segmentni displej sa zajednickom katodom
//unsigned char digits[]={0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
// Za 7-segmentni displej sa zajednickom anodom
unsigned char digits[]={0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90};

float sensor = 0, potenciometar = 0, nivo=0;
unsigned char currentSelection = 0, counter = 0, running_conversion = 0, conversion_state = 0;
int printTimer = 0;

unsigned char reverse(unsigned char b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

void printDigit(unsigned char digit, unsigned char selection) {

  // TODO: Provjeri da li onaj kloc kloc zna razliku izmedju ~ i !.
  PORTB = digits[digit];
  if(selection)
    PORTC = 0b10000001;
  else
    PORTC = 0b01000010;
}

void initializeAsyncConversion() {
  ANSELA = 0xff;
  TRISA = 0xff;
  PORTA = 0;
  TRISB = 0;
  ANSELB = 0;
  PORTB=0;
  PORTD = 0;
  TRISD = 0;
  ANSELD = 0;
  TRISC=0;
  PORTC=0;



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
  GIE = 1; //Enable global interrupt
  PEIE = 1; //Enable peripheral interrupt
  ADIE = 1; //Enable ADC interrupt
  ADIF = 0;
  running_conversion = 1;
  ADGO=1;
}

void initializeTimer() {
  OPTION_REGbits.TMR0CS = 0;
  OPTION_REGbits.PSA = 0;
  OPTION_REGbits.PS0 = 1;
  OPTION_REGbits.PS1 = 1;
  OPTION_REGbits.PS2 = 0;
  TMR0 = 0;
  INTCONbits.GIE = 1;
  INTCONbits.TMR0IE = 1;
}

static unsigned int g_seed;

// Used to seed the generator.
inline void fast_srand(int seed) {
    g_seed = seed;
}

// Compute a pseudorandom integer.
// Output value in range [0, 32767]
inline int fast_rand(void) {
    g_seed = (214013*g_seed+2531011);
    return (g_seed>>16)&0x7FFF;
}


#define TOP_VOLT 1.05f
#define BOT_VOLT 0.18f
#define MID_VOLT 0.42f
#define TOP_CM   25
#define MID_CM   17
#define BOT_CM   10

float ocitanja[] = {0.20, 0.22, 0.24, 0.26, 0.30, 0.33, 0.36, 0.4, 0.48, 0.55, 0.66, 0.76, 0.84, 0.91, 0.98, 1.05};

void interrupt interruptHandler(void){
      //Handler konverzije
 if (ADIE && ADIF) {
    float result = ((ADRESH / 255.0f) * 5.0f);
    if(!conversion_state) {
        if(result <= 2.5f)
            potenciometar = result / 2.5f * (MID_VOLT - BOT_VOLT) + BOT_VOLT;
        else
            potenciometar = (result - 2.5f) / 2.5f * (TOP_VOLT - MID_VOLT) + MID_VOLT;
    } else {
       
      float k = result >= MID_VOLT ? ((TOP_CM - MID_CM) / (TOP_VOLT - MID_VOLT)) : ((MID_CM - BOT_CM) / (MID_VOLT - BOT_VOLT));
      float n = result >= MID_VOLT ? MID_CM : (MID_CM - MID_VOLT * k);
      sensor = result;
      if(result >= MID_VOLT)
          result -= MID_VOLT;
      nivo = k * result + n;/*
        nivo = BOT_CM;
        for (int i = 0; i < TOP_CM - BOT_CM; i++) {
            if(ocitanja[i] <= result && result <= ocitanja[i + 1])
                break;
            else
                nivo++;
        }
        if(ocitanja[i] < result)
          nivo++;
        */

    }
    conversion_state = !conversion_state;
    ADCON0bits.CHS0 = !ADCON0bits.CHS0;
    ADIF = 0;
    running_conversion = 0;
  }
  //Handler timera
  if (TMR0IE && TMR0IF) {
    if(!running_conversion)
       counter++;
    if(counter == 50) {
        running_conversion = 1;
        counter = 0;
        ADGO = 1;
    }

    currentSelection = !currentSelection;
    unsigned char firstDigit = (int)(((int) nivo) % 100 / 10);
    unsigned char secondDigit = ((int) nivo) % 10;
    printDigit(currentSelection ? secondDigit : firstDigit, currentSelection);
    TMR0IF = 0;
  }

}

void main(void) {
  initializeAsyncConversion();
  initializeTimer();
  float uprev = 0, error;
  float integral = 0;
  float output = 0.0f;
  float eprev = 0.0f;
  while (1) {
    volatile float _potenciometar = potenciometar;
    volatile float _sensor = sensor;
    error = _potenciometar - _sensor;
    integral += KP * error + KI * eprev;
    output = integral;

    if (output > MAX)
      output = MAX;
    if (output < MIN)
      output = MIN;


    PORTD = ~(unsigned char)(output*255/5);
    eprev = error;
    __delay_ms(500);
  }
  return;
}

