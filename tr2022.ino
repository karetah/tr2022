#include <inttypes.h>

#include "Arduino.h"
#include <avr/io.h>

// http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
// https://content.arduino.cc/assets/Pinout-UNOrev3_latest.pdf

uint8_t sreg = SREG;

void setup()
{

    cli();

    // Stop timer before configuring
    TCCR1B = 0;

    // ADC Control and Status Register A
   ADCSRA |= (0 << ADEN);  // Disable ADC

    // ADC Multiplexer Selection Register
   ADMUX =
        0
        | (1 << MUX0) | (1 << MUX1) | (1 << MUX2) // Select ADC7 Negative input 
        | (1 << REFS1) | (1 << REFS0)  // Analog Comparator Input Capture Enable      
        ;  

   ADCSRB =
        0
        | (1 << ACME)   // Analog Comparator Multiplexer Enable
        | (1 << ADTS0) | (0 << ADTS1) | (0 << ADTS2) // ADC Auto Trigger Source : Analog Comparator 
        ;
    // Analog Comparator Control and Status Register
   ACSR =
        0
        | (1 << ACD) | (1 << ACBG)  // Analog Comparator Disable, Analog Comparator Bandgap Select
        | (1 << ACIE) | (1 << ACIC)  // Analog Comparator Interrupt Enable, Analog Comparator Input Capture Enable      
        | (0 << ACIS1) | (0 << ACIS0)  // Analog Comparator Interrupt Mode Select
        ;
    // Digital Input Disable Register 1
   DIDR1 =
        0
        | (1 << AIN1D) | (1 << AIN0D)  // AIN1, AIN0 Digital Input Disable
        ;

    // 16.11.1 TCCR1A – Timer/Counter1 Control Register A
    TCCR1A =
        0
        | (1 << COM1A1) | (1 << COM1A0)  // Set OC1A on Compare Match, clear OC1A at BOTTOM (inverting mode)
        | (1 << COM1B1) | (1 << COM1B0)  // Set OC1B on Compare Match, clear OC1B at BOTTOM (inverting mode)
        | (1 << WGM11) | (0 << WGM10)    // Fast PWM mode 14 (TOP = ICR1), part 1/2
        ;

    // 16.11.2 TCCR1B – Timer/Counter1 Control Register B
    TCCR1B =
        0
        | (1 << WGM13) | (1 << WGM12)    // Fast PWM mode 14 (TOP = ICR1), part 2/2
        ;

    // IMPORTANT NOTE ABOUT ORDER OF INITIALIZATION:
    //   "The ICR1 Register can only be written when using a Waveform
    //   Generation mode that utilizes the ICR1 Register for defining
    //   the counter’s TOP value. In these cases the Waveform
    //   Generation mode (WGM13:0) bits must be set before the TOP
    //   value can be written to the ICR1 Register."
    // Thus initializing OCR1 before TCCR1A and TCCR1B has been
    // configured with Fast PWM mode 14 is wrong.

    // ICR1 is captured from Analog Comparator Output
    ICR1 = 0x3FF;

    // IMPORTANT NOTE ABOUT ORDER OF INITIALIZATION:
    //   "The OCR1x Register is double buffered when using any of the
    //   twelve Pulse Width Modulation (PWM) modes. For the Normal
    //   and Clear Timer on Compare (CTC) modes of operation, the
    //   double buffering is disabled."
    // If initializing OCR1A before configuring TCCR1A and TCCR1B to
    // a PWM mode the value is written to the non-buffered OCR1A
    // register and the buffered OCR1A register contains some "random",
    // unused garbage value. When later changing to PWM the buffered
    // register will be enabled, and its existing garbage value will
    // be used.
    // Thus initializing OCR1A/OCR1B before TCCR1A and TCCR1B has
    // been configured with Fast PWM is wrong.

    // Init Register
    OCR1A = 0x0;

    // Init Register
    OCR1B = 0x0;

    // 14.4.3 DDRB – The Port B Data Direction Register
    DDRB =
        0
        | (1 << DDB1) // PB1 (aka OC1A) as output - pin 9 on Arduino Uno
        ;

    // Start the timer with /8 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);

    SREG = sreg;
}

void loop()
{
}

ISR(ANALOG_COMP_vect){
//      ADCSRA |= (0 << ADEN);  // disable ADC
      ACSR |= (0 << ACD); //disable Comparator
//      ACSR |= (0 << ACIE); //disable Comparator Interrupt

      ADMUX =
        0
        | (0 << MUX0) | (1 << MUX1) | (1 << MUX2) | (0 << MUX3)  // Select ADC6 ADC input
        | (1 << REFS1) | (1 << REFS0)  // Refs 1.1v     
        ;  

      ADCSRA |= (1 << ADEN);  // Enable ADC
//    

}

ISR(ADC_vect){
      // update OCR1A
      uint8_t atmp = ADCL;
      OCR1AH = ADCH;
      OCR1AL = atmp;
      
      ADCSRA |= (0 << ADEN);  // Disable ADC
      ADMUX =
        0
        | (1 << MUX0) | (1 << MUX1) | (1 << MUX2) // Select ADC7 Negative input 
        | (1 << REFS1) | (1 << REFS0)  // Analog Comparator Input Capture Enable      
        ;
      ACSR |= (1 << ACD); //enable Comparator
}