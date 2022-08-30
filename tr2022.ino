#include <inttypes.h>

#include "Arduino.h"
#include <avr/io.h>

// http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
// https://content.arduino.cc/assets/Pinout-UNOrev3_latest.pdf


void setup()
{

    uint8_t sreg = SREG;
    cli();

    // Stop timer before configuring
    TCCR1B = 0;

    // ADC Control and Status Register A
   ADCSRA |= (0 << ADEN);  // Disable ADC

    // At setup, we enable the Multiplexer
    // 
    // ADC Multiplexer Selection Register
   ADMUX =
        0
        | (1 << MUX0) | (1 << MUX1) | (1 << MUX2) // Select ADC7 Negative input 
        | (1 << REFS1) | (1 << REFS0)  // Set the Internal 1.1V Voltage Reference     
        ;  
   ADCSRB =
        0
        | (1 << ACME)   // Analog Comparator Multiplexer Enable
        | (1 << ADTS0) | (0 << ADTS1) | (0 << ADTS2) // ADC Auto Trigger Source : Analog Comparator 
        ;
    // Analog Comparator Control and Status Register
   ACSR =
        0
        | (0 << ACD) | (1 << ACBG)  // Analog Comparator Disable, Analog Comparator Bandgap Select
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
        | (0 << COM1A1) | (1 << COM1A0)  // Toggle OC1A on Compare Match, OC1B disconnected (normal port operation)
        | (0 << COM1B1) | (1 << COM1B0)  // 
        | (1 << WGM11) | (0 << WGM10)    // Fast PWM mode 14 (TOP = ICR1), part 1/2
        ;

    // 16.11.2 TCCR1B – Timer/Counter1 Control Register B
    TCCR1B =
        0
        | (1 << WGM13) | (1 << WGM12)    // Fast PWM mode 14 (TOP = ICR1), part 2/2
        ;
    // ICR1 is captured from Analog Comparator Output, but we define it to 10bit
    ICR1 = 0x3FF;
    // Init OCR1A
    OCR1A = 0x0;
    // Init OCR1B
    OCR1B = 0x0;
    // 14.4.3 DDRB – The Port B Data Direction Register
    DDRB =
        0
        | (1 << DDB1) // PB1 (aka OC1A) as output - pin 9 on Arduino Uno
        ;
   // Timer/Counter1 Interrupt Mask Register
   TIMSK1 =
        0
        | (1 << ICIE1) | (1 << TOIE1)  // Input Capture Interrupt Enable, Overflow Interrupt Enable
        ;
    // Start the timer with /8 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    // restore SREG
    SREG = sreg;
}

void loop()
{
}
// Catch Comparator event
ISR(ANALOG_COMP_vect){
      ACSR =
        0
        | (1 << ACD)  // Disable Comparator 
        ;
      ADMUX =
        0
        | (0 << MUX0) | (1 << MUX1) | (1 << MUX2) | (0 << MUX3)  // Select ADC6 ADC input
        | (1 << REFS1) | (1 << REFS0)  // Refs 1.1v     
        ;  
      ADCSRA |= (1 << ADEN);  // Enable ADC
}
// Catch ADC event
ISR(ADC_vect){
      // update OCR1A
      uint8_t atmp = ADCL; // Load ADC Result's Low Byte to temp
      OCR1AH = ADCH; // Set the OCR1A High Byte to the High Byte in ADC Result 
      OCR1AL = atmp; // Set the OCR1A Low Byte to temp
      
      ADCSRA |= (0 << ADEN);  // Disable ADC
      ADMUX =
        0
        | (1 << MUX0) | (1 << MUX1) | (1 << MUX2) // Select ADC7 Comparator's negative Input
        | (1 << REFS1) | (1 << REFS0)  
        ;
      ACSR =
        0
        | (0 << ACD)  // enable Comparator
        ;
}
