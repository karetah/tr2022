#include <inttypes.h>

#include "Arduino.h"
#include <avr/io.h>

#include <Wire.h>
//SCL to A5, SDA to A4
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27); 

// http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
// https://content.arduino.cc/assets/Pinout-UNOrev3_latest.pdf

uint8_t mux;


int value;
int mapped;
unsigned long now;
unsigned long lastSecond = 0;


void setup()
{

    uint8_t sreg = SREG;
    cli();

    // Init Trigger Source for ADC
    // Pin Change Interrupt Control Register
    PCICR = 
          0
          | (1 << PCIE2)  // Pin Change Interrupt Enable 2
          ;  
    // Pin Change Mask Register 2
    PCMSK2 = 
          0
          | (1 << PCINT18)  // PD2 (D2)
          ;  
    // External Interrupt Mask Register
   EIMSK = 
          0
          | (1 << INT0)  // External Interrupt Request 0 Enable
          ;  
    // External Interrupt Control Register A
   EICRA = 
          0
          | (1 << ISC00) | (1 << ISC01)  // Any logical change on INT0 generates an interrupt request.
          ;        


    // Setup ADC
    // ADC Control and Status Register A
   ADCSRA =
          0
          | (1 << ADEN) | (1 << ADATE) // Enable ADC, ADC Auto Trigger Enable
          | (1 << ADIE)  // ADC Interrupt Enable, 
          | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) // Division Factor 128
          ;  
   ADMUX =
        0
        | (1 << REFS1) | (1 << REFS0)  // Refs 1.1v         
        ;  
   // ADMUX = 199; // 0xC7 // ADC7 // set TOP
   // ADMUX = 198; // 0xC6 // ADC6 // set OCR1A
        
   ADCSRB =
        0
        | (0 << ACME)   // Analog Comparator Multiplexer Enable
        | (0 << ADTS0) | (1 << ADTS1) | (0 << ADTS2) // ADC Auto Trigger Source : External Interrupt Request 0 
        ;

    // Setup Timer1
    // Stop timer before configuring
    TCCR1B = 0;
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
    // ICR1 default value 10bit
    ICR1 = 0x3FF;
    // Init OCR1A default 0
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
    sei();


  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(0x27);
  int error = Wire.endTransmission();
    if (error == 0) { lcd.begin(16, 2);} // initialize the lcd
    else {
    }
  lcd.setBacklight(127);
  lcd.home();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TRACK");

}

void bLNK () {
      if (mapped == HIGH) {digitalWrite(LED_BUILTIN, HIGH);mapped = LOW;}
      else if (mapped == LOW) {digitalWrite(LED_BUILTIN, LOW);mapped = HIGH;}
}

void loop()
{
    now = millis();
    if (now - lastSecond > 1000) 
    {
      ADCSRA |= (1 << ADSC);
      lcd.setCursor(0, 0);   
      lcd.print(value);

      bLNK();
      lastSecond = now;
    }
}
// Catch PD0 signal
ISR(PCINT2_vect)
      {
// EIFR |= (0 << INTF0); //enable if not catch    
      }
// Catch ADC event
ISR(ADC_vect){
      // update OCR1A
      if (ADMUX == 0xC7) {
        uint8_t ADCLow = ADCL;
        uint8_t ADCHigh = ADCH;
        uint16_t val = ADCLow + (ADCHigh << 8);
        //ICR1 = val;
        ADMUX |= 0xC6;
      }
      else if (ADMUX == 0xC6){
        uint16_t val = ADCL + (ADCH << 8);
        OCR1 = val;
        ADMUX |= 0xC7;        
      }
}
