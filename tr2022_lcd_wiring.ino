
#include "Arduino.h"
#include <Wire.h>
//SCL to A5, SDA to A4
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27); 

uint8_t mux;


uint16_t value;
uint16_t top;
uint8_t error;
int mapped;
unsigned long now;
unsigned long lastSecond = 0;
uint8_t PWM_OUT = 9;
uint8_t ACCEL = A6;
uint8_t TOPMOTOR = A7;
uint8_t REV_CTRL = 4;
uint8_t REV_OUT = 3;
bool REV_ST;

uint16_t icr = 0x3ff;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  setupPWM16();
  analogReference(INTERNAL);
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
    if (error == 0) {
      lcd.begin(16, 2);
      lcd.setBacklight(127);
      lcd.home();
      lcd.clear();
      digitalWrite(LED_BUILTIN, HIGH);
      } // initialize the lcd
    else {
      digitalWrite(LED_BUILTIN, LOW);
    }

}

void setupPWM16() {
DDRB |= _BV(PB1) | _BV(PB2); //Set pins as outputs
TCCR1A = _BV(COM1A1) | _BV(COM1B1) //Non-Inv PWM
| _BV(WGM11); // Mode 14: Fast PWM, TOP=ICR1
TCCR1B = _BV(WGM13) | _BV(WGM12)
| _BV(CS10); // Prescaler 1
ICR1 = icr; // TOP counter value (Relieving OCR1A*)
}

//* 16-bit version of analogWrite(). Only for D9 & D10
void analogWrite16(uint8_t pin, uint16_t val)
{
switch (pin) {
case 9: OCR1A = val; break;
case 10: OCR1B = val; break;
}
}

void bLNK () {
      if (mapped == HIGH) {digitalWrite(LED_BUILTIN, HIGH);mapped = LOW;}
      else if (mapped == LOW) {digitalWrite(LED_BUILTIN, LOW);mapped = HIGH;}
}

void loop()
{
    now = millis();
    top = analogRead(TOPMOTOR);
    ICR1 = top;
    value = analogRead(ACCEL);
    analogWrite16(PWM_OUT,value);

    if (now - lastSecond > 1000) 
    {
        if (error == 0) {
      lcd.setCursor(0, 0);   
      lcd.print(value);
      lcd.setCursor(0, 1);   
      lcd.print(top);
        }


    //  bLNK();
      lastSecond = now;
    }
}
