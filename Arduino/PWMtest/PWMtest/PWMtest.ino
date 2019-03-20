// A sketch that creates an 4MHz, 50% duty cycle PWM and a 125KHz,
// 6bit resolution PWM with varying duty cycle (changes every 5μs
// or about every period.

// https://withinspecifications.30ohm.com/2014/02/20/Fast-PWM-on-AtMega328/

#include <avr/io.h>
#include <util/delay.h>

#define LED 4
#define PWMPIN 5

int main(void)
{
  // pinMode(3, OUTPUT); // output pin for OCR2B (Hall on OpenTag)
  pinMode(PWMPIN, OUTPUT); // output pin for OCR0B
  pinMode(LED, OUTPUT);

//  // Set up the 125KHz output
//  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
//  TCCR2B = _BV(WGM22) | _BV(CS20);
//  OCR2A = 200;  // 63 = 125 kHz
//  OCR2B = 100;

  // Set up the pin 5 output
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(WGM02) | _BV(CS00);
  OCR0A = 200;
  OCR0B = 100;

  while (1) {
    delay(200);
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
  }
}
