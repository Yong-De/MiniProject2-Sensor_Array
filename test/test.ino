#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define THRESHOLD 10
volatile bool buttonChanged = false;
volatile long currTime, lastTime;

// Corresponds to digital pin 3 on the mega
ISR(INT5_vect) {
  buttonChanged = true;
}


void setup() {
    Serial.begin(9600);
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.

    // DDRD |= (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6);
    // PORTD |= (1 << PD3);
    // PORTD &= ~(1 << PD4);

    DDRE &= ~(1 << PE5);
    EIMSK |= 0b00100000;
    EICRB |= 0b00000100;

    sei();
}

void loop() {
    if (buttonChanged) {
        buttonChanged = false;
        Serial.println("YAY IT WORKED!");
        // currTime = millis();

        // if (currTime - lastTime > THRESHOLD) {
        //     bool pinHigh = PINE & (1 << PE5);

        //     if (buttonState == STATE_RUNNING && pinHigh) {
        //         buttonState = STATE_STOPPED;
        //         stateChanged = true;
        //     }
        //     else if (buttonState == STATE_STOPPED && !pinHigh) {
        //         buttonState = STATE_RUNNING;
        //         stateChanged = true;
        //     }

        //     lastTime = currTime;
        // }
    } else {
      Serial.println("Reading...");
      delay(1000);
    }
}

