#include "Arduino.h"
#include <avr/interrupt.h>

#define BASE_PIN = (1 << 0)
#define SHOULDER_PIN = (1 << 1)
#define ELBOW_PIN = (1 << 2)
#define GRIPPER_PIN = (1 << 3)

int basePos = 90, shoulderPos = 90, elbowPos = 90, gripperPos = 90;
int msPerDeg = 10;

int baseVal, shoulderVal, elbowVal, gripperVal;
int targetVal;
bool baseOn = false, shoulderOn = false, elbowOn = false, gripperOn = false;

int parse3(const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}

void moveSmooth(Servo *sv, int *cur, int target) {
  if (!sv || !cur) return;

  target = constrain(target, 0, 180);
  int step = (target > *cur) ? 1 : -1;

  while (*cur != target) {
    *cur += step;
    sv->write(*cur);
    delay(msPerDeg);
  }
}

void homeAll() {
  PORTC |= BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN;
  _delay_ms(1.5);
}

void setupTimer() {
  // set to CTC mode
  TCCR1A = 0b01000000; 
  TCCR1B = 0b00001000; 
  // 1 tick = 4us
  OCR1A = 0;
  OCR1B = 5000;
  TIMSK1 |= 0b110; 
}

void startTimer () {
  TCCR1B |= 0b11;
}

ISR (TIMER1_COMPA_vect) {
  if (OCR1A == 0 && baseOn) {
    OCR1B = 250;
  }
  else if (OCR1A == 1250 && shoulderOn) {
    OCR1B = 1500;
  }
  else if (OCR1A == 2500 && elbowOn) {
    OCR1B = 2750;
  }
  else if (OCR1A == 3750 && gripperOn) {
    OCR1B = 4000;
  }
  OCR1A = (OCR1A + 1250) % 5000;
}


ISR (TIMER1_COMPB_vect) {
  if (OCR1B == 250) {
    PORTC |= BASE_PIN;
    targetVal = 250 + baseVal;
  }
  else if (OCR1B == 1500) {
    PORTC |= SHOULDER_PIN;
    targetVal = 1500 + shoulderVal;
  }
  else if (OCR1B == 2750) {
    PORTC |= ELBOW_PIN;
    targetVal = 2750 + elbowVal;
  }
  else if (OCR1B == 4000) {
    PORTC |= GRIPPER_PIN;
    OCR1B += gripperVal;
    targetVal = 4000 + gripperVal;
  }
  else if (OCR1B < targetVal){
    OCR1B += min(msPerDeg, baseVal);
    baseVal -= msPerDeg;
  }
  else {
    PORTC = 0;
    OCR1B = 5000;
    baseOn = shoulderOn = elbowOn = gripperOn = false;
    targetVal = -1;
  }
}

void setup() {
  cli();
  // Set A0-A4 as outputs;
  DDRC |= BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN;
  EICRA |= 0b00000010;
  // Setup timer
  setupTimer();
  // Start timer
  startTimer();
  sei();
}

void loop() {
  if (!Serial.available()) return;

  // Reads a string with the command until newline
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); // Remove any extra whitespace
  if (!cmd.length()) return; // didn't read anything

  // Handle the home command
  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  // All subsequent commands need to have 4 characters
  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  // c is now the command character
  char c = cmd.charAt(0);
  // val is the numerical value of the argument
  int val = parse3(&cmd.substring(1));
  if (val < 0) { 
    Serial.println("ERROR: Argument not valid");
    return;
  }

  // Vddd sets velocity as ms per degree
  if (c == 'V') {
    Serial.print("Setting velocity to ");
    Serial.println(val);
    msPerDeg = (val/180) * 2000 + 500;
    return;
  } else if (c == 'B') {
    Serial.print("Moving base to ");
    Serial.println(val);
    baseOn = true;
    baseVal = (val/180) * (2000) + 500;
    return;
  } else if (c == 'S') {
    Serial.print("Moving shoulder to ");
    Serial.println(val);
    shoulderOn = true;
    shoulderVal = (val/180) * (2000) + 500;
    return;
  } else if (c == 'E') {
    Serial.print("Moving elbow to ");
    Serial.println(val);
    elbowOn = true;
    elbowVal = (val/180) * (2000) + 500;
    return;
  } else if (c == 'G') {
    Serial.print("Moving gripper to ");
    Serial.println(val);
    gripperOn = true;
    gripperVal = (val/180) * (2000) + 500;
    return;
  } else {
    Serial.println("ERROR: Unknown command");
    return;
  }
}
