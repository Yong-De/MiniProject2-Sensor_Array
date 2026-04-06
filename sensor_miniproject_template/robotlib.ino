
#include <AFMotor.h>
// Direction values
typedef enum dir
{
  STOP,
  GO,
  BACK,
  CCW,
  CW
} dir;

// Motor control
#define FRONT_LEFT   4 // M4 on the driver shield
#define FRONT_RIGHT  1 // M1 on the driver shield
#define BACK_LEFT    3 // M3 on the driver shield
#define BACK_RIGHT   2 // M2 on the driver shield

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(int speed, int direction)
{
  
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);

  switch(direction)
    {
      case BACK:
        motorFL.run(BACKWARD); // reversed
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(FORWARD); 
      break;
      case GO:
        motorFL.run(FORWARD); // reversed
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(BACKWARD); // reversed
      break;
      case CW:
        motorFL.run(FORWARD); // reversed
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); // reversed
      break;
      case CCW:
        motorFL.run(BACKWARD); // reversed
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); // reversed
      break;
      case STOP:
      default:
        motorFL.run(RELEASE);
        motorFR.run(RELEASE);
        motorBL.run(RELEASE);
        motorBR.run(RELEASE); 
    }
}

void forward(int speed)
{
  move(speed, GO);
}

void backward(int speed)
{
  move(speed, BACK);
}

void ccw(int speed)
{
  move(speed, CCW);
}

void cw(int speed)
{
  move(speed, CW);
}

void stop()
{
  move(0, STOP);
}

// EVERYTHING BELOW IS FOR THE ARM

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Global state variables
int basePos = 120;
int shoulderPos = 90;
int elbowPos = 120;
int gripperPos = 45;
int msPerDeg = 10;

// Volatile variables for the ISR
volatile int timer_ticks = 0;
volatile int tick_array[4]   = {183, 150, 183, 100};
volatile int target_ticks[4] = {183, 150, 183, 100};
unsigned long lastStepMillis = 0;

// --- Timer 1 Interrupt Service Routine ---
// This handles the PWM generation for the 4 servos on PORTK
ISR(TIMER3_COMPA_vect) {
    timer_ticks++;
    
    if (timer_ticks >= 2000) {
        timer_ticks = 0; // Reset every 20ms
    }

    // A8 - Base
    if (timer_ticks < tick_array[0]) PORTK |= (1 << PK0);
    else PORTK &= ~(1 << PK0);

    // A9 - Shoulder
    if (timer_ticks < tick_array[1]) PORTK |= (1 << PK1);
    else PORTK &= ~(1 << PK1);

    // A10 - Elbow
    if (timer_ticks < tick_array[2]) PORTK |= (1 << PK2);
    else PORTK &= ~(1 << PK2);

    // A11 - Gripper
    if (timer_ticks < tick_array[3]) PORTK |= (1 << PK3);
    else PORTK &= ~(1 << PK3);
}

/**
 * Initializes the Pins and Timer 1 for Servo Control
 * Call this in your main setup()
 */
void initRobotArm() {
    // Set Pins A8, A9, A10, A11 as Outputs
    DDRK |= (1 << PK0) | (1 << PK1) | (1 << PK2) | (1 << PK3);
    
    // Configure Timer 1 for CTC Mode
    TCNT3 = 0;
    OCR3A = 19;             // 10us interval at 16MHz with prescaler 8
    TCCR3A = 0;
    TCCR3B = (1 << WGM32) | (1 << CS31); // CTC mode, Prescaler = 8
    TIMSK3 |= (1 << OCIE3A);             // Enable Timer Compare Interrupt
}

/**
 * Converts degrees (0-180) to timer ticks (50-250)
 */
int degToTicks(int angle) {
    angle = constrain(angle, 0, 180);
    // 0.5ms (50 ticks) to 2.5ms (250 ticks)
    return (int)(((long)angle * 200) / 180) + 50;
}

/**
 * Moves a specific servo smoothly to a target degree
 * targetIdx: 0=Base, 1=Shoulder, 2=Elbow, 3=Gripper
 */
void setServo(int idx, int *curPos, int targetDeg) {
    if (idx < 0 || idx > 3) return;
    targetDeg = constrain(targetDeg, 0, 180);
    *curPos = targetDeg;                      // track logical position
    cli();
    target_ticks[idx] = degToTicks(targetDeg);
    sei();
}

void updateArm() {
    unsigned long now = millis();
    if (now - lastStepMillis < (unsigned long)msPerDeg) return;
    lastStepMillis = now;

    cli();
    for (int i = 0; i < 4; i++) {
        if      (tick_array[i] < target_ticks[i]) tick_array[i]++;
        else if (tick_array[i] > target_ticks[i]) tick_array[i]--;
    }
    sei();
}

// * Returns all servos to the 90-degree position

void homeAll() {
    setServo(0, &basePos, 120);
    setServo(1, &shoulderPos, 90);
    setServo(2, &elbowPos, 90);
    setServo(3, &gripperPos, 45);
}

