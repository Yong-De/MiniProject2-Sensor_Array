/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"
#include <avr/io.h>
#include <avr/interrupt.h>

void usartInit(uint16_t ubrr) {
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr);
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */


static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */

#define THRESHOLD 5
volatile bool buttonChanged = false;
volatile long currTime, lastTime;
bool prevPinHigh = false; // because button starts NOT pressed
volatile bool buttonPressed = false;

// Corresponds to digital pin 3 on the mega
ISR(INT3_vect) {
  buttonChanged = true;
}

// =============================================================
// Color sensor (TCS3200)
// =============================================================

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 */

const int S0 = 3;
const int S1 = 4;
const int S2 = 5;
const int S3 = 6;
const int output_PIN = 2;


 /*
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 */


 /*
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 */

volatile unsigned long edgeCount = 0;
unsigned int redFreq = 0;
unsigned int blueFreq = 0;
unsigned int greenFreq = 0;

void countEdge() {
  edgeCount++;
}

ISR(INT2_vect) {
    edgeCount++;
}

 /* Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 */

static uint32_t measureChannel(uint8_t s2_val, uint8_t s3_val) {
  if(s2_val) {
    PORTA |= (1 << PA2);
  } else {
    PORTA &= ~(1 << PA2);
  }

  if(s3_val) {
    PORTA |= (1 << PA3);
  } else {
    PORTA &= ~(1 << PA3);
  }

  edgeCount = 0;

  //for(volatile unsigned long i = 0; i < 160000; i++);............................
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
        // Wait
  }

  cli();
  uint32_t count = edgeCount;
  sei();

  return count;
}


static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
       // Set S2/S3 for each channel, measure edge count, multiply by 10
       *r = measureChannel(0, 0) * 10;  // red,   in Hz
       *g = measureChannel(1, 1) * 10;  // green, in Hz
       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
}



// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
int modified_speed = 120;
uint16_t _speed = modified_speed;
int std_change = 15;
uint16_t change = 0;

static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "E-Stop is activated.", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
        case COMMAND_COLOR:
        {
            uint32_t red, green, blue;
            readColorChannels(&red, &green, &blue);

            TPacket colorPkt;
            memset(&colorPkt, 0, sizeof(colorPkt));
            colorPkt.packetType = PACKET_TYPE_RESPONSE;
            colorPkt.command    = RESP_COLOR;

            colorPkt.params[0] = (uint16_t)red;
            colorPkt.params[1] = (uint16_t)green;
            colorPkt.params[2] = (uint16_t)blue;

            snprintf(colorPkt.data, sizeof(colorPkt.data), "R=%lu Hz, G=%lu Hz, B=%lu Hz", red, green ,blue);

            sendFrame(&colorPkt);
            break;
        }
        // Movement Actions
        case COMMAND_FORWARD:
            forward(_speed);

            TPacket forwardPkt;
            memset(&forwardPkt, 0, sizeof(forwardPkt));
            forwardPkt.packetType = PACKET_TYPE_RESPONSE;
            forwardPkt.command    = RESP_FORWARD;

            forwardPkt.params[0] = (uint16_t) _speed;

            snprintf(forwardPkt.data, sizeof(forwardPkt.data), "Moving forward at %u", _speed);

            sendFrame(&forwardPkt);
            break;
        case COMMAND_LEFT:
            ccw(_speed);

            TPacket leftPkt;
            memset(&leftPkt, 0, sizeof(leftPkt));
            leftPkt.packetType = PACKET_TYPE_RESPONSE;
            leftPkt.command    = RESP_LEFT;

            leftPkt.params[0] = (uint16_t) _speed;

            snprintf(leftPkt.data, sizeof(leftPkt.data), "Turning left at %u", _speed);

            sendFrame(&leftPkt);
            break;
        case COMMAND_BACKWARD:
            backward(_speed);

            TPacket backwardPkt;
            memset(&backwardPkt, 0, sizeof(backwardPkt));
            backwardPkt.packetType = PACKET_TYPE_RESPONSE;
            backwardPkt.command    = RESP_BACKWARD;

            backwardPkt.params[0] = (uint16_t) _speed;

            snprintf(backwardPkt.data, sizeof(backwardPkt.data), "Moving backward at %u", _speed);

            sendFrame(&backwardPkt);
            break;
        case COMMAND_RIGHT:
            cw(_speed);

            TPacket rightPkt;
            memset(&rightPkt, 0, sizeof(rightPkt));
            rightPkt.packetType = PACKET_TYPE_RESPONSE;
            rightPkt.command    = RESP_RIGHT;

            rightPkt.params[0] = (uint16_t) _speed;

            snprintf(rightPkt.data, sizeof(rightPkt.data), "Turning right at %u", _speed);

            sendFrame(&rightPkt);
            break;
        case COMMAND_STOP:
            stop();

            TPacket stopPkt;
            memset(&stopPkt, 0, sizeof(stopPkt));
            stopPkt.packetType = PACKET_TYPE_RESPONSE;
            stopPkt.command    = RESP_STOP;

            strncpy(stopPkt.data, "Robot has stopped.", sizeof(stopPkt.data) - 1);
            stopPkt.data[sizeof(stopPkt.data) - 1] = '\0';

            sendFrame(&stopPkt);
            break;
            
        // Speed Actions
        case COMMAND_INCREASE:
            modified_speed += std_change;
            if (modified_speed > 255) {
                modified_speed = 255;
                printf("Speed cannot go above 255!");
            }
            change = modified_speed - _speed;
            _speed = modified_speed;

            TPacket increasePkt;
            memset(&increasePkt, 0, sizeof(increasePkt));
            increasePkt.packetType = PACKET_TYPE_RESPONSE;
            increasePkt.command    = RESP_INCREASE;

            increasePkt.params[0] = (uint16_t) _speed;

            snprintf(increasePkt.data, sizeof(increasePkt.data), "Increasing speed by %u", change);

            sendFrame(&increasePkt);
            break;
        case COMMAND_DECREASE:
            modified_speed -= std_change;
            if (modified_speed < 0) {
                modified_speed = 0;
                printf("Speed cannot go below 0!");
            }
            change = _speed - modified_speed;
            _speed = modified_speed;

            TPacket decreasePkt;
            memset(&decreasePkt, 0, sizeof(decreasePkt));
            decreasePkt.packetType = PACKET_TYPE_RESPONSE;
            decreasePkt.command    = RESP_DECREASE;

            decreasePkt.params[0] = (uint16_t) _speed;

            snprintf(decreasePkt.data, sizeof(decreasePkt.data), "Decreasing speed by %u", change);

            sendFrame(&decreasePkt);
            break;
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.

    DDRA  |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);
    PORTA |=  (1 << PA0);   // S0 HIGH
    PORTA &= ~(1 << PA1);   // S1 LOW

    DDRD &= ~((1 << PD2) | (1 << PD3));
    EIMSK |= 0b00001100;
    EICRA |= 0b01110000;

    sei();
}

void loop() {
    if (buttonChanged) {
      buttonChanged = false;

      currTime = millis();

        if (currTime - lastTime > THRESHOLD) {
            bool pinHigh = PIND & (1 << PD3);

            // ---- Detect button press (LOW -> HIGH) ----
            if (buttonState == STATE_RUNNING && pinHigh) {
                buttonState = STATE_STOPPED;
                stop();
                stateChanged = true;
            }

            // ---- Detect button release (HIGH -> LOW) ----
            else if (buttonState == STATE_STOPPED && !pinHigh) {
                buttonState = STATE_RUNNING;
                stateChanged = true;
            }

            lastTime = currTime;
        }
    }
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}

