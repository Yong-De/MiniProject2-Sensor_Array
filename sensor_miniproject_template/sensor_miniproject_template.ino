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

#include packets.h
#include serial_driver.h
#include <avr/io.h>
#include <avr/interrupt.h>

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

#define THRESHOLD 10
volatile bool buttonChanged = false;
volatile long currTime, lastTime;

// Corresponds to digital pin 3 on the mega
ISR(INT5_vect) {
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

<<<<<<< Updated upstream

ISR(INT4_vect) {
  edgeCount++;
}

=======
>>>>>>> Stashed changes
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

ISR(INT4_vect) {
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
<<<<<<< Updated upstream
    PORTH |= (1 << PH5);
  } else {
    PORTH &= ~(1 << PH5);
  }

  if(s3_val) {
    PORTH |= (1 << PH6);
  } else {
    PORTH &= ~(1 << PH6);
=======
    PORTA |= (1 << PA2);
  } else {
    PORTA &= ~(1 << PA2);
  }

  if(s3_val) {
    PORTA |= (1 << PA3);
  } else {
    PORTA &= ~(1 << PA3);
>>>>>>> Stashed changes
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
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
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

                snprintf(colorPkt.data, sizeof(colorPkt.data), "Color: R=%lu Hz,G=%lu Hz,B=%lu Hz", red, green ,blue);

                sendFrame(&colorPkt);
                break;
            }
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

<<<<<<< Updated upstream
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5) | (1 << PH6);
    PORTH |= (1 << PH3);
    PORTH &= ~(1 << PH4);

    DDRE &= ~(1 << PE4);
    EIMSK |= (1 << INT4);
    EICRB |= (1 << ISC41);
    EICRB &= ~(1 << ISC40);

    DDRE &= ~(1 << PE5);
    EIMSK |= (1 << INT5);
    EICRB |= (1 << ISC51);
    EICRB &= ~(1 << ISC50);
=======
    DDRA  |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);
    PORTA |=  (1 << PA0);   // S0 HIGH
    PORTA &= ~(1 << PA1);   // S1 LOW

    DDRE &= ~(1 << PE5) & ~(1 << PE4);
    EIMSK |= 0b00110000;
    EICRB |= 0b00000111;
>>>>>>> Stashed changes

    sei();
}

void loop() {
    if (buttonChanged) {
      buttonChanged = false;
      currTime = millis();

        if (currTime - lastTime > THRESHOLD) {
            bool pinHigh = PINE & (1 << PE5);

            if (buttonState == STATE_RUNNING && pinHigh) {
                buttonState = STATE_STOPPED;
                stateChanged = true;
            }
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

