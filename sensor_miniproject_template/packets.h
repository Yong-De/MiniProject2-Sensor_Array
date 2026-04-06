/*
 * packets.h
 * Studio 13: Sensor Mini-Project
 *
 * TPacket protocol: enums, struct, and framing constants.
 * This file must be kept in sync with the constants in pi_sensor.py.
 */

#pragma once

#include <stdint.h>

// =============================================================
// TPacket protocol
// =============================================================

typedef enum {
    PACKET_TYPE_COMMAND  = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
    // Estop Command
    COMMAND_ESTOP       = 0,
    // TODO (Activity 2): add your own command type for the color sensor
    // Color Command
    COMMAND_COLOR       = 2,
    // Movement Commands
    COMMAND_FORWARD     = 3,
    COMMAND_LEFT        = 4,
    COMMAND_BACKWARD    = 5,
    COMMAND_RIGHT       = 6,
    COMMAND_STOP        = 7,
    // Speed Commands
    COMMAND_INCREASE    = 8,
    COMMAND_DECREASE    = 9,
    // Arm Commands
    COMMAND_BASE        = 10,
    COMMAND_SHOULDER    = 11,
    COMMAND_ELBOW       = 12,
    COMMAND_GRIPPER     = 13,
    COMMAND_VELOCITY    = 14,
    COMMAND_HOME        = 15,
} TCommandType;

typedef enum {
    RESP_OK         = 0,
    RESP_STATUS     = 1,
    // TODO (Activity 2): add your own response type for the color sensor
    RESP_COLOR      = 2,
    // Movement Response
    RESP_FORWARD    = 3,
    RESP_LEFT       = 4,
    RESP_BACKWARD   = 5,
    RESP_RIGHT      = 6,
    RESP_STOP       = 7,
    // Speed Response
    RESP_INCREASE   = 8,
    RESP_DECREASE   = 9,
    //Arm Response
    RESP_BASE       = 10,
    RESP_SHOULDER   = 11,
    RESP_ELBOW      = 12,
    RESP_GRIPPER    = 13,
    RESP_VELOCITY   = 14,
    RESP_HOME       = 15,
} TResponseType;

typedef enum {
    STATE_RUNNING = 0,
    STATE_STOPPED = 1,
} TState;

typedef struct {
    uint8_t  packetType;
    uint8_t  command;
    uint8_t  dummy[2];
    char     data[32];
    uint32_t params[16];
} TPacket;

// =============================================================
// Framing constants
// =============================================================

#define MAGIC_HI        0xDE
#define MAGIC_LO        0xAD
#define TPACKET_SIZE    ((uint8_t)sizeof(TPacket))   // 100 bytes
#define FRAME_SIZE      (2 + TPACKET_SIZE + 1)       // 103 bytes
