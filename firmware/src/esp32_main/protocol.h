#pragma once
// ============================================================================
//  Pharma Rover framed binary protocol (shared by server and ESP32)
//
//  Frame:  0xAA | LEN | CMD | PAYLOAD (LEN-1 bytes) | CHK | 0x55
//          LEN = 1 + payload length
//          CHK = LEN ^ CMD ^ every payload byte
//  Multi-byte numbers are big-endian.
//
//  TCP 5000 = base (car)    TCP 5001 = arm    TCP 8080 = text lines to the TFT
// ============================================================================
#include <stdint.h>

#define PROTO_START  0xAA
#define PROTO_END    0x55
#define PROTO_MAX_PAYLOAD 64

// ---------------- Base (car) commands  -> TCP 5000 ----------------
#define CMD_DRIVE_FORWARD   0x10  // dist_mm u16, speed_pct u8
#define CMD_DRIVE_BACKWARD  0x11  // dist_mm u16, speed_pct u8
#define CMD_TURN_LEFT       0x12  // deg u16, speed_pct u8
#define CMD_TURN_RIGHT      0x13  // deg u16, speed_pct u8
#define CMD_STOP_MOTORS     0x14  // (empty)
#define CMD_DRIVE_VEL       0x16  // v_mm_s i16, w_mrad_s i16   (resend every <500 ms)
#define CMD_SET_OBSTACLE    0x17  // front_stop_mm u16, back_stop_mm u16 (0 = disable)

// ---------------- Arm commands  -> TCP 5001 ----------------
#define CMD_ARM_SET_JOINT   0x21  // joint u8, (deg+180)*10 u16
#define CMD_GRIPPER_OPEN    0x23  // (empty)
#define CMD_GRIPPER_CLOSE   0x24  // medicine_id u8, mission_token[8]
#define CMD_ARM_HOME        0x25  // (empty)
#define CMD_ARM_MOVE_JOINTS 0x26  // duration_ms u16, 5x (deg+180)*10 u16, gripper u16 (ignored)

// ---------------- Safety (both ports) ----------------
#define CMD_SET_ESTOP       0x30  // u8 1 = latch e-stop, 0 = release
#define CMD_ARM_AUTHORISE   0x31  // medicine_id u8, mission_token[8], lease_ms u16
#define CMD_ARM_DEAUTHORISE 0x32  // (empty)
#define CMD_ARM_ABORT       0x33  // (empty)  hold every servo where it is now
#define CMD_PING            0x3F  // (empty)  -> RESP_ACK

// ---------------- Replies from the ESP32 ----------------
#define RESP_ACK            0x01  // cmd u8
#define RESP_NACK           0x02  // cmd u8, err u8
#define RESP_CMD_DONE       0x03  // cmd u8
#define RESP_CMD_FAILED     0x04  // cmd u8, err u8
#define RESP_HEARTBEAT      0x05  // uptime_s u32
#define RESP_TELEMETRY      0x06  // see below
#define RESP_OBSTACLE       0x0E  // sensor u8 (0 F,1 B,2 L,3 R), dist_mm u16

// ---------------- Error codes ----------------
#define ERR_NONE               0x00
#define ERR_JOINT_OUT_OF_RANGE 0x07
#define ERR_BAD_PAYLOAD        0x08
#define ERR_NOT_AUTHORISED     0x09
#define ERR_STALL              0x0A
#define ERR_TORQUE             0x0B
#define ERR_KEEPOUT            0x0C  // obstacle in the requested direction
#define ERR_ESTOP_TRIGGERED    0x0D
#define ERR_BUSY               0x0F
#define ERR_PCA_OFFLINE        0x10
#define ERR_UNKNOWN_CMD        0x11

// ---------------- Telemetry layouts ----------------
// Base telemetry (type byte = 1), 17 bytes:
//   [0] 1 | [1..8] front, back, left, right distance mm (u16, 0xFFFF = nothing seen)
//   [9] flags: b0 moving, b1 estop, b2 obstacle stop active, b3 step active
//   [10..11] left motor pwm i16 | [12..13] right motor pwm i16 | [14..15] battery mV u16
//   [16] last obstacle sensor (0xFF none)
//
// Arm telemetry (type byte = 2), 36 bytes:
//   [0] 2
//   [1..12]  6x current servo angle  (deg*10 u16)  yaw, shoulder, elbow, wrist_pitch, wrist_roll, gripper
//   [13..22] 5x current joint angle  ((deg+180)*10 u16) yaw, q1, q2, q3, roll  (server convention)
//   [23..34] 6x target servo angle   (deg*10 u16)
//   [35] flags: b0 moving, b1 grab authorised, b2 estop, b3 PCA 0x40 ok, b4 PCA 0x41 ok, b5 gripper closed,
//               b6 box detected by the grip sensor, b7 grip sensor fitted
