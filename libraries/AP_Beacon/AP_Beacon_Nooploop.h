#pragma once

#include "AP_Beacon_Backend.h"

#define NOOPLOOP_MSG_BUF_MAX      256

#define NOOPLOOP_HEADER                     0x55    // message header
#define NOOPLOOP_FUNCTION_MARK_NODE_FRAME2  4
#define NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX   4096    // frames should be less than 4k bytes
#define NOOPLOOP_NODE_FRAME2_SYSTIME        6       // start of 4 bytes holding system time in ms
#define NOOPLOOP_NODE_FRAME2_PRECISION_X    10      // start of 1 byte holding precision in m*100 in x axis
#define NOOPLOOP_NODE_FRAME2_PRECISION_Y    11      // start of 1 byte holding precision in m*100 in y axis
#define NOOPLOOP_NODE_FRAME2_PRECISION_Z    12      // start of 1 byte holding precision in m*100 in y axis
#define NOOPLOOP_NODE_FRAME2_POSX           13      // start of 3 bytes holding x position in m*1000
#define NOOPLOOP_NODE_FRAME2_POSY           16      // start of 3 bytes holding y position in m*1000
#define NOOPLOOP_NODE_FRAME2_POSZ           19      // start of 3 bytes holding z position in m*1000
#define NOOPLOOP_NODE_FRAME2_VALID_NODES    118
#define NOOPLOOP_NODE_FRAME2_NODE_BLOCK     119

#define NOOPLOOP_HEADER2                    0x54    // message header
#define NOOPLOOP_FUNCTION_MARK_SETTING_FRAME0  0
#define NOOPLOOP_SETTING_FRAME0_A0          37

class AP_Beacon_Nooploop : public AP_Beacon_Backend
{

public:
    AP_Beacon_Nooploop(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update the state of the sensor
    void update() override;

private:
    enum class MsgType : uint8_t {
        INVALID = 0,
        NODE_FRAME2,
        SETTING_FRAME0
    };

    // process one byte received on serial port
    // message is stored in _msgbuf
    MsgType parse_byte(uint8_t b);

    void request_setting();
    void parse_node_frame2();
    void parse_setting_frame0();

    // enum of valid function marks
    enum class FunctionMark : uint8_t {
        ANCHOR_FRAME0 = 0x00,
        SETTING_FRAME0 = 0x00,
        TAG_FRAME0 = 0x01,
        NODE_FRAME0 = 0x02,
        NODE_FRAME1 = 0x03,
        NODE_FRAME2 = 0x04,
        NODE_FRAME3 = 0x05
    };

    enum class ParseState : uint8_t {
        HEADER = 0,         // waiting for header
        H55_FUNCTION_MARK,  // waiting for function mark
        H54_FUNCTION_MARK,  // waiting for function mark
        LEN_L,              // waiting for low byte of length
        LEN_H,              // waiting for high byte of length
        NF2_PAYLOAD,        // receiving payload bytes
        SF0_PAYLOAD,        // receiving payload bytes
    } _state = ParseState::HEADER;

    // members
    AP_HAL::UARTDriver *_uart = nullptr;        // pointer to uart configured for use with nooploop
    uint8_t _msgbuf[NOOPLOOP_MSG_BUF_MAX];      // buffer to hold most recent message from tag
    uint16_t _msg_len;                          // number of bytes received from the current message (may be larger than size of _msgbuf)
    uint16_t _frame_len;                        // message supplied frame length
    uint8_t _crc_expected;                      // calculated crc which is compared against actual received crc
    uint32_t _last_update_ms = 0;
    bool _anchor_pos_avail = false;
};
