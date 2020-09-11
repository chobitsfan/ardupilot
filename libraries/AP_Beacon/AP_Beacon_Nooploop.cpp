/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Beacon_Nooploop.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Beacon_Nooploop::AP_Beacon_Nooploop(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
    }
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Nooploop::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - _last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_Nooploop::update(void)
{
    // return immediately if not serial port
    if (_uart == nullptr) {
        return;
    }

    // check uart for any incoming messages
    uint32_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        uint8_t b = _uart->read();
        MsgType type = parse_byte(b);
        if (type == MsgType::NODE_FRAME2) {
            //hal.console->printf("nooploop nf2\n");
            if (_anchor_pos_avail) {
                parse_node_frame2();
            } else {
                request_setting();
            }
        } else if (type == MsgType::SETTING_FRAME0) {
            //hal.console->printf("nooploop sf0\n");
            parse_setting_frame0();
        } 
    }
}

void AP_Beacon_Nooploop::request_setting()
{
    if (_uart->txspace() >= 128) {
        uint8_t buf[128] = {0x54, 0, 1};
        buf[127] = 0x55;
        _uart->write(buf, 128);
    }
}

// process one byte received on serial port
// message is stored in _msgbuf
AP_Beacon_Nooploop::MsgType AP_Beacon_Nooploop::parse_byte(uint8_t b)
{
    // process byte depending upon current state
    switch (_state) {

    case ParseState::HEADER:
        if (b == NOOPLOOP_HEADER) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc_expected = b;
            _state = ParseState::H55_FUNCTION_MARK;
        } else if (b == NOOPLOOP_HEADER2) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc_expected = b;
            _state = ParseState::H54_FUNCTION_MARK;
        }
        break;

    case ParseState::H55_FUNCTION_MARK:
        if (b == NOOPLOOP_FUNCTION_MARK_NODE_FRAME2) {
            _msgbuf[1] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::LEN_L;
        } else {
            _state = ParseState::HEADER;
        }
        break;

    case ParseState::H54_FUNCTION_MARK:
        if (b == NOOPLOOP_FUNCTION_MARK_SETTING_FRAME0) {
            _msgbuf[1] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::SF0_PAYLOAD;
        } else {
            _state = ParseState::HEADER;
        }
        break;

    case ParseState::LEN_L:
        _msgbuf[2] = b;
        _msg_len++;
        _crc_expected += b;
        _state = ParseState::LEN_H;
        break;

    case ParseState::LEN_H:
        // extract and sanity check frame length
        _frame_len = (uint16_t)b << 8 | _msgbuf[2];
        if (_frame_len > NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX) {
            _state = ParseState::HEADER;
            hal.console->printf("nooploop wrong len:%d\n", _frame_len);
        } else {
            _msgbuf[3] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::NF2_PAYLOAD;
        }
        break;

    case ParseState::NF2_PAYLOAD:
        // add byte to buffer if there is room
        if (_msg_len < NOOPLOOP_MSG_BUF_MAX) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len >= _frame_len) {
            _state = ParseState::HEADER;
            // check crc
            if (b == _crc_expected) {
                return MsgType::NODE_FRAME2;
            } else {
                return MsgType::INVALID;
                hal.console->printf("nooploop bad crc\n");
            }
        } else {
            _crc_expected += b;
        }
        break;

    case ParseState::SF0_PAYLOAD:
        // add byte to buffer if there is room
        if (_msg_len < NOOPLOOP_MSG_BUF_MAX) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len >= 128) {
            _state = ParseState::HEADER;
            // check crc
            if (b == _crc_expected) {
                return MsgType::SETTING_FRAME0;
            } else {
                return MsgType::INVALID;
                hal.console->printf("nooploop bad crc\n");
            }
        } else {
            _crc_expected += b;
        }
        break;
    }

    return MsgType::INVALID;
}

void AP_Beacon_Nooploop::parse_node_frame2()
{
    // a message has been received
    _last_update_ms = AP_HAL::millis();

    // estimated precision for x,y position in meters
    const float precision_x = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_X] * 0.01;
    const float precision_y = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_Y] * 0.01;
    const float pos_err = sqrtf(sq(precision_x)+sq(precision_y));

    // x,y,z position in m*1000 in ENU frame
    const int32_t pos_x = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX] << 8) >> 8;
    const int32_t pos_y = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY] << 8) >> 8;
    const int32_t pos_z = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ] << 8) >> 8;

    // position scaled to meters and changed to NED
    const Vector3f pos_m {pos_y * 0.001f, pos_x * 0.001f, -pos_z * 0.001f};

    set_vehicle_position(pos_m, pos_err);

    const uint8_t valid_nodes = _msgbuf[NOOPLOOP_NODE_FRAME2_VALID_NODES];
    for (int i = 0; i < valid_nodes; i++) {
        int offset = NOOPLOOP_NODE_FRAME2_NODE_BLOCK + i * 13;
        uint8_t id = _msgbuf[offset+1];
        if (id == 2) id = 3; else if (id == 3) id = 2;
        const int32_t dist = ((int32_t)_msgbuf[offset+2+2] << 24 | (int32_t)_msgbuf[offset+2+1] << 16 | (int32_t)_msgbuf[offset+2] << 8) >> 8;
        set_beacon_distance(id, dist * 0.001f);
        //hal.console->printf("nooploop beacon %d %d\n", id, (int)dist);
    }
}

void AP_Beacon_Nooploop::parse_setting_frame0()
{
    for (int i = 0; i < 4; i++) {
        int offset = NOOPLOOP_SETTING_FRAME0_A0 + i * 9;

        // x,y,z position in m*1000 in ENU frame
        const int32_t pos_x = ((int32_t)_msgbuf[offset+2] << 24 | (int32_t)_msgbuf[offset+1] << 16 | (int32_t)_msgbuf[offset] << 8) >> 8;
        if (pos_x == -8388000) return;
        offset+=3;
        const int32_t pos_y = ((int32_t)_msgbuf[offset+2] << 24 | (int32_t)_msgbuf[offset+1] << 16 | (int32_t)_msgbuf[offset] << 8) >> 8;
        if (pos_y == -8388000) return;
        offset+=3;
        const int32_t pos_z = ((int32_t)_msgbuf[offset+2] << 24 | (int32_t)_msgbuf[offset+1] << 16 | (int32_t)_msgbuf[offset] << 8) >> 8;
        if (pos_z == -8388000) return;

        // position scaled to meters and changed to NED
        const Vector3f pos_m {pos_y * 0.001f, pos_x * 0.001f, -pos_z * 0.001f};

        int id = i;
        if (id == 2) id = 3; else if (id == 3) id = 2;
        set_beacon_position(id, pos_m); 
        //hal.console->printf("nooploop setting %d %d %d\n", id, (int)pos_x, (int)pos_y);
    }
    _anchor_pos_avail = true;
}
