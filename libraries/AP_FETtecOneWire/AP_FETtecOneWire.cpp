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

/* Protocol implementation was provided by FETtec */
/* Strongly modified by Amilcar Lucas, IAV GmbH */

#include <AP_Math/AP_Math.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_FETtecOneWire.h"
#if HAL_AP_FETTEC_ONEWIRE_ENABLED

extern const AP_HAL::HAL& hal;

static constexpr uint8_t FRAME_OVERHEAD = 6;
static constexpr uint8_t MAX_TRANSMIT_LENGTH = 4;
static constexpr uint8_t MAX_RECEIVE_LENGTH = 12;
static constexpr uint8_t MAX_RESPONSE_LENGTH = FRAME_OVERHEAD + MAX_RECEIVE_LENGTH;

const AP_Param::GroupInfo AP_FETtecOneWire::var_info[] {

    // @Param: MASK
    // @DisplayName: Servo Channel Output Bitmask
    // @Description: Servo channel mask specifying FETtec ESC output.  Set bits must be contiguous.  The SERVOn number is used as the FETtec ESC Id
    // @Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12,12:SERVO13,13:SERVO14,14:SERVO15,15:SERVO16
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_FETtecOneWire, _motor_mask, 0),

    // @Param: POLES
    // @DisplayName: Nr. electrical poles
    // @Description: Number of motor electrical poles
    // @Range: 2 50
    // @RebootRequired: False
    // @User: Standard
    AP_GROUPINFO("POLES", 2, AP_FETtecOneWire, _pole_count, 14),

    AP_GROUPEND
};

AP_FETtecOneWire *AP_FETtecOneWire::_singleton;

AP_FETtecOneWire::AP_FETtecOneWire()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_FETtecOneWire must be singleton");
    }
#endif
    _singleton = this;

    _response_length[uint8_t(msg_type::OK)] = 1;
    _response_length[uint8_t(msg_type::BL_START_FW)] = 0;        // Bootloader only
    _response_length[uint8_t(msg_type::REQ_TYPE)] = 1;
    _response_length[uint8_t(msg_type::REQ_SN)] = 12;
    _response_length[uint8_t(msg_type::REQ_SW_VER)] = 2;
    _response_length[uint8_t(msg_type::SET_FAST_COM_LENGTH)] = 1;
    _response_length[uint8_t(msg_type::SET_TLM_TYPE)] = 1;
}

/**
  initialize the serial port, scan the bus, setup the found ESCs

*/
void AP_FETtecOneWire::init()
{
    if (_uart == nullptr) {
        AP_SerialManager& serial_manager = AP::serialmanager();
        _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FETtecOneWire, 0);
        if (_uart) {
            _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            _uart->set_unbuffered_writes(true);
            _uart->set_blocking_writes(false);
            _uart->begin(500000U);
        } else {
            return; // no serial port available, so nothing to do here
        }
    }

    if (_scan.state != scan_state_t::DONE) {
        scan_escs();
        return;
    }

#if HAL_WITH_ESC_TELEM
    _update_rate_hz = AP::scheduler().get_loop_rate_hz();
    _crc_error_rate_factor = 100.0f/(float)_update_rate_hz; //to save the division in loop, precalculate by the motor loops 100%/400Hz
#endif

    // get the user-configured FETtec ESCs bitmask parameter
    // if the user changes this parameter, he will have to reboot
    _mask = uint16_t(_motor_mask.get());
    uint16_t smask = _mask; // shifted version of the _mask user parameter
    uint16_t mmask = 0;     // will be a copy of _mask with only the contiguous LSBs set

    static_assert(MOTOR_COUNT_MAX <= sizeof(_mask)*8, "_mask is too narrow for MOTOR_COUNT_MAX ESCs");
    static_assert(MOTOR_COUNT_MAX <= sizeof(smask)*8, "smask is too narrow for MOTOR_COUNT_MAX ESCs");
    static_assert(MOTOR_COUNT_MAX <= sizeof(mmask)*8, "mmask is too narrow for MOTOR_COUNT_MAX ESCs");

    _nr_escs_in_bitmask = 0;
    // count the number of contiguous user-configured FETtec ESCs in the bitmask parameter
    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        if ((smask & 0x01) == 0x00) {
            break;
        }
        smask >>= 1;
        _nr_escs_in_bitmask++;

        // build a copy of _mask with only the contiguous LSBs set
        mmask |= 0x1;
        mmask <<= 1;
    }

    // tell SRV_Channels about ESC capabilities
    SRV_Channels::set_digital_outputs(mmask, 0);

    _initialised = true;
}

/**
  check if the current configuration is OK
*/
void AP_FETtecOneWire::configuration_check()
{
    if (hal.util->get_soft_armed()) {
        return; // checks are only done when vehicle is disarmed, because the GCS_SEND_TEXT() function calls use lots of resources
    }

    const uint32_t now = AP_HAL::millis();
    if ((now - _last_config_check_ms < 3000) && _last_config_check_ms != 0) {  // only runs once every 3 seconds
        return;
    }
    _last_config_check_ms = now;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!_uart->is_dma_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW UART needs DMA");
        return;
    }
#endif

    const bool all_escs_found = _found_escs_count >= _nr_escs_in_bitmask;
    bool telem_rx_missing = false;
#if HAL_WITH_ESC_TELEM
    // TLM recovery, if e.g. a power loss occurred but FC is still powered by USB.
    const uint8_t num_active_escs = AP::esc_telem().get_num_active_escs(_mask);
    telem_rx_missing = (num_active_escs < _nr_escs_in_bitmask) && (_send_msg_count > 2 * MOTOR_COUNT_MAX);
#endif

    if (__builtin_popcount(_motor_mask.get()) != _nr_escs_in_bitmask) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: gap in SERVO_FTW_MASK paramter bits");
    }

    if (_fast_throttle.max_id - _fast_throttle.min_id > _found_escs_count - 1){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: gap in IDs found");
    }

    if (!all_escs_found || telem_rx_missing) {
        if (!all_escs_found) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: found only %i of %i ESCs", _found_escs_count, _nr_escs_in_bitmask);
        }
#if HAL_WITH_ESC_TELEM
        if (telem_rx_missing) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: got TLM from only %i of %i ESCs", num_active_escs, _nr_escs_in_bitmask);
        }
#endif
        // re-init the entire device driver
        _scan.state = scan_state_t::WAIT_FOR_BOOT;
        _initialised = false;
    }
}

/**
    transmits a FETtec OneWire frame to an ESC
    @param esc_id id of the ESC
    @param bytes 8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param length length of the bytes array (max 4)
    @return false if length is bigger than MAX_TRANSMIT_LENGTH, true on write success
*/
bool AP_FETtecOneWire::transmit(const uint8_t esc_id, const uint8_t* bytes, uint8_t length)
{
    /*
    a frame looks like:
    byte 1 = frame header (master is always 0x01)
    byte 2 = target ID (5bit)
    byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
    byte 5 = frame length over all bytes
    byte 6 - X = request type, followed by the payload
    byte X+1 = 8bit CRC
    */
    uint8_t transmit_arr[FRAME_OVERHEAD+MAX_TRANSMIT_LENGTH] = {0x01};
    transmit_arr[1] = esc_id+uint8_t(1);
    if (length > MAX_TRANSMIT_LENGTH) {
        return false; // no, do not send at all
    }
    transmit_arr[4] = length + FRAME_OVERHEAD;
    for (uint8_t i = 0; i < length; i++) {
        transmit_arr[i + 5] = bytes[i];
    }
    transmit_arr[length + 5] = crc8_dvb_update(0, transmit_arr, length + 5); // crc
    _uart->write(transmit_arr, length + FRAME_OVERHEAD);
    return true;
}

/**
    reads the FETtec OneWire answer frame of an ESC
    @param bytes 8 bit byte array, where the received answer gets stored in
    @param length the expected answer length
    @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
    @return receive_response enum
*/
AP_FETtecOneWire::receive_response AP_FETtecOneWire::receive(uint8_t* bytes, uint8_t length, return_type return_full_frame)
{
    /*
    a frame looks like:
    byte 1 = frame header (0x02 = bootloader, 0x03 = ESC firmware)
    byte 2 = sender ID (5bit)
    byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
    byte 5 = frame length over all bytes
    byte 6 - X = answer type, followed by the payload
    byte X+1 = 8bit CRC
    */

    if (length > MAX_RECEIVE_LENGTH) {
        return receive_response::REQ_OVERLENGTH;
    }
    // look for the real answer
    const uint8_t raw_length = FRAME_OVERHEAD + length;
    if (_uart->available() >= uint32_t(raw_length)) {
        // sync to frame start byte
        uint8_t test_frame_start;
        uint8_t head = 0; // nr of attempts at finding the frame start byte
        do {
            test_frame_start = _uart->read();
            if (test_frame_start == 0x02 || test_frame_start == 0x03) {
                break; // frame start byte detected, continue decoding the rest of the response
            }
            if (++head > 5) {
                // too many attempts at finding the frame start byte failed
                _uart->discard_input();
                return receive_response::NO_ANSWER_YET;
            }
        }
        while (_uart->available());
        // copy message
        if (_uart->available() >= uint32_t(raw_length-1u)) {
            uint8_t receive_buf[FRAME_OVERHEAD + MAX_RECEIVE_LENGTH];
            receive_buf[0] = test_frame_start;
            for (uint8_t i = 1; i < raw_length; i++) {
                receive_buf[i] = _uart->read();
            }
            // check CRC
            if (crc8_dvb_update(0, receive_buf, raw_length-1u) == receive_buf[raw_length-1u]) {
                if (return_full_frame == return_type::RESPONSE) {
                    for (uint8_t i = 0; i < length; i++) {
                        bytes[i] = receive_buf[5u + i];
                    }
                } else {
                    for (uint8_t i = 0; i < raw_length; i++) {
                        bytes[i] = receive_buf[i];
                    }
                }
                return receive_response::ANSWER_VALID;
            } else {
                return receive_response::CRC_MISSMATCH;
            }
        } else {
            return receive_response::NO_ANSWER_YET;
        }
    } else {
        return receive_response::NO_ANSWER_YET;
    }
}

/**
    Resets a pending pull request
*/
void AP_FETtecOneWire::pull_reset()
{
    _pull_busy = false;
}

/**
    Pulls a complete request between flight controller and ESC
    @param esc_id  id of the ESC
    @param command 8bit array containing the command that should be send including the possible payload
    @param response 8bit array where the response will be stored in
    @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
    @param req_len transmit request length
    @return true if the request is completed, false if dont
*/
bool AP_FETtecOneWire::pull_command(const uint8_t esc_id, const uint8_t* command, uint8_t* response,
        return_type return_full_frame, const uint8_t req_len)
{
    if (!_pull_busy) {
        _pull_busy = transmit(esc_id, command, req_len);
    } else if (receive(response, _response_length[command[0]], return_full_frame) == receive_response::ANSWER_VALID) {
        _pull_busy = false;
        return true;
    }
    return false;
}

/**
    Scans for all ESCs in bus. Configures fast-throttle and telemetry for the ones found.
    Should be periodically called until _scan.state == scan_state_t::DONE
*/
void AP_FETtecOneWire::scan_escs()
{
    uint8_t response[MAX_RESPONSE_LENGTH];
    uint8_t request[2];

    const uint32_t now = AP_HAL::micros();
    if (now - _scan.last_us < (_scan.state == scan_state_t::WAIT_START_FW ? 5000U : 2000U)) {
        // the scan_escs() call period must be bigger than 2000 US,
        // as the bootloader has some message timing requirements. And we might be in bootloader
        return;
    }
    _scan.last_us = now;

    switch (_scan.state) {

    // initial state, wait for a ESC(s) cold-start
    case scan_state_t::WAIT_FOR_BOOT:
        _found_escs_count = 0;
        _scan.id = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
            _found_escs[i].active = false;
        }
        if (now > 500000U) {
            _scan.state++;
        }
        return;
        break;

    // is bootloader running?
    case scan_state_t::IN_BOOTLOADER:
        request[0] = uint8_t(msg_type::OK);
        if (pull_command(_scan.id, request, response, return_type::FULL_FRAME, 1)) {
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
            if (response[0] == 0x02) {
                _scan.state++; // is in bootloader, must start firmware
            } else {
                if (!_found_escs[_scan.id].active) {
                    _found_escs_count++; // found a new ESC not in bootloader
                }
                _found_escs[_scan.id].active = true;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
                _scan.state = scan_state_t::ESC_TYPE;
#else
                _scan.state = scan_state_t::NEXT_ID;
#endif
            }
            return;
        }
        break;

    // start the firmware
    case scan_state_t::START_FW:
        request[0] = uint8_t(msg_type::BL_START_FW);
        if (transmit(_scan.id, request, 1)) {
            _scan.state++;
        }
        return;
        break;

    // wait for the firmware to start
    case scan_state_t::WAIT_START_FW:
        _uart->discard_input(); // discard the answer to the previous transmit
        _scan.state = IN_BOOTLOADER;
        return;
        break;

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    // ask the ESC type
    case scan_state_t::ESC_TYPE:
        request[0] = uint8_t(msg_type::REQ_TYPE);
        if (pull_command(_scan.id, request, response, return_type::RESPONSE, 1)) {
            _found_escs[_scan.id].esc_type = response[0];
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
            _scan.state++;
            return;
        }
        break;

    // ask the software version
    case scan_state_t::SW_VER:
        request[0] = uint8_t(msg_type::REQ_SW_VER);
        if (pull_command(_scan.id, request, response, return_type::RESPONSE, 1)) {
            _found_escs[_scan.id].firmware_version = response[0];
            _found_escs[_scan.id].firmware_sub_version = response[1];
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
            _scan.state++;
            return;
        }
        break;

    // ask the serial number
    case scan_state_t::SN:
        request[0] = uint8_t(msg_type::REQ_SN);
        if (pull_command(_scan.id, request, response, return_type::RESPONSE, 1)) {
            for (uint8_t i = 0; i < SERIAL_NR_BITWIDTH; i++) {
                _found_escs[_scan.id].serial_number[i] = response[i];
            }
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
            _scan.state++;
            return;
        }
        break;
#endif

    // increment ESC ID and jump to IN_BOOTLOADER
    case scan_state_t::NEXT_ID:
        _scan.state = scan_state_t::IN_BOOTLOADER;
        _scan.id++; // re-run this state machine with the next ESC ID
        if (_scan.id == MOTOR_COUNT_MAX) {
            _scan.id = 0;
            if (_found_escs_count) {
                _scan.state = scan_state_t::CONFIG_FAST_THROTTLE;  // one or more ESCs found, scan is completed, now configure the ESCs found
                config_fast_throttle();
                _scan.id = _fast_throttle.min_id;
            }
        }
        return;
        break;

    // configure fast-throttle command header
    case scan_state_t::CONFIG_FAST_THROTTLE:
        if (pull_command(_scan.id, _fast_throttle.command, response, return_type::RESPONSE, 4)) {
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
#if HAL_WITH_ESC_TELEM
            _scan.state++;
#else
            _scan.state = CONFIG_NEXT_ACTIVE_ESC;
#endif
            return;
        }
        break;

#if HAL_WITH_ESC_TELEM
    // configure telemetry mode
    case scan_state_t::CONFIG_TLM:
        request[0] = uint8_t(msg_type::SET_TLM_TYPE);
        request[1] = 1; // Alternative telemetry mode -> a single ESC sends it's full telem (Temp, Volt, Current, ERPM, Consumption, CrcErrCount) in a single frame
        if (pull_command(_scan.id, request, response, return_type::RESPONSE, 2)) {
            _scan.rx_try_cnt = 0;
            _scan.trans_try_cnt = 0;
            _scan.state++;
            return;
        }
        break;
#endif

    // increment ESC ID and jump to CONFIG_FAST_THROTTLE
    case scan_state_t::CONFIG_NEXT_ACTIVE_ESC:
        do {
            _scan.id++;
        } while (_scan.id < MOTOR_COUNT_MAX && _found_escs[_scan.id].active == false);
        _scan.state = scan_state_t::CONFIG_FAST_THROTTLE;
        if (_scan.id == MOTOR_COUNT_MAX) {
            _scan.id = 0;
            _scan.state = scan_state_t::DONE;  // one or more ESCs found, scan is completed
        }
        return;
        break;
    }

    // it will try twice to read the response of a request
    if (_scan.rx_try_cnt > 1) {
        _scan.rx_try_cnt = 0;

        pull_reset(); // re-transmit the request, in the hope of getting a valid response later

        if (_scan.trans_try_cnt > 4) {
            // the request re-transmit failed multiple times, give-up on this ESC, goto the next one
            _scan.trans_try_cnt = 0;
            _scan.state = scan_state_t::NEXT_ID;
        } else {
            _scan.trans_try_cnt++;
        }
    } else {
        _scan.rx_try_cnt++;
    }
}

/**
    configure the fast-throttle command.
    Should be called once after scan_escs() is completted and before config_escs()
*/
void AP_FETtecOneWire::config_fast_throttle()
{
    _fast_throttle.min_id = MOTOR_COUNT_MAX;
    _fast_throttle.max_id = 0;
    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        if (_found_escs[i].active) {
            if (i < _fast_throttle.min_id) {
                _fast_throttle.min_id = i;
            }
            if (i > _fast_throttle.max_id) {
                _fast_throttle.max_id = i;
            }
        }
    }

    _fast_throttle.byte_count = 1;
    int16_t bit_count = 12 + (_found_escs_count * 11);
    _fast_throttle.bits_to_add_left = bit_count - 16;
    while (bit_count > 0) {
        _fast_throttle.byte_count++;
        bit_count -= 8;
    }
    _fast_throttle.command[0] = uint8_t(msg_type::SET_FAST_COM_LENGTH);
    _fast_throttle.command[1] = _fast_throttle.byte_count; // just for older ESC FW versions since 1.0 001 this byte is ignored as the ESC calculates it itself
    _fast_throttle.command[2] = _fast_throttle.min_id+1;   // min ESC id
    _fast_throttle.command[3] = _found_escs_count;         // count of ESCs that will get signals
}

#if HAL_WITH_ESC_TELEM
/**
    increment message packet count for every ESC
*/
void AP_FETtecOneWire::inc_send_msg_count()
{
    _send_msg_count++;
    if (_send_msg_count > 4 * _update_rate_hz) { // resets every four seconds
        _send_msg_count = 0; //reset the counter
        for (int i=0; i<_found_escs_count; i++) {
            _found_escs[i].error_count_since_overflow = _found_escs[i].error_count; //save the current ESC error state
        }
    }
}

/**
    calculates tx (outgoing packets) error-rate by converting the CRC error counts reported by the ESCs into percentage
    @param esc_id id of ESC, that the error is calculated for
    @param current_error_count the error count given by the esc
    @return the error in percent
*/
float AP_FETtecOneWire::calc_tx_crc_error_perc(const uint8_t esc_id, uint16_t current_error_count)
{
    _found_escs[esc_id].error_count = current_error_count; //Save the error count to the esc
    uint16_t corrected_error_count = (uint16_t)((uint16_t)_found_escs[esc_id].error_count - (uint16_t)_found_escs[esc_id].error_count_since_overflow); //calculates error difference since last overflow.
    return (float)corrected_error_count*_crc_error_rate_factor; //calculates percentage
}

/**
    if init is complete checks if the requested telemetry is available.
    @param t telemetry datastructure where the read telemetry will be stored in.
    @param centi_erpm 16bit centi-eRPM value returned from the ESC
    @param tx_err_count Ardupilot->ESC communication CRC error counter
    @param tlm_from_id receives the ID from the ESC that has respond with its telemetry
    @return receive_response enum
*/
AP_FETtecOneWire::receive_response AP_FETtecOneWire::decode_single_esc_telemetry(TelemetryData& t, int16_t& centi_erpm, uint16_t& tx_err_count, uint8_t &tlm_from_id)
{
    receive_response ret = receive_response::NO_ANSWER_YET;
    if (_found_escs_count > 0) {
        uint8_t telem[FRAME_OVERHEAD + 11];
        ret = receive((uint8_t *) telem, 11, return_type::FULL_FRAME);

        if (ret == receive_response::ANSWER_VALID) {
            if (telem[1] > 0 && telem[1] <=  MOTOR_COUNT_MAX) {
                tlm_from_id = (uint8_t)telem[1]-1;
            } else {
                tlm_from_id = 0;
            }

            t.temperature_cdeg = int16_t(telem[5+0] * 100);
            t.voltage = float((telem[5+1]<<8)|telem[5+2]) * 0.01f;
            t.current = float((telem[5+3]<<8)|telem[5+4]) * 0.01f;
            centi_erpm = (telem[5+5]<<8)|telem[5+6];
            t.consumption_mah = float((telem[5+7]<<8)|telem[5+8]);
            tx_err_count = (telem[5+9]<<8)|telem[5+10];
        }
    }
    return ret;
}
#endif

/**
    if init is complete sends a single fast-throttle frame containing the throttle for all found OneWire ESCs.
    @param motor_values a 16bit array containing the throttle values that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 0-999 reversed rotation
    @param tlm_request the ESC to request telemetry from (-1 for no telemetry, 0 for ESC1, 1 for ESC2, 2 for ESC3, ...)
*/
void AP_FETtecOneWire::escs_set_values(const uint16_t* motor_values, const int8_t tlm_request)
{
    if (_found_escs_count > 0) {
        // 8  bits - OneWire Header
        // 4  bits - telemetry request
        // 11 bits - throttle value per ESC
        // 8  bits - frame CRC
        // 7  dummy for rounding up the division by 8
        uint8_t fast_throttle_command[(8+4+(11*MOTOR_COUNT_MAX)+8+7)/8] = { 0 };
        uint8_t act_throttle_command = 0;

        // byte 1:
        // bit 0,1,2,3 = ESC ID, Bit 4 = MSB bit of first ESC (11bit) throttle value, bit 5,6,7 = frame header
        // so AAAABCCC
        // A = ID from the ESC telemetry is requested from. ESC ID == 0 means no request.
        // B = MSB from first throttle value
        // C = frame header
        static_assert(MOTOR_COUNT_MAX<=15, "OneWire supports at most 15 ESCs, because of the 4 bit limitation bellow");
        fast_throttle_command[0] = (tlm_request+1) << 4;
        fast_throttle_command[0] |= ((motor_values[act_throttle_command] >> 10) & 0x01) << 3;
        fast_throttle_command[0] |= 0x01;

        // byte 2:
        // AAABBBBB
        // A = next 3 bits from (11bit) throttle value
        // B = 5bit target ID
        fast_throttle_command[1] = (((motor_values[act_throttle_command] >> 7) & 0x07)) << 5;
        fast_throttle_command[1] |= 0x1F;      // All IDs

        // following bytes are the rest 7 bit of the first (11bit) throttle value,
        // and all bits from all other values, followed by the CRC byte
        uint8_t bits_left_from_command = 7;
        uint8_t act_byte = 2;
        uint8_t bits_from_byte_left = 8;
        int16_t bits_to_add_left = _fast_throttle.bits_to_add_left; // must be signed
        while (bits_to_add_left > 0) {
            if (bits_from_byte_left >= bits_left_from_command) {
                fast_throttle_command[act_byte] |=
                        (motor_values[act_throttle_command] & ((1 << bits_left_from_command) - 1))
                                << (bits_from_byte_left - bits_left_from_command);
                bits_to_add_left -= bits_left_from_command;
                bits_from_byte_left -= bits_left_from_command;
                act_throttle_command++;
                bits_left_from_command = 11;
                if (bits_to_add_left == 0) {
                    act_byte++;
                    bits_from_byte_left = 8;
                }
            } else {
                fast_throttle_command[act_byte] |=
                        (motor_values[act_throttle_command] >> (bits_left_from_command - bits_from_byte_left))
                                & ((1 << bits_from_byte_left) - 1);
                bits_to_add_left -= bits_from_byte_left;
                bits_left_from_command -= bits_from_byte_left;
                act_byte++;
                bits_from_byte_left = 8;
                if (bits_left_from_command == 0) {
                    act_throttle_command++;
                    bits_left_from_command = 11;
                }
            }
        }

        fast_throttle_command[_fast_throttle.byte_count - 1] =
            crc8_dvb_update(0, fast_throttle_command, _fast_throttle.byte_count - 1);

        // No command was yet sent, so no reply is expected and all information
        // on the receive buffer is either garbage or noise. Discard it
        _uart->discard_input();

        // send throttle commands to all configured ESCs in a single packet transfer
        _uart->write(fast_throttle_command, _fast_throttle.byte_count);
    }
}

/// periodically called from SRV_Channels::push()
void AP_FETtecOneWire::update()
{
    if (!_initialised) {
        init();
        return; // the rest of this function can only run after fully initted
    }

    // get ESC set points, stop as soon as there is a gap
    uint16_t motor_pwm[MOTOR_COUNT_MAX] {};
    for (uint8_t i = 0; i < _nr_escs_in_bitmask; i++) {
        const SRV_Channel* c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            break;
        }
        motor_pwm[i] = constrain_int16(c->get_output_pwm(), 1000, 2000);
    }

#if HAL_WITH_ESC_TELEM
    // receive and decode the telemetry data from one ESC
    // but do not process it any further to reduce timing jitter in the escs_set_values() function call
    TelemetryData t {};
    int16_t centi_erpm = 0;    // initialize to prevent false positive error: ‘centi_erpm’ may be used uninitialized in this function
    uint16_t tx_err_count = 0; // initialize to prevent false positive error: ‘tx_err_count’ may be used uninitialized in this function
    receive_response tlm_ok = receive_response::NO_ANSWER_YET; //decode_single_esc_telemetry returns 1 if telemetry is ok, 0 if its waiting and 2 if there is a crc mismatch.
    uint8_t tlm_from_id = 0;
    if (_requested_telemetry_from_esc != -1) {
        tlm_ok = decode_single_esc_telemetry(t, centi_erpm, tx_err_count, tlm_from_id);
    }
    if (_nr_escs_in_bitmask) {
        _requested_telemetry_from_esc++;
        if (_requested_telemetry_from_esc == _found_escs_count) { //if found esc number is reached restart request counter
            _requested_telemetry_from_esc = 0; // restart from the first ESC
        }
    }
#endif

    if (_nr_escs_in_bitmask) {
        // send motor setpoints to ESCs, and request for telemetry data
        escs_set_values(motor_pwm, _requested_telemetry_from_esc);

#if HAL_WITH_ESC_TELEM
        // now that escs_set_values() has been executed we can fully process the telemetry data from the ESC

        inc_send_msg_count(); // increment message packet count for every ESC

        if (_requested_telemetry_from_esc != -1 && tlm_ok == receive_response::ANSWER_VALID) { //only use telemetry if it is ok.
            if (_pole_count < 2) { // if user set parameter is invalid use 14 Poles
                _pole_count = 14;
            }
            const float tx_err_rate = calc_tx_crc_error_perc(tlm_from_id, tx_err_count);
            update_rpm(tlm_from_id, centi_erpm*100*2/_pole_count.get(), tx_err_rate);

            update_telem_data(tlm_from_id, t, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE|AP_ESC_Telem_Backend::TelemetryType::VOLTAGE|AP_ESC_Telem_Backend::TelemetryType::CURRENT|AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
        }
#endif
    }

    // Now that all real-time tasks above have been done, do some periodic checks.
    configuration_check();
}

#if HAL_AP_FETTEC_ESC_BEEP
/**
    makes all connected ESCs beep
    @param beep_frequency a 8 bit value from 0-255. higher make a higher beep
*/
void AP_FETtecOneWire::beep(const uint8_t beep_frequency)
{
    if (_found_escs_count > 0) {
        const uint8_t request[2] = {uint8_t(msg_type::BEEP), beep_frequency};
        const uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _fast_throttle.min_id; i <= _fast_throttle.max_id; i++) {
            transmit(i, request, sizeof(request));
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, sizeof(spacer));
        }
    }
}
#endif

#if HAL_AP_FETTEC_ESC_LIGHT
/**
    sets the racewire color for all ESCs
    @param r red brightness
    @param g green brightness
    @param b blue brightness
*/
void AP_FETtecOneWire::led_color(const uint8_t r, const uint8_t g, const uint8_t b)
{
    if (_found_escs_count > 0) {
        const uint8_t request[4] = {uint8_t(msg_type::SET_LED_TMP_COLOR), r, g, b};
        const uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _fast_throttle.min_id; i <= _fast_throttle.max_id; i++) {
            transmit(i, request, sizeof(request));
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, sizeof(spacer));
        }
    }
}
#endif

#endif  // HAL_AP_FETTEC_ONEWIRE_ENABLED
