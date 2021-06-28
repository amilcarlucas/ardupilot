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

static constexpr uint32_t DELAY_TIME_US = 700;
static constexpr uint32_t BAUDRATE = 500000;
static constexpr uint8_t ALL_ID = 0x1F;
static constexpr uint8_t FRAME_OVERHEAD = 6;
static constexpr uint8_t MAX_TRANSMIT_LENGTH = 4;
static constexpr uint8_t MAX_RECEIVE_LENGTH = 12;
static constexpr uint8_t MAX_RESPONSE_LENGTH = FRAME_OVERHEAD + MAX_RECEIVE_LENGTH;

const AP_Param::GroupInfo AP_FETtecOneWire::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of FETtec OneWire ESC protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
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

    _response_length[OW_OK] = 1;
    _response_length[OW_BL_START_FW] = 0;       // BL only
    _response_length[OW_REQ_TYPE] = 1;
    _response_length[OW_REQ_SN] = 12;
    _response_length[OW_REQ_SW_VER] = 2;
    _response_length[OW_SET_FAST_COM_LENGTH] = 1;
    _response_length[OW_SET_TLM_TYPE] = 1;
}

/**
  initialize the serial port, scan the OneWire bus, setup the found ESCs
*/
void AP_FETtecOneWire::init()
{
    if (!_uart_initialised) {
        AP_SerialManager& serial_manager = AP::serialmanager();
        _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FETtecOneWire, 0);
        if (_uart) {
            _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            _uart->set_unbuffered_writes(true);
            _uart->set_blocking_writes(false);
            _uart->begin(BAUDRATE);
            _uart_initialised = true;
        }
    }

    if (_uart == nullptr) {
        return; // no serial port available, so nothing to do here
    }

    const uint32_t now = AP_HAL::micros();
    if (now - _last_send_us < DELAY_TIME_US) {
        // scan_escs(), config_escs(), and set_full_telemetry() are to be called periodically multiple times
        // but the call period must to be bigger than DELAY_TIME_US,
        // as the bootloader has some message timing requirements.
        return;
    }
    _last_send_us = now;

    if (!scan_escs()) {
        // scan for all ESCs in OneWire bus
        return;
    }

    if (_config_active <= MOTOR_COUNT_MAX) {
        // check if in bootloader, start ESCs FW if they are and prepare fast-throttle command
        _config_active = config_escs();
        return;
    }

#if HAL_WITH_ESC_TELEM
    if (_set_full_telemetry_active <= MOTOR_COUNT_MAX) {
        // set telemetry to alternative mode (full telemetry from a single ESC, in a single response packet)
        _set_full_telemetry_active = set_full_telemetry(1);
        return;
    }
    _update_rate_hz = AP::scheduler().get_loop_rate_hz();
    _crc_error_rate_factor = 100.0f/(float)_update_rate_hz; //to save the division in loop, precalculate by the motor loops 100%/400Hz
#endif

    static_assert(MOTOR_COUNT_MAX <= 16, "16bit bitmasks are too narrow for MOTOR_COUNT_MAX ESCs");

    // get the user-configured FETtec ESCs bitmask parameter
    // if the user changes this parameter, he will have to reboot
    _mask = uint16_t(_motor_mask.get());

    // tell SRV_Channels about ESC capabilities
    SRV_Channels::set_digital_outputs(_mask, 0);

    uint16_t smask = _mask; // shifted version of the _mask user parameter
    _nr_escs_in_bitmask = 0;

    // count the number of user-configured FETtec ESCs in the bitmask parameter
    for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
        SRV_Channel* c = SRV_Channels::srv_channel(i);
        if (c == nullptr || (smask & 0x01) == 0x00) {
            break;
        }
        smask >>= 1;
        _nr_escs_in_bitmask++;
    }

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

    bool scan_missing = _id_count < _nr_escs_in_bitmask;
    bool telem_rx_missing = false;
#if HAL_WITH_ESC_TELEM
    // TLM recovery, if e.g. a power loss occurred but FC is still powered by USB.
    const uint8_t num_active_escs = AP::esc_telem().get_num_active_escs(_mask);
    telem_rx_missing = (num_active_escs < _nr_escs_in_bitmask) && (_send_msg_count > 2 * MOTOR_COUNT_MAX);
#endif

    if (_id_count == 0){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: No ESCs found");
    }

    if (_max_id - _min_id > _id_count - 1){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW: Gap in IDs found. Fix first.");
    }

    if (scan_missing || telem_rx_missing) {
        if (scan_missing) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW found only %i of %i ESCs", _id_count, _nr_escs_in_bitmask);
        }
#if HAL_WITH_ESC_TELEM
        if (telem_rx_missing) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FTW got TLM from only %i of %i ESCs", num_active_escs, _nr_escs_in_bitmask);
        }
        _set_full_telemetry_active = 1;
#endif
        _config_active = 0;
        _initialised = 0;
    }
}

/**
    transmits a FETtec OneWire frame to an ESC
    @param esc_id id of the ESC
    @param bytes 8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
    @param length length of the bytes array (max 4)
*/
void AP_FETtecOneWire::transmit(const uint8_t esc_id, const uint8_t* bytes, uint8_t length)
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
    uint8_t transmit_arr[FRAME_OVERHEAD+MAX_TRANSMIT_LENGTH] = {0x01, esc_id};
    if (length > MAX_TRANSMIT_LENGTH) {
        length = MAX_TRANSMIT_LENGTH;
    }
    transmit_arr[4] = length + FRAME_OVERHEAD;
    for (uint8_t i = 0; i < length; i++) {
        transmit_arr[i + 5] = bytes[i];
    }
    transmit_arr[length + 5] = crc8_dvb_update(0, transmit_arr, length + 5); // crc
    _uart->write(transmit_arr, length + FRAME_OVERHEAD);
}

/**
    reads the FETtec OneWire answer frame of an ESC
    @param bytes 8 bit byte array, where the received answer gets stored in
    @param length the expected answer length
    @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
    @return 2 on CRC error, 1 if the expected answer frame was there, 0 if dont
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
        length = MAX_RECEIVE_LENGTH;
    }
    // look for the real answer
    const uint32_t raw_length = FRAME_OVERHEAD + length;
    if (_uart->available() >= raw_length) {
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
        if (_uart->available() >= raw_length-1u) {
            uint8_t receive_buf[FRAME_OVERHEAD + MAX_RECEIVE_LENGTH];
            receive_buf[0] = test_frame_start;
            for (uint8_t i = 1; i < raw_length; i++) {
                receive_buf[i] = _uart->read();
            }
            // empty buffer, we are not expecting any more data now
            _uart->discard_input();
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
        _pull_busy = true;
        transmit(esc_id, command, req_len);
    } else if (receive(response, _response_length[command[0]], return_full_frame) == receive_response::ANSWER_VALID) {
        _pull_busy = false;
        return true;
    }
    return false;
}

/**
    scans for ESCs in bus.
    Should be periodically called until it returns true
    @return true when OneWire bus scan is complete
*/
bool AP_FETtecOneWire::scan_escs()
{
    uint8_t response[MAX_RESPONSE_LENGTH];
    uint8_t request[1];
    const uint32_t now = AP_HAL::micros();
    if (now - _scan.last_us < 2000) {
        // the call period must to be bigger than 2000 US,
        // as the bootloader has some message timing requirements.
        return false;
    }
    _scan.last_us = now;
    switch (_scan.state) {
    case 0:
        if (now > 500000) {
            _scan.state++;
        }
        return false;
        break;
    case 1:
        request[0] = OW_OK;
        if (pull_command(_scan.id+1, request, response, return_type::FULL_FRAME, 1)) {
            _found_escs[_scan.id].active = true;
            _found_escs[_scan.id].in_boot_loader = (response[0] == 0x02);
            _scan.rx_retry_cnt = 0;
            _scan.trans_retry_cnt = 0;
            _found_escs_count++;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
            _scan.state++;
#else
            _scan.state = 5;
#endif
            return false;
        }
        break;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    case 2:
        request[0] = OW_REQ_TYPE;
        if (pull_command(_scan.id+1, request, response, return_type::RESPONSE, 1)) {
            _found_escs[_scan.id].esc_type = response[0];
            _scan.rx_retry_cnt = 0;
            _scan.trans_retry_cnt = 0;
            _scan.state++;
            return false;
        }
        break;
    case 3:
        request[0] = OW_REQ_SW_VER;
        if (pull_command(_scan.id+1, request, response, return_type::RESPONSE, 1)) {
            _found_escs[_scan.id].firmware_version = response[0];
            _found_escs[_scan.id].firmware_sub_version = response[1];
            _scan.rx_retry_cnt = 0;
            _scan.trans_retry_cnt = 0;
            _scan.state++;
            return false;
        }
        break;
    case 4:
        request[0] = OW_REQ_SN;
        if (pull_command(_scan.id+1, request, response, return_type::RESPONSE, 1)) {
            for (uint8_t i = 0; i < SERIAL_NR_BITWIDTH; i++) {
                _found_escs[_scan.id].serialNumber[i] = response[i];
            }
            _scan.rx_retry_cnt = 0;
            _scan.trans_retry_cnt = 0;
            _scan.state++;
            return false;
        }
        break;
#endif
    case 5:
        _scan.state = 1;
        _scan.id++; // re-run this state machine with the next ESC ID
        if (_scan.id == MOTOR_COUNT_MAX) {
            _scan.id = 0;
            if (_found_escs_count) {
                return true;
            }
        }
        return false;
        break;
    }

    _scan.rx_retry_cnt++;
    // it will try twice to read the response of a request
    if (_scan.rx_retry_cnt > 2) {
        _scan.rx_retry_cnt = 0;

        pull_reset(); // re-transmit the request, in the hope of getting a valid response later

        _scan.trans_retry_cnt++;
        if (_scan.trans_retry_cnt > 4) {
            // the request re-transmit failed multiple times, give up this ESC, goto the next one
            _scan.trans_retry_cnt = 0;
            _scan.state = 5;
        }
    }
    return false;
}

/**
    starts all ESCs in bus and prepares them for receiving the fast throttle command.
    Should be periodically called until _config_active >= MOTOR_COUNT_MAX
    @return the current used ID
*/
uint8_t AP_FETtecOneWire::config_escs()
{
    uint8_t response[MAX_RESPONSE_LENGTH];
    uint8_t request[1];
    if (_config_active == 0) {
        _config.delay_loops = 0;
        _config.active_id = 1;
        _config.state = 0;
        _config.timeout = 0;
        _config.wake_from_bl = 1;
        return _config_active + 1;
    }
    while (_found_escs[_config_active-1].active == false && _config_active < MOTOR_COUNT_MAX) {
        _config_active++;
    }

    if (_config_active == MOTOR_COUNT_MAX && _config.wake_from_bl == 0) {
        return _config_active;
    } else if (_config_active == MOTOR_COUNT_MAX && _config.wake_from_bl) {
        _config.wake_from_bl = 0;
        _config.active_id = 1;
        _config_active = 1;
        _config.state = 0;
        _config.timeout = 0;

        _min_id = MOTOR_COUNT_MAX;
        _max_id = 0;
        _id_count = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT_MAX; i++) {
            if (_found_escs[i].active) {
                _id_count++;
                if (i < _min_id) {
                    _min_id = i;
                }
                if (i > _max_id) {
                    _max_id = i;
                }
            }
        }

        if (_id_count == 0 || _max_id - _min_id > _id_count - 1) { // try again, if no ESCs are found or a gap is in ID list. Setup should not started.
            _config.wake_from_bl = 1;
            return _config.active_id;
        }
        _fast_throttle_byte_count = 1;
        int16_t bitCount = 12 + (_id_count * 11);
        while (bitCount > 0) {
            _fast_throttle_byte_count++;
            bitCount -= 8;
        }
        _config.set_fast_command[1] = _fast_throttle_byte_count; // just for older ESC FW versions since 1.0 001 this byte is ignored as the ESC calculates it itself
        _config.set_fast_command[2] = _min_id+1;                 // min ESC id
        _config.set_fast_command[3] = _id_count;                 // count of ESCs that will get signals
    }

    if (_config.delay_loops > 0) {
        _config.delay_loops--;
        return _config_active;
    }

    if (_config.active_id < _config_active) {
        _config.active_id = _config_active;
        _config.state = 0;
        _config.timeout = 0;
    }

    if (_config.timeout == 3 || _config.timeout == 6 || _config.timeout == 9 || _config.timeout == 12) {
        pull_reset();
    }

    if (_config.timeout < 15) {
        if (_config.wake_from_bl) {
            switch (_config.state) {
            case 0:
                request[0] = OW_BL_START_FW;
                if (_found_escs[_config.active_id-1].in_boot_loader) {
                    transmit(_config.active_id, request, 1);
                    _config.delay_loops = 5;
                } else {
                    return _config.active_id + 1;
                }
                _config.state = 1;
                break;
            case 1:
                request[0] = OW_OK;
                if (pull_command(_config.active_id, request, response, return_type::FULL_FRAME, 1)) {
                    _config.timeout = 0;
                    if (response[0] == 0x02) {
                        _found_escs[_config.active_id-1].in_boot_loader = true;
                        _config.state = 0;
                    } else {
                        _found_escs[_config.active_id-1].in_boot_loader = false;
                        _config.delay_loops = 1;
                        return _config.active_id + 1;
                    }
                } else {
                    _config.timeout++;
                }
                break;
            }
        } else {
            if (pull_command(_config.active_id, _config.set_fast_command, response, return_type::RESPONSE, 4)) {
                _config.timeout = 0;
                _config.delay_loops = 1;
                return _config.active_id + 1;
            } else {
                _config.timeout++;
            }
        }
    } else {
        pull_reset();
        return _config.active_id + 1;
    }
    return _config.active_id;
}

#if HAL_WITH_ESC_TELEM
/**
    sets the telemetry mode to full mode, where one ESC answers with all telem values including CRC Error count and a CRC
    @return returns the response code
*/
uint8_t AP_FETtecOneWire::set_full_telemetry(uint8_t active)
{
    if (_found_escs[_set_full_telemetry_active-1].active) { //If ESC is detected at this ID
        uint8_t response[1];
        uint8_t request[2];
        request[0] = OW_SET_TLM_TYPE;
        request[1] = active; //Alternative Tlm => 1, normal TLM => 0
        bool pull_response = pull_command(_set_full_telemetry_active, request, response, return_type::RESPONSE, 2);
        if (pull_response) {
            if(response[0] == OW_OK) {//Ok received or max retries reached.
                _set_full_telemetry_active++;   //If answer from ESC is OK, increase ID.
                _set_full_telemetry_retry_count = 0; //Reset retry count for new ESC ID
            } else {
                _set_full_telemetry_retry_count++; //No OK received, increase retry count
            }
        } else {

            _set_full_telemetry_retry_count++;
            if (_set_full_telemetry_retry_count > 128) { //It is important to have the correct telemetry set so start over if there is something wrong.
                _pull_busy = false;
                _set_full_telemetry_active = 1;
                _set_full_telemetry_retry_count = 0;
            }
        }
    } else { //If there is no ESC detected skip it.
        _set_full_telemetry_active++;
    }
    return _set_full_telemetry_active;
}

/**
    increment message packet count for every ESC
*/
void AP_FETtecOneWire::inc_send_msg_count()
{
    _send_msg_count++;
    if (_send_msg_count > 4 * _update_rate_hz) { // resets every four seconds
        _send_msg_count = 0; //reset the counter
        for (int i=0; i<_id_count; i++) {
            _error_count_since_overflow[i] = _error_count[i]; //save the current ESC error state
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
    _error_count[esc_id] = current_error_count; //Save the error count to the esc
    uint16_t corrected_error_count = (uint16_t)((uint16_t)_error_count[esc_id] - (uint16_t)_error_count_since_overflow[esc_id]); //calculates error difference since last overflow.
    return (float)corrected_error_count*_crc_error_rate_factor; //calculates percentage
}

/**
    if init is complete checks if the requested telemetry is available.
    @param t telemetry datastructure where the read telemetry will be stored in.
    @param centi_erpm 16bit centi-eRPM value returned from the ESC
    @param tx_err_count Ardupilot->ESC communication CRC error counter
    @param tlm_from_id receives the ID from the ESC that has respond with its telemetry
    @return 1 if CRC is correct, 2 on CRC mismatch, 0 on waiting for answer
*/
AP_FETtecOneWire::receive_response AP_FETtecOneWire::decode_single_esc_telemetry(TelemetryData& t, int16_t& centi_erpm, uint16_t& tx_err_count, uint8_t &tlm_from_id)
{
    receive_response ret = receive_response::NO_ANSWER_YET;
    if (_id_count > 0) {
        uint8_t telem[FRAME_OVERHEAD + 11];
        ret = receive((uint8_t *) telem, 11, return_type::FULL_FRAME);

        if (ret == receive_response::ANSWER_VALID) {
            tlm_from_id = (uint8_t)telem[1];

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
    if (_id_count > 0) {
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
        fast_throttle_command[1] |= ALL_ID;

        // following bytes are the rest 7 bit of the first (11bit) throttle value,
        // and all bits from all other values, followed by the CRC byte
        uint8_t bits_left_from_command = 7;
        uint8_t act_byte = 2;
        uint8_t bits_from_byte_left = 8;
        uint16_t bits_to_add_left = (12 + (_id_count * 11)) - 16;
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

        fast_throttle_command[_fast_throttle_byte_count - 1] =
            crc8_dvb_update(0, fast_throttle_command, _fast_throttle_byte_count - 1);

        // No command was yet sent, so no reply is expected and all information
        // on the receive buffer is either garbage or noise. Discard it
        _uart->discard_input();

        // send throttle commands to all configured ESCs in a single packet transfer
        _uart->write(fast_throttle_command, _fast_throttle_byte_count);
    }
}

/// periodically called from SRV_Channels::push()
void AP_FETtecOneWire::update()
{
    if (!_initialised) {
        init();
        return; // the rest of this function can only run after fully initted
    }

    if (_uart == nullptr) {
        return; // no serial port available, so nothing to do here
    }

    // get ESC set points, stop as soon as there is a gap
    uint16_t motor_pwm[MOTOR_COUNT_MAX];
    for (uint8_t i = 0; i < _nr_escs_in_bitmask; i++) {
        SRV_Channel* c = SRV_Channels::srv_channel(i);
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
        if (_requested_telemetry_from_esc == _id_count) { //if found esc number is reached restart request counter
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
            const float tx_err_rate = calc_tx_crc_error_perc(_requested_telemetry_from_esc, tx_err_count);
            update_rpm(tlm_from_id-1, centi_erpm*100*2/_pole_count.get(), tx_err_rate);

            update_telem_data(tlm_from_id-1, t, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE|AP_ESC_Telem_Backend::TelemetryType::VOLTAGE|AP_ESC_Telem_Backend::TelemetryType::CURRENT|AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);
        }
#endif
    }

    // Now that all real-time tasks above have been done, do some periodic checks.
    configuration_check();
}
#endif  // HAL_AP_FETTEC_ONEWIRE_ENABLED
