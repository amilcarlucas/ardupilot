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

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_AP_FETTEC_ONEWIRE_ENABLED
#define HAL_AP_FETTEC_ONEWIRE_ENABLED !HAL_MINIMIZE_FEATURES && !defined(HAL_BUILD_AP_PERIPH) && BOARD_FLASH_SIZE > 1024
#endif

#ifndef HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
#define HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO 0
#endif

#if HAL_AP_FETTEC_ONEWIRE_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>



class AP_FETtecOneWire : public AP_ESC_Telem_Backend
{

public:
    AP_FETtecOneWire();

    /// Do not allow copies
    AP_FETtecOneWire(const AP_FETtecOneWire &other) = delete;
    AP_FETtecOneWire &operator=(const AP_FETtecOneWire&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    /// periodically called from SRV_Channels::push()
    void update();
    static AP_FETtecOneWire *get_singleton() {
        return _singleton;
    }
private:
    static AP_FETtecOneWire *_singleton;
    AP_HAL::UARTDriver *_uart;

#if HAL_WITH_ESC_TELEM
    static constexpr uint8_t MOTOR_COUNT_MAX = ESC_TELEM_MAX_ESCS; /// OneWire supports up-to 15 ESCs, but Ardupilot only supports 12
#else
    static constexpr uint8_t MOTOR_COUNT_MAX = 12;                 /// OneWire supports up-to 15 ESCs, but Ardupilot only supports 12
#endif
    AP_Int32 _motor_mask;
    AP_Int8 _pole_count;

    enum class return_type : uint8_t
    {
        RESPONSE,
        FULL_FRAME
    };

    enum class receive_response : uint8_t
    {
        NO_ANSWER_YET,
        ANSWER_VALID,
        CRC_MISSMATCH
    };

    /**
        initialize the serial port, scan the OneWire bus, setup the found ESCs
    */
    void init();

    /**
        check if the current configuration is OK
    */
    void configuration_check();

    /**
        transmits a FETtec OneWire frame to an ESC
        @param esc_id id of the ESC
        @param bytes  8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
        @param length length of the bytes array
    */
    void transmit(const uint8_t esc_id, const uint8_t *bytes, uint8_t length);

    /**
        reads the FETtec OneWire answer frame of an ESC
        @param bytes 8 bit byte array, where the received answer gets stored in
        @param length the expected answer length
        @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
        @return 2 on CRC error, 1 if the expected answer frame was there, 0 if dont
    */
    receive_response receive(uint8_t *bytes, uint8_t length, return_type return_full_frame);

    /**
        Resets a pending pull request
    */
    void pull_reset();

    /**
        Pulls a complete request between flight controller and ESC
        @param esc_id id of the ESC
        @param command 8bit array containing the command that should be send including the possible payload
        @param response 8bit array where the response will be stored in
        @param return_full_frame can be return_type::RESPONSE or return_type::FULL_FRAME
        @param req_len transmit request length
        @return true if the request is completed, false if dont
    */
    bool pull_command(const uint8_t esc_id, const uint8_t *command, uint8_t *response, return_type return_full_frame, const uint8_t req_len);

    /**
        scans for ESCs in bus.
        Should be periodically called until it returns true
        @return true when OneWire bus scan is complete
    */
    bool scan_escs();

    /**
        starts all ESCs in bus and prepares them for receiving the fast throttle command.
        Should be periodically called until _config_active >= MOTOR_COUNT_MAX
        @return the current used ID
    */
    uint8_t config_escs();

#if HAL_WITH_ESC_TELEM
    /**
        sets the telemetry mode to full mode, where one ESC answers with all telem values including CRC Error count and a CRC
        @param active if full telemetry should be used
        @return returns the response code
    */
    uint8_t set_full_telemetry(uint8_t active);

    /**
        increment message packet count for every ESC
    */
    void inc_send_msg_count();

    /**
        calculates tx (outgoing packets) error-rate by converting the CRC error counts reported by the ESCs into percentage
        @param esc_id id of ESC, that the error is calculated for
        @param esc_error_count the error count given by the esc
        @return the error in percent
    */
    float calc_tx_crc_error_perc(const uint8_t esc_id, uint16_t esc_error_count);

    /**
        if init is complete checks if the requested telemetry is available.
        @param t telemetry datastructure where the read telemetry will be stored in.
        @param centi_erpm 16bit centi-eRPM value returned from the ESC
        @param tx_err_count Ardupilot->ESC communication CRC error counter
        @param tlm_from_id receives the ID from the ESC that has respond with its telemetry
        @return 1 if CRC is correct, 2 on CRC mismatch, 0 on waiting for answer
    */
    receive_response decode_single_esc_telemetry(TelemetryData& t, int16_t& centi_erpm, uint16_t& tx_err_count, uint8_t &tlm_from_id);
#endif

    /**
        if init is complete sends a single fast-throttle frame containing the throttle for all found OneWire ESCs.
        @param motor_values a 16bit array containing the throttle values that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 0-999 reversed rotation
        @param tlm_request the ESC to request telemetry from (0 for no telemetry, 1 for ESC0, 2 for ESC1, 3 for ESC2, ...)
    */
    void escs_set_values(const uint16_t *motor_values, const uint8_t tlm_request);

    static constexpr uint8_t SERIAL_NR_BITWIDTH = 12;

    typedef struct FETtecOneWireESC
    {
        bool in_boot_loader;
        bool active;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        uint8_t firmware_version;
        uint8_t firmware_sub_version;
        uint8_t esc_type;
        uint8_t serial_number[SERIAL_NR_BITWIDTH];
#endif
    } FETtecOneWireESC_t;

    FETtecOneWireESC_t _found_escs[MOTOR_COUNT_MAX]; ///< Zero-indexed array
    uint32_t _last_config_check_ms;
    uint32_t _last_send_us;
#if HAL_WITH_ESC_TELEM
    float _crc_error_rate_factor; ///< multiply factor. Used to avoid division operations
    uint16_t _error_count[MOTOR_COUNT_MAX]; ///< error counter from the ESCs. Zero-indexed array
    uint16_t _error_count_since_overflow[MOTOR_COUNT_MAX]; ///< error counter from the ESCs to pass the overflow. Zero-indexed array
    uint16_t _send_msg_count; ///< number of fast-throttle commands send by the flight controller
    uint16_t _update_rate_hz;
#endif
    uint16_t _mask;
    uint8_t _nr_escs_in_bitmask; ///< number of ESCs set on the FTW_MASK parameter

    uint8_t _found_escs_count;   ///< number of ESCs auto-scanned in the OneWire bus by the scan_escs() function
    uint8_t _config_active;
#if HAL_WITH_ESC_TELEM
    uint8_t _set_full_telemetry_active = 1; ///< to set alternative TLM for every ESC
    uint8_t _set_full_telemetry_retry_count;
#endif
    int8_t _min_id;          ///< Zero-indexed ESC ID
    int8_t _max_id;          ///< Zero-indexed ESC ID
    uint8_t _id_count;       ///< number of ESCs fully operational in the OneWire bus and configured by the config_escs() function
    uint8_t _fast_throttle_byte_count;
    uint8_t _requested_telemetry_from_esc; ///< the ESC to request telemetry from (0 for no telemetry, 1 for ESC0, 2 for ESC1, 3 for ESC2, ...)
    bool _initialised;       ///< device driver and ESCs are fully initialized
    bool _uart_initialised;  ///< serial UART is fully initialized
    bool _pull_busy;         ///< request-reply transaction is busy

    enum msg_type
    {
        OW_OK = 0,
        OW_BL_PAGE_CORRECT,   // BL only
        OW_NOT_OK,
        OW_BL_START_FW,       // BL only
        OW_BL_PAGES_TO_FLASH, // BL only
        OW_REQ_TYPE,
        OW_REQ_SN,
        OW_REQ_SW_VER,
        OW_BEEP = 13,
        OW_SET_FAST_COM_LENGTH = 26,
        OW_SET_TLM_TYPE = 27, //1 for alternative telemetry. ESC sends full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
        OW_SET_LED_TMP_COLOR = 51,
    };

    /// presistent scan state data (only used inside scan_escs() function)
    struct scan_state
    {
        uint32_t last_us;
        uint8_t id;
        uint8_t state;
        uint8_t rx_retry_cnt;
        uint8_t trans_retry_cnt;
    } _scan;

    /// presistent config state data (only used inside config_escs() function)
    struct config_state
    {
        uint8_t delay_loops;
        uint8_t active_id;
        uint8_t state;
        uint8_t timeout;
        uint8_t wake_from_bl;
        uint8_t set_fast_command[4] = {OW_SET_FAST_COM_LENGTH, 0, 0, 0};
    } _config;

    uint8_t _response_length[OW_SET_TLM_TYPE+1]; ///< OW_SET_LED_TMP_COLOR is ignored here. You must update this if you add new msg_type cases

};
#endif // HAL_AP_FETTEC_ONEWIRE_ENABLED

