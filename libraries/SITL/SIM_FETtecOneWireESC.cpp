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
/*
  Simulator for the FETtecOneWireESC

  TODO: if we don't response to the second configuration message the driver does not currently reset
 - verify the assertion that DMA is required
 - verify prearm checks work if esc telem not compiled in
 - current code structure means we have to wake things from bootloader?
 - raw_length is used to find the checksum and that's determined from the expected message type rather than the received message type....
 - stop ignoring REQ_TYPE while in bootloader?
 - fix when telemetry is actually sent
 - correct visibility of members in simulation
 - consider "no-pulses" behaviour? (safety switch on)
 - need to rename FETtecOneWireESC to just FETtecESC now
*/

#include <AP_Math/AP_Math.h>

#include "SIM_FETtecOneWireESC.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include <stdio.h>
#include <errno.h>


#define FTW_DEBUGGING 0
#if FTW_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {::fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo FETtecOneWireESC::var_info[] = {

    // @Param: ENA
    // @DisplayName: FETtecOneWireESC Generator sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FETtecOneWireESC simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 0, FETtecOneWireESC, _enabled, 0),

    AP_GROUPEND
};

FETtecOneWireESC::FETtecOneWireESC() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise serial numbers and IDs
    for (uint8_t n=0; n<ARRAY_SIZE(escs); n++) {
        ESC &esc = escs[n];
        esc.id = n+1;
        for (uint8_t i=0; i<ARRAY_SIZE(esc.sn); i++) {
            esc.sn[i] = n+1;
        }
    }
}

void FETtecOneWireESC::update_escs()
{
    for (uint8_t i=0; i<ARRAY_SIZE(escs); i++) {
        ESC &esc = escs[i];
        switch (esc.state) {
        case ESC::State::IN_BOOTLOADER:
        case ESC::State::RUNNING:
            continue;
        case ESC::State::RUNNING_START:
            esc.state = ESC::State::RUNNING;
            send_response(PackedMessage<OK> {
                esc.id,
                OK{}
            });
        }
    }
}

void FETtecOneWireESC::update(const struct sitl_input &input)
{
    if (!_enabled.get()) {
        return;
    }

    update_escs();

    update_input();
    update_send();
}

void FETtecOneWireESC::handle_config_message()
{
    ESC &esc = escs[u.config_message_header.target_id-1];
    debug("Config message type=%u s=%u esc=%u", (unsigned)u.config_message_header.request_type, (unsigned)esc.state, (unsigned)u.config_message_header.target_id);
    if ((ResponseFrameHeaderID)u.config_message_header.header != ResponseFrameHeaderID::MASTER) {
        AP_HAL::panic("Unexpected header ID");
    }
    switch (esc.state) {
    case ESC::State::IN_BOOTLOADER:
        return bootloader_handle_config_message(esc);
    case ESC::State::RUNNING_START:
        return;
    case ESC::State::RUNNING:
        return running_handle_config_message(esc);
    }
    AP_HAL::panic("Unknown state");
}

template <typename T>
void FETtecOneWireESC::send_response(const T &r)
{
    debug("Sending response");
    if (write_to_autopilot((char*)&r, sizeof(r)) != sizeof(r)) {
        AP_HAL::panic("short write");
    }
}

void FETtecOneWireESC::bootloader_handle_config_message(FETtecOneWireESC::ESC &esc)
{
    switch ((ConfigMessageType)u.config_message_header.request_type) {
    case ConfigMessageType::OK: {
        PackedMessage<OK> msg {
            esc.id,
            OK{},
        };
        msg.header = (uint8_t)ResponseFrameHeaderID::BOOTLOADER;
        msg.update_checksum();
        send_response(msg);
        return;
    }
    case ConfigMessageType::BL_PAGE_CORRECT:   // BL only
    case ConfigMessageType::NOT_OK:
        break;
    case ConfigMessageType::BL_START_FW:       // BL only
        esc.state = ESC::State::RUNNING_START;
        // the main firmware sends an OK
        return;
    case ConfigMessageType::BL_PAGES_TO_FLASH: // BL only
        break;
    case ConfigMessageType::REQ_TYPE:
        // ignore this for now
        return;
    case ConfigMessageType::REQ_SN:
    case ConfigMessageType::REQ_SW_VER:
    case ConfigMessageType::BEEP:
    case ConfigMessageType::SET_FAST_COM_LENGTH:
    case ConfigMessageType::SET_TLM_TYPE: //1 for alternative telemetry. ESC sends full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
    case ConfigMessageType::SET_LED_TMP_COLOR:
        break;
    }
    return;
    AP_HAL::panic("Unhandled config message in bootloader (%u)",
                  (unsigned)u.config_message_header.request_type);
}

void FETtecOneWireESC::running_handle_config_message(FETtecOneWireESC::ESC &esc)
{
    switch ((ConfigMessageType)u.config_message_header.request_type) {

    case ConfigMessageType::OK:
        return send_response(PackedMessage<OK> {
            esc.id,
            OK{}
        });

    case ConfigMessageType::BL_PAGE_CORRECT:   // BL only
    case ConfigMessageType::NOT_OK:
    case ConfigMessageType::BL_START_FW:       // BL only
    case ConfigMessageType::BL_PAGES_TO_FLASH: // BL only
        break;

    case ConfigMessageType::REQ_TYPE:
        return send_response(PackedMessage<ESC_TYPE> {
            esc.id,
            ESC_TYPE{esc.type}
        });

    case ConfigMessageType::REQ_SN:
        return send_response(PackedMessage<SN> {
            esc.id,
            SN{esc.sn, ARRAY_SIZE(esc.sn)}
        });

    case ConfigMessageType::REQ_SW_VER:
        return send_response(PackedMessage<SW_VER> {
            esc.id,
            SW_VER{esc.sw_version, esc.sw_subversion}
        });

    case ConfigMessageType::BEEP:
        break;

    case ConfigMessageType::SET_FAST_COM_LENGTH:
        esc.fast_com.length = u.packed_set_fast_com_length.msg.length;
        esc.fast_com.byte_count = u.packed_set_fast_com_length.msg.byte_count;
        esc.fast_com.min_esc_id = u.packed_set_fast_com_length.msg.min_esc_id;
        esc.fast_com.id_count = u.packed_set_fast_com_length.msg.id_count;
        return send_response(PackedMessage<OK> {
            esc.id,
            OK{}
        });

    case ConfigMessageType::SET_TLM_TYPE: //1 for alternative telemetry. ESC sends full telem per ESC: Temp, Volt, Current, ERPM, Consumption, CrcErrCount
        return handle_config_message_set_tlm_type(esc);

    case ConfigMessageType::SET_LED_TMP_COLOR:
        break;
    }
    AP_HAL::panic("Unknown config message (%u)", (unsigned)u.config_message_header.request_type);
}


void FETtecOneWireESC::handle_config_message_set_tlm_type(ESC &esc)
{
    const TLMType type = (TLMType)u.packed_set_tlm_type.msg.type;
    switch (type) {
    case TLMType::NORMAL:
    case TLMType::ALTERNATIVE:
        esc.telem_type = type;
        send_response(PackedMessage<OK> {
            esc.id,
            OK{}
        });
        return;
    }
    AP_HAL::panic("unknown telem type=%u", (unsigned)type);
}

void FETtecOneWireESC::handle_fast_esc_data()
{
    uint8_t id_count = 12; // esc.fast_com.id_count;  // should come from autopilot

    // decode first byte - see driver for details
    const uint8_t telem_request = u.buffer[0] >> 4;

    // ::fprintf(stderr, "telem_request=%u\n", (unsigned)telem_request);
    escs[0].pwm = ((u.buffer[0] >> 3) & 0x1) << 10;
    if ((u.buffer[0] & 0b111) != 0x1) {
        AP_HAL::panic("expected fast-throttle command");
    }

    // decode second byte
    escs[0].pwm |= (u.buffer[1] >> 5) << 7;
    if ((u.buffer[1] & 0b00011111) != 0x1f) {
        AP_HAL::panic("Unexpected 5-bit target id");
    }

    // decode enough of third byte to complete pwm[0]
    escs[0].pwm |= u.buffer[2] >> 1;

    if (telem_request == escs[0].id) {
        escs[0].telem_request = true;
    }
        ::fprintf(stderr, "pwm[%u] out: %u\n", 0, (unsigned)escs[0].pwm);

    // decode remainder of ESC values
    // slides a window across the input buffer, extracting 11-bit ESC values
    // byte_ofs*8 and bit_ofs are added to find current MSB
    uint8_t byte_ofs = 2;
    uint8_t bit_ofs = 7;
    for (uint8_t i=1; i<id_count; i++) {
        if (telem_request == escs[i].id) {
            escs[i].telem_request = true;
        }
        const uint16_t window = u.buffer[byte_ofs]<<8|u.buffer[byte_ofs+1];
        // zero top bits in window by shifting left then right
        const uint16_t tmp = uint16_t(window << bit_ofs);
        escs[i].pwm = tmp >> 5;
        ::fprintf(stderr, "pwm[%u] out: %u\n", i, (unsigned)escs[i].pwm);
        bit_ofs += 11;
        while (bit_ofs > 7) {
            byte_ofs++;
            bit_ofs -= 8;
        }
    }
}

void FETtecOneWireESC::consume_bytes(uint8_t count)
{
    if (count > buflen) {
        AP_HAL::panic("Consuming more bytes than in buffer?");
    }
    if (buflen == count) {
        buflen = 0;
        return;
    }
    memmove(&u.buffer[0], &u.buffer[u.config_message_header.frame_len], buflen - u.config_message_header.frame_len);
    buflen -= u.config_message_header.frame_len;
}

void FETtecOneWireESC::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&u.buffer[buflen], ARRAY_SIZE(u.buffer) - buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }

    // bool config_message_checksum_fail = false;

    if (buflen > offsetof(ConfigMessageHeader, header) &&
        u.config_message_header.header == 0x01 &&
        buflen > offsetof(ConfigMessageHeader, frame_len) &&
        buflen >= u.config_message_header.frame_len) {
        const uint8_t calculated_checksum = crc8_dvb_update(0, u.buffer, u.config_message_header.frame_len-1);
        const uint8_t received_checksum = u.buffer[u.config_message_header.frame_len-1];
        if (calculated_checksum == received_checksum) {
            handle_config_message();
            // consume the message:
            consume_bytes(u.config_message_header.frame_len);
            return;
        } else {
            debug("Checksum mismatch");
            abort();
            // config_message_checksum_fail = true;
        }
        return; // 1 message/loop....
    }

    // no config message, so let's see if there's fast PWM input.  We
    // use the first ESC's concept of the fast-com bytes...
    uint8_t id_count = 12; //esc.fast_com.id_count;
    const uint16_t total_bits_required = 12 + (id_count*11);
    const uint8_t bytes_required = 1 + (total_bits_required + 7) / 8;

    if (buflen < bytes_required) {
        return;
    }

    if (buflen == bytes_required) {
        const uint8_t calculated_checksum = crc8_dvb_update(0, u.buffer, buflen-1);
        if (u.buffer[buflen-1] != calculated_checksum) {
            AP_HAL::panic("checksum failure");
        }

        handle_fast_esc_data();
        consume_bytes(bytes_required);
        return;
    }

    // debug("Read (%d) bytes from autopilot (%u)", (signed)n, config_message_checksum_fail);
    buflen = 0;
    if (n >= 0) {
        abort();
    }
}

void FETtecOneWireESC::update_sitl_input_pwm(struct sitl_input &input)
{
    // overwrite the SITL input values passed through from
    // sitl_model->update_model with those we're receiving serially
    for (auto &esc : escs) {
        if (esc.id > ARRAY_SIZE(input.servos)) {
            // silently ignore; input.servos is 12-long, we are
            // usually 16-long
            continue;
        }
        input.servos[esc.id-1] = esc.pwm;
    }
}

void FETtecOneWireESC::send_esc_telemetry()
{
    for (auto &esc : escs) {
        if (!esc.telem_request) {
            continue;
        }
        if (esc.state != ESC::State::RUNNING) {
            continue;
        }
        if (esc.telem_type != TLMType::ALTERNATIVE) {
            // no idea what "normal" looks like
            abort();
        }
        esc.telem_request = false;

        const int8_t temp_cdeg = (5 + esc.id) * 100;
        const uint16_t voltage = (6 + esc.id * 100);
        const uint16_t current = (6 + esc.id * 100);
        const int16_t rpm = ARRAY_SIZE(escs)/2 - esc.id;
        const uint16_t consumption_mah = 0;
        const uint16_t errcount = 17;
        send_response(PackedMessage<ESCTelem> {
            esc.id,
            ESCTelem{temp_cdeg, voltage, current, rpm, consumption_mah, errcount}
        });
    }
}

void FETtecOneWireESC::update_send()
{
    send_esc_telemetry();
}
