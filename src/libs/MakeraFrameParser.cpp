#include "MakeraFrameParser.h"

#include <string.h>

extern const unsigned short crc_table[256];

const char *makera_event_message(MakeraEvent event)
{
    switch (event) {
        case MakeraEvent::TooLarge:
            return "ERROR: command discarded, longer than the 256 byte limit\r\n";
        case MakeraEvent::QueueFull:
            return "ERROR: command discarded, command queue full\r\n";
        default:
            return nullptr;
    }
}

MakeraFrameParser::MakeraFrameParser()
{
    frame = nullptr;
    frame_size = 0;
    slots = nullptr;
    slot_lengths = nullptr;
    slot_count = 0;
    slot_size = 0;
    clear();
}

void MakeraFrameParser::init(uint8_t *frame_buf, uint16_t frame_buf_size,
                             char *slot_storage, uint16_t *slot_length_storage,
                             uint8_t slot_count_, uint16_t slot_size_)
{
    frame = frame_buf;
    frame_size = frame_buf_size;
    slots = slot_storage;
    slot_lengths = slot_length_storage;
    slot_count = slot_count_;
    slot_size = slot_size_;
    clear();
}

uint16_t MakeraFrameParser::crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = (uint16_t)((crc << 8) ^ crc_table[tmp]);
    }
    return crc;
}

void MakeraFrameParser::reset_parser()
{
    header = 0;
    received = 0;
    data_length = 0;
    last_byte_ms = 0;
}

void MakeraFrameParser::clear()
{
    reset_parser();
    head = 0;
    tail = 0;
}

uint16_t MakeraFrameParser::bytes_wanted() const
{
    if (received < 2) return 2;                      // sliding header window
    if (received < 4) return (uint16_t)(4 - received); // rest of the length field
    // data_length was validated when it was read, and a frame that had all of
    // its bytes would already have been dispatched, so this cannot underflow.
    return (uint16_t)(data_length + 6 - received);
}

char *MakeraFrameParser::slot(uint8_t index) const
{
    return slots + (size_t)index * slot_size;
}

bool MakeraFrameParser::queue_empty() const
{
    return head == tail;
}

bool MakeraFrameParser::queue_full() const
{
    if (slot_count == 0) return true;
    return (uint8_t)((tail + 1) % slot_count) == head;
}

bool MakeraFrameParser::queue_push(const uint8_t *payload, uint16_t len)
{
    if (queue_full()) return false;

    memcpy(slot(tail), payload, len);
    slot_lengths[tail] = len;
    tail = (uint8_t)((tail + 1) % slot_count);
    return true;
}

const char *MakeraFrameParser::queue_front(uint16_t *len) const
{
    if (queue_empty()) return nullptr;
    if (len != nullptr) *len = slot_lengths[head];
    return slot(head);
}

void MakeraFrameParser::queue_pop()
{
    if (queue_empty()) return;
    head = (uint8_t)((head + 1) % slot_count);
}

MakeraResult MakeraFrameParser::process_byte(uint8_t byte, uint32_t now_ms)
{
    MakeraResult result = { MakeraEvent::None, 0, 0 };

    if (frame == nullptr) return result;

    // Drop a frame that stopped arriving. Unsigned arithmetic makes the
    // comparison correct across the tick counter's wrap.
    if ((received > 0 || header != 0) &&
        (uint32_t)(now_ms - last_byte_ms) >= (uint32_t)FRAME_TIMEOUT_MS) {
        reset_parser();
    }
    last_byte_ms = now_ms;

    // Hunting for the header: keep a sliding two-byte window so a frame that
    // follows stray bytes is still picked up.
    if (received < 2) {
        header = (uint16_t)((header << 8) | byte);
        if (header == HEADER) {
            frame[0] = (HEADER >> 8) & 0xff;
            frame[1] = HEADER & 0xff;
            received = 2;
        }
        return result;
    }

    frame[received++] = byte;

    if (received == 4) {
        data_length = (uint16_t)((frame[2] << 8) | frame[3]);
        if (data_length < 3 || (uint32_t)data_length + 6 > frame_size) {
            reset_parser();
            result.event = MakeraEvent::FrameError;
        }
        return result;
    }

    if (received < data_length + 6) return result;

    const uint16_t footer = (uint16_t)((frame[received - 2] << 8) | frame[received - 1]);
    const uint16_t frame_crc = (uint16_t)((frame[received - 4] << 8) | frame[received - 3]);
    if (footer != FOOTER || frame_crc != crc16_ccitt(&frame[2], data_length)) {
        reset_parser();
        result.event = MakeraEvent::FrameError;
        return result;
    }

    result.frame_type = frame[4];
    switch (result.frame_type) {
        case PTYPE_CTRL_SINGLE:
            // data_length 3 is a frame type with no payload, so there is no
            // control byte to act on.
            if (data_length >= 4) {
                result.event = MakeraEvent::Control;
                result.control = frame[5];
            } else {
                result.event = MakeraEvent::Other;
            }
            break;

        case PTYPE_CTRL_MULTI:
        case PTYPE_FILE_START: {
            const uint16_t payload_len = (uint16_t)(data_length - 3);
            if (payload_len == 0) {
                result.event = MakeraEvent::Other;
            } else if (payload_len > slot_size) {
                result.event = MakeraEvent::TooLarge;
            } else if (!queue_push(&frame[5], payload_len)) {
                result.event = MakeraEvent::QueueFull;
            } else {
                result.event = (result.frame_type == PTYPE_FILE_START)
                             ? MakeraEvent::FileStart
                             : MakeraEvent::Command;
            }
            break;
        }

        default:
            result.event = MakeraEvent::Other;
            break;
    }

    reset_parser();
    return result;
}
