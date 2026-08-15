/*
 * Receive-side framing for the Makera binary protocol.
 *
 * The UART console and the wifi provider speak the same wire format, so the
 * framing, CRC validation and command queueing live here rather than being
 * maintained twice. Deliberately free of Kernel and mbed dependencies so it
 * can be compiled and exercised on a host -- see tests/TEST_MakeraFrameParser.
 *
 * Wire format, multi-byte fields big-endian:
 *
 *   [0..1]                  HEADER
 *   [2..3]                  data_length
 *   [4]                     frame type (PTYPE_*)
 *   [5 .. data_length+1]    payload, data_length - 3 bytes
 *   [data_length+2 .. +3]   CRC16-CCITT over [2 .. data_length+1]
 *   [data_length+4 .. +5]   FOOTER
 *
 * A whole frame is therefore data_length + 6 bytes, and the smallest legal
 * data_length is 3: a frame type with an empty payload.
 */

#ifndef MAKERAFRAMEPARSER_H
#define MAKERAFRAMEPARSER_H

#include <stdint.h>

#include "PublicData.h" // HEADER, FOOTER, PTYPE_*

enum class MakeraEvent : uint8_t {
    None,       // byte consumed, frame still incomplete
    Control,    // CTRL_SINGLE frame; the control byte is in MakeraResult::control
    Command,    // CTRL_MULTI frame; payload queued
    FileStart,  // FILE_START frame; payload queued
    Other,      // complete frame this parser does not act on
    FrameError, // bad length, CRC or footer; the parser has resynchronised
    TooLarge,   // valid frame whose payload does not fit a queue slot
    QueueFull,  // valid frame that arrived with no free queue slot
};

struct MakeraResult {
    MakeraEvent event;
    uint8_t frame_type; // PTYPE_* of the completed frame, 0 while event is None
    uint8_t control;    // first payload byte, only when event is Control
};

// Text describing a frame that was understood but could not be accepted, or
// nullptr for events that need no report. Shared so that both consoles say the
// same thing.
const char *makera_event_message(MakeraEvent event);

class MakeraFrameParser
{
public:
    MakeraFrameParser();

    // Storage is owned by the caller so that each console keeps control of
    // which memory region its buffers land in (the wifi provider places both
    // in AHB SRAM). slot_storage is slot_count * slot_size bytes.
    void init(uint8_t *frame_buf, uint16_t frame_buf_size,
              char *slot_storage, uint16_t *slot_length_storage,
              uint8_t slot_count, uint16_t slot_size);

    // A partial frame is abandoned once this long has passed without another
    // byte, so that one lost byte cannot leave the parser mid-frame
    // indefinitely, swallowing every frame that follows. This is an
    // inter-byte gap rather than a budget for the whole frame, so it only has
    // to outlast a host stalling mid-frame, never the time a frame takes to
    // arrive.
    enum { FRAME_TIMEOUT_MS = 1000 };

    // now_ms is passed in rather than read here so this stays free of mbed.
    MakeraResult process_byte(uint8_t byte, uint32_t now_ms);

    void reset_parser(); // framing state only, leaves the queue alone
    void clear();        // framing state and queue

    bool at_frame_boundary() const { return received == 0; }

    // Bytes still needed to finish the frame in progress. The wifi path reads
    // exactly this many from the module so that whatever follows a FILE_START
    // frame stays queued in the module for the file parser to collect.
    uint16_t bytes_wanted() const;

    bool queue_empty() const;
    bool queue_full() const;
    // Returns nullptr when the queue is empty. The payload stays valid until
    // the matching queue_pop().
    const char *queue_front(uint16_t *len) const;
    void queue_pop();

    static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

private:
    bool queue_push(const uint8_t *payload, uint16_t len);
    char *slot(uint8_t index) const;

    uint8_t *frame;
    uint16_t frame_size;
    char *slots;
    uint16_t *slot_lengths;
    uint8_t slot_count;
    uint16_t slot_size;

    uint16_t header;
    uint16_t received;
    uint16_t data_length;
    uint32_t last_byte_ms;

    volatile uint8_t head;
    volatile uint8_t tail;
};

#endif /* MAKERAFRAMEPARSER_H */
