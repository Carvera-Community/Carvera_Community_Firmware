/*
 * Host-side tests for MakeraFrameParser.
 *
 * The parser is deliberately free of Kernel and mbed dependencies, so it can
 * be compiled and driven natively. Run with ./run.sh -- see README.md.
 */

#include "MakeraFrameParser.h"

#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

static int checks = 0;
static int failures = 0;
static const char *current_test = "";

#define CHECK(cond)                                                            \
    do {                                                                       \
        checks++;                                                              \
        if (!(cond)) {                                                         \
            failures++;                                                        \
            printf("  FAIL (%s:%d) %s\n", current_test, __LINE__, #cond);      \
        }                                                                      \
    } while (0)

#define TEST(name)                                                             \
    current_test = name;                                                       \
    printf("%s\n", name);

// Same geometry as both consoles use.
enum { FRAME_BUF_SIZE = 544, QUEUE_DEPTH = 4, MAX_PAYLOAD = 256 };

struct Fixture {
    uint8_t frame_buf[FRAME_BUF_SIZE];
    char slots[QUEUE_DEPTH][MAX_PAYLOAD];
    uint16_t lengths[QUEUE_DEPTH];
    MakeraFrameParser parser;

    Fixture()
    {
        memset(frame_buf, 0, sizeof(frame_buf));
        memset(slots, 0, sizeof(slots));
        memset(lengths, 0, sizeof(lengths));
        parser.init(frame_buf, sizeof(frame_buf), &slots[0][0], lengths,
                    QUEUE_DEPTH, MAX_PAYLOAD);
    }
};

static std::vector<uint8_t> build_frame(uint8_t type, const std::string &payload)
{
    std::vector<uint8_t> f;
    const uint16_t data_length = (uint16_t)(payload.size() + 3);

    f.push_back((HEADER >> 8) & 0xff);
    f.push_back(HEADER & 0xff);
    f.push_back((data_length >> 8) & 0xff);
    f.push_back(data_length & 0xff);
    f.push_back(type);
    for (size_t i = 0; i < payload.size(); i++) {
        f.push_back((uint8_t)payload[i]);
    }

    const uint16_t crc = MakeraFrameParser::crc16_ccitt(&f[2], data_length);
    f.push_back((crc >> 8) & 0xff);
    f.push_back(crc & 0xff);
    f.push_back((FOOTER >> 8) & 0xff);
    f.push_back(FOOTER & 0xff);
    return f;
}

// Feeds every byte and returns the one result that was not None, checking that
// nothing else fired along the way.
static MakeraResult feed(MakeraFrameParser &parser, const std::vector<uint8_t> &bytes)
{
    MakeraResult last = { MakeraEvent::None, 0, 0 };
    int events = 0;

    for (size_t i = 0; i < bytes.size(); i++) {
        MakeraResult r = parser.process_byte(bytes[i]);
        if (r.event != MakeraEvent::None) {
            last = r;
            events++;
        }
    }

    CHECK(events <= 1);
    return last;
}

static std::string queued_payload(MakeraFrameParser &parser)
{
    uint16_t len = 0;
    const char *p = parser.queue_front(&len);
    if (p == nullptr) return std::string();
    return std::string(p, len);
}

int main()
{
    {
        TEST("control frame reports the control byte");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_SINGLE, "?"));
        CHECK(r.event == MakeraEvent::Control);
        CHECK(r.control == '?');
        CHECK(r.frame_type == PTYPE_CTRL_SINGLE);
        CHECK(f.parser.queue_empty());
    }

    {
        TEST("command frame is queued and readable");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "G0 X10"));
        CHECK(r.event == MakeraEvent::Command);
        CHECK(!f.parser.queue_empty());
        CHECK(queued_payload(f.parser) == "G0 X10");
        f.parser.queue_pop();
        CHECK(f.parser.queue_empty());
        CHECK(f.parser.queue_front(nullptr) == nullptr);
    }

    {
        TEST("file start frame is queued and distinguishable");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_FILE_START, "/sd/gcodes/part.nc"));
        CHECK(r.event == MakeraEvent::FileStart);
        CHECK(queued_payload(f.parser) == "/sd/gcodes/part.nc");
    }

    {
        TEST("queue preserves order across several frames");
        Fixture f;
        feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "first"));
        feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "second"));
        CHECK(queued_payload(f.parser) == "first");
        f.parser.queue_pop();
        CHECK(queued_payload(f.parser) == "second");
        f.parser.queue_pop();
        CHECK(f.parser.queue_empty());
    }

    {
        TEST("a ring of depth N holds N-1 commands, then reports QueueFull");
        Fixture f;
        for (int i = 0; i < QUEUE_DEPTH - 1; i++) {
            MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "cmd"));
            CHECK(r.event == MakeraEvent::Command);
        }
        CHECK(f.parser.queue_full());
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "overflow"));
        CHECK(r.event == MakeraEvent::QueueFull);
        CHECK(r.frame_type == PTYPE_CTRL_MULTI);
        // The rejected command must not have displaced anything already queued.
        CHECK(queued_payload(f.parser) == "cmd");
    }

    {
        TEST("a full queue drains and accepts again");
        Fixture f;
        for (int i = 0; i < QUEUE_DEPTH - 1; i++) {
            feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "cmd"));
        }
        f.parser.queue_pop();
        CHECK(!f.parser.queue_full());
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "later"));
        CHECK(r.event == MakeraEvent::Command);
    }

    {
        TEST("payload larger than a queue slot is reported, not queued");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, std::string(MAX_PAYLOAD + 1, 'x')));
        CHECK(r.event == MakeraEvent::TooLarge);
        CHECK(f.parser.queue_empty());
    }

    {
        TEST("payload exactly filling a queue slot is accepted");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, std::string(MAX_PAYLOAD, 'x')));
        CHECK(r.event == MakeraEvent::Command);
        CHECK(queued_payload(f.parser).size() == MAX_PAYLOAD);
    }

    {
        TEST("garbage before a frame is skipped");
        Fixture f;
        std::vector<uint8_t> bytes;
        const uint8_t noise[] = { 0x00, 0xff, 0x86, 0x12, 0x55 };
        bytes.insert(bytes.end(), noise, noise + sizeof(noise));
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "G28");
        bytes.insert(bytes.end(), frame.begin(), frame.end());

        MakeraResult last = { MakeraEvent::None, 0, 0 };
        for (size_t i = 0; i < bytes.size(); i++) {
            MakeraResult r = f.parser.process_byte(bytes[i]);
            if (r.event != MakeraEvent::None) last = r;
        }
        CHECK(last.event == MakeraEvent::Command);
        CHECK(queued_payload(f.parser) == "G28");
    }

    {
        TEST("a header byte pair split by noise still syncs");
        Fixture f;
        // 0x86 0x86 0x68 must sync on the second and third bytes.
        f.parser.process_byte(0x86);
        CHECK(f.parser.at_frame_boundary());
        f.parser.process_byte(0x86);
        CHECK(f.parser.at_frame_boundary());
        f.parser.process_byte(0x68);
        CHECK(!f.parser.at_frame_boundary());
    }

    {
        TEST("bad CRC is rejected and the parser resynchronises");
        Fixture f;
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "G28");
        frame[frame.size() - 3] ^= 0xff; // corrupt the low CRC byte
        MakeraResult r = feed(f.parser, frame);
        CHECK(r.event == MakeraEvent::FrameError);
        CHECK(f.parser.queue_empty());
        CHECK(f.parser.at_frame_boundary());

        r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "G29"));
        CHECK(r.event == MakeraEvent::Command);
        CHECK(queued_payload(f.parser) == "G29");
    }

    {
        TEST("bad footer is rejected");
        Fixture f;
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "G28");
        frame[frame.size() - 1] ^= 0xff;
        MakeraResult r = feed(f.parser, frame);
        CHECK(r.event == MakeraEvent::FrameError);
        CHECK(f.parser.queue_empty());
    }

    {
        // Checked byte by byte: a frame with a bad length would also fail its
        // CRC later, so only the timing of the rejection proves the length
        // itself was validated.
        TEST("a length below the minimum is rejected as soon as it is read");
        Fixture f;
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "G28");
        frame[2] = 0x00;
        frame[3] = 0x02; // data_length 2, below the 3 byte minimum

        CHECK(f.parser.process_byte(frame[0]).event == MakeraEvent::None);
        CHECK(f.parser.process_byte(frame[1]).event == MakeraEvent::None);
        CHECK(f.parser.process_byte(frame[2]).event == MakeraEvent::None);
        CHECK(f.parser.process_byte(frame[3]).event == MakeraEvent::FrameError);
        CHECK(f.parser.at_frame_boundary());
    }

    {
        TEST("a length overflowing the frame buffer is rejected as soon as it is read");
        Fixture f;
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "G28");
        frame[2] = 0xff;
        frame[3] = 0xff;

        for (size_t i = 0; i < 3; i++) {
            CHECK(f.parser.process_byte(frame[i]).event == MakeraEvent::None);
        }
        CHECK(f.parser.process_byte(frame[3]).event == MakeraEvent::FrameError);
        CHECK(f.parser.at_frame_boundary());
    }

    {
        TEST("an unknown frame type completes without queueing");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_FILE_DATA, "payload"));
        CHECK(r.event == MakeraEvent::Other);
        CHECK(r.frame_type == PTYPE_FILE_DATA);
        CHECK(f.parser.queue_empty());
    }

    {
        TEST("a control frame with no control byte completes without acting");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_SINGLE, ""));
        CHECK(r.event == MakeraEvent::Other);
    }

    {
        TEST("an empty command payload completes without queueing");
        Fixture f;
        MakeraResult r = feed(f.parser, build_frame(PTYPE_CTRL_MULTI, ""));
        CHECK(r.event == MakeraEvent::Other);
        CHECK(f.parser.queue_empty());
    }

    {
        TEST("bytes_wanted walks the frame down to zero remaining");
        Fixture f;
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "G28");

        CHECK(f.parser.bytes_wanted() == 2); // hunting for the header
        f.parser.process_byte(frame[0]);
        CHECK(f.parser.bytes_wanted() == 2); // still hunting, header incomplete
        f.parser.process_byte(frame[1]);
        CHECK(f.parser.bytes_wanted() == 2); // header found, needs the length
        f.parser.process_byte(frame[2]);
        CHECK(f.parser.bytes_wanted() == 1);
        f.parser.process_byte(frame[3]);
        // Length is known now, so the parser asks for exactly the remainder.
        CHECK(f.parser.bytes_wanted() == frame.size() - 4);

        for (size_t i = 4; i < frame.size() - 1; i++) {
            f.parser.process_byte(frame[i]);
            CHECK(f.parser.bytes_wanted() == frame.size() - 1 - i);
        }
        f.parser.process_byte(frame[frame.size() - 1]);
        CHECK(f.parser.at_frame_boundary());
        CHECK(f.parser.bytes_wanted() == 2);
    }

    {
        TEST("clear drops both the partial frame and the queue");
        Fixture f;
        feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "queued"));
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "partial");
        for (size_t i = 0; i < 6; i++) f.parser.process_byte(frame[i]);
        CHECK(!f.parser.at_frame_boundary());
        CHECK(!f.parser.queue_empty());

        f.parser.clear();
        CHECK(f.parser.at_frame_boundary());
        CHECK(f.parser.queue_empty());
    }

    {
        TEST("reset_parser drops the partial frame but keeps the queue");
        Fixture f;
        feed(f.parser, build_frame(PTYPE_CTRL_MULTI, "queued"));
        std::vector<uint8_t> frame = build_frame(PTYPE_CTRL_MULTI, "partial");
        for (size_t i = 0; i < 6; i++) f.parser.process_byte(frame[i]);

        f.parser.reset_parser();
        CHECK(f.parser.at_frame_boundary());
        CHECK(queued_payload(f.parser) == "queued");
    }

    {
        TEST("an uninitialised parser consumes bytes without faulting");
        MakeraFrameParser parser;
        MakeraResult r = parser.process_byte(0x86);
        CHECK(r.event == MakeraEvent::None);
        CHECK(parser.queue_empty());
    }

    {
        TEST("crc16_ccitt matches a known vector");
        // The protocol's CRC is CRC-16/XMODEM: "123456789" -> 0x31C3.
        const uint8_t v[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
        CHECK(MakeraFrameParser::crc16_ccitt(v, sizeof(v)) == 0x31C3);
    }

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
