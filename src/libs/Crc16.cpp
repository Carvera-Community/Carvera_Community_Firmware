#include "Crc16.h"

namespace {

enum { Polynomial = 0x1021 };

constexpr uint16_t crc_entry_step(uint16_t crc, unsigned bit)
{
    return bit == 8
        ? crc
        : crc_entry_step(static_cast<uint16_t>((crc << 1) ^ ((crc & 0x8000) ? Polynomial : 0)), bit + 1);
}

constexpr uint16_t crc_entry(size_t index)
{
    return crc_entry_step(static_cast<uint16_t>(index << 8), 0);
}

template <size_t... indexes>
struct IndexSequence {};

template <size_t n, size_t... indexes>
struct MakeIndexSequence : MakeIndexSequence<n - 1, n - 1, indexes...> {};

template <size_t... indexes>
struct MakeIndexSequence<0, indexes...> {
    typedef IndexSequence<indexes...> type;
};

struct CrcTable {
    uint16_t values[256];
};

template <size_t... indexes>
constexpr CrcTable make_crc_table(IndexSequence<indexes...>)
{
    return {{ crc_entry(indexes)... }};
}

constexpr CrcTable crc_table = make_crc_table(typename MakeIndexSequence<256>::type());

}

namespace crc16 {

uint16_t ccitt_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        const uint8_t tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table.values[tmp]) & 0xffff;
    }

    return crc;
}

uint16_t ccitt(const uint8_t *data, size_t len)
{
    return ccitt_update(0, data, len);
}

}
