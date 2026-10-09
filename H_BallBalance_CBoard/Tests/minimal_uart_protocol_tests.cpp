#include "MinimalUartProtocol.h"

#include <cassert>
#include <cstddef>
#include <cstdint>

namespace {

void pushBytes(
    MinimalUartParser &parser,
    const std::uint8_t *bytes,
    const std::size_t size,
    std::size_t &matches) {
    for (std::size_t index = 0U; index < size; ++index) {
        if (MinimalUartParser_Push(&parser, bytes[index])) {
            matches += 1U;
        }
    }
}

} // namespace

int main() {
    MinimalUartParser parser{};
    MinimalUartParser_Init(&parser);

    std::size_t matches = 0U;
    pushBytes(parser, kMinimalUartPing, kMinimalUartPingSize, matches);
    assert(matches == 1U);

    const std::uint8_t noise[] = {
        0x00U, 'P', 'X', 'P', 'I', 'N', 'G', '\r', '\n'};
    pushBytes(parser, noise, sizeof(noise), matches);
    assert(matches == 2U);

    pushBytes(
        parser, kMinimalUartPing,
        kMinimalUartPingSize - 1U, matches);
    assert(matches == 2U);

    parser.match_index = kMinimalUartPingSize + 3U;
    assert(!MinimalUartParser_Push(&parser, 'P'));
    assert(parser.match_index == 0U);
    pushBytes(parser, kMinimalUartPing, kMinimalUartPingSize, matches);
    assert(matches == 3U);

    return 0;
}
