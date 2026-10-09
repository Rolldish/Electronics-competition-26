#include "MinimalUartProtocol.h"

const uint8_t kMinimalUartPing[] = {'P', 'I', 'N', 'G', '\r', '\n'};
const size_t kMinimalUartPingSize = sizeof(kMinimalUartPing);
const uint8_t kMinimalUartPong[] = {'P', 'O', 'N', 'G', '\r', '\n'};
const size_t kMinimalUartPongSize = sizeof(kMinimalUartPong);

void MinimalUartParser_Init(MinimalUartParser *parser)
{
    if (parser != NULL) {
        parser->match_index = 0U;
    }
}

bool MinimalUartParser_Push(MinimalUartParser *parser, const uint8_t byte)
{
    if (parser == NULL) {
        return false;
    }
    if (parser->match_index >= kMinimalUartPingSize) {
        parser->match_index = 0U;
        return false;
    }

    if (byte == kMinimalUartPing[parser->match_index]) {
        parser->match_index += 1U;
    } else {
        parser->match_index = (byte == kMinimalUartPing[0]) ? 1U : 0U;
    }

    if (parser->match_index == kMinimalUartPingSize) {
        parser->match_index = 0U;
        return true;
    }
    return false;
}
