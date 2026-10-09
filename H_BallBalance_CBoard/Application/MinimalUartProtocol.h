#ifndef MINIMAL_UART_PROTOCOL_H
#define MINIMAL_UART_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    size_t match_index;
} MinimalUartParser;

extern const uint8_t kMinimalUartPing[];
extern const size_t kMinimalUartPingSize;
extern const uint8_t kMinimalUartPong[];
extern const size_t kMinimalUartPongSize;

void MinimalUartParser_Init(MinimalUartParser *parser);
bool MinimalUartParser_Push(MinimalUartParser *parser, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif
