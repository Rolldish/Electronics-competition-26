#include "k230_comm.h"

/* 全局解析结果 */
volatile uint8_t  k230_digit       = 0;
volatile uint8_t  k230_confidence  = 0;
volatile bool     k230_new_result  = false;
volatile bool     k230_connected   = false;
volatile uint32_t k230_byte_count  = 0;

/*
 * 接收状态机 - 解析 "$NUM,n,confidence#"
 * 示例: "$NUM,3,85#"  → digit=3, confidence=85
 */
enum {
    S_WAIT_DOLLAR,   /* 0: 等 '$' */
    S_WAIT_N,        /* 1: 等 'N' */
    S_WAIT_U,        /* 2: 等 'U' */
    S_WAIT_M,        /* 3: 等 'M' */
    S_WAIT_COMMA1,   /* 4: 等 ',' */
    S_DIGIT,         /* 5: 读数字 */
    S_WAIT_COMMA2,   /* 6: 等 ',' */
    S_CONFIDENCE,    /* 7: 读置信度 (多字节) */
    S_WAIT_HASH      /* 8: 等 '#' 收尾 */
};

static volatile uint8_t s_state      = S_WAIT_DOLLAR;
static volatile uint8_t s_digit      = 0;
static volatile uint8_t s_confidence = 0;

void K230_Comm_Init(void)
{
    k230_digit      = 0;
    k230_confidence = 0;
    k230_new_result = false;
    k230_connected  = false;
    k230_byte_count = 0;
    s_state         = S_WAIT_DOLLAR;
    s_digit         = 0;
    s_confidence    = 0;
}

void UART_K230_INST_IRQHandler(void)
{
    if (!(DL_UART_Main_getPendingInterrupt(UART_K230_INST) & DL_UART_INTERRUPT_RX))
        return;

    uint8_t c = DL_UART_Main_receiveData(UART_K230_INST);
    k230_byte_count++;

    switch (s_state) {

    case S_WAIT_DOLLAR:
        if (c == '$') s_state = S_WAIT_N;
        break;

    case S_WAIT_N:
        s_state = (c == 'N') ? S_WAIT_U : S_WAIT_DOLLAR;
        break;

    case S_WAIT_U:
        s_state = (c == 'U') ? S_WAIT_M : S_WAIT_DOLLAR;
        break;

    case S_WAIT_M:
        s_state = (c == 'M') ? S_WAIT_COMMA1 : S_WAIT_DOLLAR;
        break;

    case S_WAIT_COMMA1:
        s_state = (c == ',') ? S_DIGIT : S_WAIT_DOLLAR;
        break;

    case S_DIGIT:
        if (c >= '1' && c <= '8') {
            s_digit  = c - '0';       /* '3' → 3 */
            s_state  = S_WAIT_COMMA2;
        } else {
            s_state = S_WAIT_DOLLAR;  /* 不是1~8, 丢弃 */
        }
        break;

    case S_WAIT_COMMA2:
        if (c == ',') {
            s_confidence = 0;
            s_state      = S_CONFIDENCE;
        } else {
            s_state = S_WAIT_DOLLAR;
        }
        break;

    case S_CONFIDENCE:
        if (c >= '0' && c <= '9') {
            s_confidence = s_confidence * 10 + (c - '0');
            if (s_confidence > 100) {   /* 超出范围, 丢弃 */
                s_state = S_WAIT_DOLLAR;
            }
        } else if (c == '#') {
            /* 收尾 → 提交结果 */
            k230_digit      = s_digit;
            k230_confidence = s_confidence;
            k230_new_result = true;
            k230_connected  = true;
            s_state         = S_WAIT_DOLLAR;
        } else {
            s_state = S_WAIT_DOLLAR;   /* 非法字符, 丢弃 */
        }
        break;
    }

    DL_UART_Main_clearInterruptStatus(UART_K230_INST, DL_UART_INTERRUPT_RX);
}
