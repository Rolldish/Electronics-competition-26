#ifndef __K230_COMM_H
#define __K230_COMM_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* 解析结果 (主循环读取) */
extern volatile uint8_t k230_digit;       /* 识别到的数字 1~8, 0=无效 */
extern volatile uint8_t k230_confidence;  /* 置信度 0~100 */
extern volatile bool    k230_new_result;  /* 新结果标志 (主循环消费后清零) */
extern volatile bool    k230_connected;   /* 是否收到过有效帧 */
extern volatile uint32_t k230_byte_count; /* 接收总字节数 */

void K230_Comm_Init(void);
void UART_K230_INST_IRQHandler(void);

#endif
