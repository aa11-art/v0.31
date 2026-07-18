#ifndef _uart_h_
#define _uart_h_

#include "zf_common_headfile.h"

#define UART_USE_INTERRUPT 1
#define UART_INDEX            (UART_1)                                    
#define UART_BAUDRATE         (115200)                                   
#define UART_TX_PIN           (UART1_TX_B12 )                            
#define UART_RX_PIN           (UART1_RX_B13)                           

void UART_init(void);
void label_uart_init(void);
void label_uart_write_request(const uint8_t *data, uint32_t length);


#endif
