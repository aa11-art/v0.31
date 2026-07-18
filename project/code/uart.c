#include "uart.h"

void UART_init(void)
{
    uart_init(
        UART_INDEX,                                                    
        UART_BAUDRATE,                                                   
        UART_TX_PIN,                                                   
        UART_RX_PIN);                                                    
    #if UART_USE_INTERRUPT  
            uart_rx_interrupt(DEBUG_UART_INDEX, 1);
    #endif
}


void label_uart_init(void)
{
    uart_init(
        UART_4,
        115200,
        UART4_TX_C16,
        UART4_RX_C17);

    uart_rx_interrupt(UART_4, 1);
}


void label_uart_write_request(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    for(index = 0u; index < length; index++)
    {
        uart_write_byte(UART_4, data[index]);

        if((index + 1u) < length)
        {
            system_delay_ms(30);
        }
    }
}
