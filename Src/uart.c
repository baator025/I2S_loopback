#include "uart.h"

static void uart2_set_baudrate(uint32_t peripheral_clock, uint32_t baudrate);
static uint16_t compute_uart_baudrate(uint32_t peripheral_clock, uint32_t baudrate);
void uart2_write(int ch);
int __io_putchar(int ch);

int __io_putchar(int ch)
{
    uart2_write(ch);
    return ch;
}

void uart2_tx_init(void)
{
    /* UART GPIO Pin Config */

    // GPIO clock access
    RCC->AHB1ENR |= GPIOAEN;
    // Set alternate function mode
    GPIOA->MODER &= ~(1U << 4);
    GPIOA->MODER |= (1U << 5);
    // Choose alternate function mode
    GPIOA->AFR[0] |= (1U << 8);
    GPIOA->AFR[0] |= (1U << 9);
    GPIOA->AFR[0] |= (1U << 10);
    GPIOA->AFR[0] &= ~(1U << 11);

    /* UART config */
    // UART2 clock access
    RCC->APB1ENR |= UART2EN;
    // baudrate config
    uart2_set_baudrate(APB1_CLK, UART_BAUDRATE);
    // transfer direction
    USART2->CR1 = CR1_TE;
    // enable UART module
    USART2->CR1 |= CR1_UI;
}

void uart2_write(int ch)
{
    // wait until register is empty
    while(!(USART2->SR & SR_TXE)){}
    // put character to transmit register
    USART2->DR = (ch & 0xFF);

}

static void uart2_set_baudrate(uint32_t peripheral_clock, uint32_t baudrate)
{
    USART2->BRR = compute_uart_baudrate(peripheral_clock, baudrate);
}

static uint16_t compute_uart_baudrate(uint32_t peripheral_clock, uint32_t baudrate)
{
    return ((peripheral_clock + (baudrate / 2U)) / baudrate);
}