#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "hw.h"


void usart3_setup(void) {
    nvic_enable_irq(NVIC_USART3_IRQ);


    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_PR_TX);

    gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART3_PR_RX);
    gpio_set(GPIOC, GPIO_USART3_PR_RX);

    AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP;

    usart_set_baudrate(USART3, 31250);
    usart_set_databits(USART3, 8);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_stopbits(USART3, USART_STOPBITS_1);

    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART3, USART_MODE_TX_RX);

    USART_CR1(USART3) |= USART_CR1_RXNEIE;
    usart_enable(USART3);
}

void usart2_setup(void) {
    nvic_enable_irq(NVIC_USART2_IRQ);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

    // Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive.
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART2_RX);
    gpio_set(GPIOA, GPIO_USART2_RX);

    // Setup UART parameters. 
    usart_set_baudrate(USART2, 31250);
    usart_set_databits(USART2, 8);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_stopbits(USART2, USART_STOPBITS_1);

    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);

    /* Enable USART1 Receive interrupt. */
    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART2);
}

void usart1_setup(void) {
    /* Enable the USART1 interrupt. */
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    /* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_USART1_RX);
    gpio_set(GPIOA, GPIO_USART1_RX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);

    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    /* Enable USART1 Receive interrupt. */
    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART1);
}

void usart_setup(void) {
    usart1_setup();
//    usart2_setup();
//    usart3_setup();
}



