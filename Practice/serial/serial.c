#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define DATA_BAUD 9600
#define AT_BAUD 38400
#define UBRR(BR) ((F_CPU / 16 / BR) - 1)

// HC-05 control pins
#define HC05_KEY_PORT PORTB
#define HC05_KEY_DDR DDRB
#define HC05_KEY_PIN PB0

#define DATA_LED_PORT PORTB
#define DATA_LED_DDR DDRB
#define DATA_LED_PIN PB1

void USART_Init(unsigned int br) {
    unsigned int ubrr = UBRR(br);
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)(ubrr);
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void USART_Transmit(unsigned char data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

void USART_SendString(const char* str) {
    while (*str)
        USART_Transmit(*str++);
}

void USART_FlushRxBuffer() {
    unsigned char dummy;
    while (UCSRA & (1 << RXC)) dummy = UDR;
    (void)dummy;
}

int USART_ReceiveString(char* buffer, int max_len) {
    int i = 0;
    while (i < max_len - 1) {
        if (UCSRA & (1 << RXC)) {
            char c = UDR;
            buffer[i++] = c;
            if (c == '\n' || c == '\r') break;
        }
        _delay_us(100);
    }
    buffer[i] = '\0';
    return i;
}

int HC05_SendCommand(const char* command, char* response_buffer, int max_len, const char* expected_resp) {
    USART_FlushRxBuffer();
    USART_SendString(command);
    _delay_ms(100);
    int len = USART_ReceiveString(response_buffer, max_len);
    if (len > 0) {
        response_buffer[len] = '\0';
        if (strstr(response_buffer, expected_resp) != NULL) return 1;
    }
    return 0;
}

void HC05_EnterATMode() {
    HC05_KEY_DDR |= (1 << HC05_KEY_PIN);
    HC05_KEY_PORT |= (1 << HC05_KEY_PIN); // hold KEY high
    _delay_ms(100);
    USART_Init(AT_BAUD);
    _delay_ms(500);
    USART_FlushRxBuffer();
}

void HC05_EnterDATAMode() {
    // Pull KEY pin LOW for 100ms to request Data mode
    HC05_KEY_PORT &= ~(1 << HC05_KEY_PIN);
    _delay_ms(100);
    DATA_LED_PORT &= ~(1 << DATA_LED_PIN);
    DATA_LED_PORT |= (1 << DATA_LED_PIN); // Set high again as per your request
    _delay_ms(200);
    USART_Init(DATA_BAUD);
    _delay_ms(500);
    USART_FlushRxBuffer();
}

int main(void) {
    // Configure LEDs and HC05_KEY
    HC05_KEY_DDR |= (1 << HC05_KEY_PIN);  // KEY pin output
    DATA_LED_DDR |= (1 << DATA_LED_PIN);
    DATA_LED_PORT |= (1 << DATA_LED_PIN);
    // HC05_KEY_PORT |= (1 << HC05_KEY_PIN); // Always HIGH by default

    char hc05_response[64];
    char app_msg[80];

    // --- Go to AT Mode ---
    HC05_EnterATMode();
    if (HC05_SendCommand("AT\r\n", hc05_response, sizeof(hc05_response), "OK")) {
        sprintf(app_msg, "AT_RESP:OK\r\n");
    } else {
        sprintf(app_msg, "AT_RESP:FAIL(%s)\r\n", hc05_response);
    }
    USART_SendString(app_msg);
    _delay_ms(1000);

    // --- Now switch to Data Mode ---
    HC05_EnterDATAMode();
    DATA_LED_PORT |= (1 << DATA_LED_PIN);
    USART_SendString("MODE:DATA\r\n");

    _delay_ms(1000);

    // --- Main Loop: Send stored response ---
    while (1) {
        USART_SendString("HC05_RESP:");
        USART_SendString(hc05_response);
        USART_SendString("\r\n");
        _delay_ms(1000);
    }

    return 0;
}
