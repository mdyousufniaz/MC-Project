#include <avr/io.h>
#define F_CPU 8000000UL // Adjust to your ATmega32 clock frequency (e.g., 8000000UL for 8MHz)
#include <util/delay.h>
#include <stdlib.h> // Required for strtol
#include <string.h> // Required for strstr, strrchr, strncpy
#include <stdio.h>  // Required for sprintf

// --- Define BAUD rates ---
#define HC05_DATA_BAUD 9600 // IMPORTANT: This MUST match your HC-05's configured data mode baud rate
#define HC05_AT_BAUD 38400  // Fixed AT command mode baud rate for HC-05

// Calculate UBRR values for different baud rates
#define MYUBRR_DATA (F_CPU/16/HC05_DATA_BAUD)-1
#define MYUBRR_AT (F_CPU/16/HC05_AT_BAUD)-1

// --- HC-05 KEY/EN Pin Definition ---
// Connect HC-05 KEY/EN to PB0 on ATmega32
#define HC05_KEY_PORT PORTB
#define HC05_KEY_DDR DDRB
#define HC05_KEY_PIN PB0

// --- LED Pin Definition for Status ---
// Connect an LED (with a current-limiting resistor, e.g., 220-470 Ohm) to PB1
#define STATUS_LED_PORT PORTB
#define STATUS_LED_DDR DDRB
#define STATUS_LED_PIN PB1

// --- USART (UART) Functions ---

// Initialize USART with a given UBRR value
void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSRB = (1 << RXEN) | (1 << TXEN);
    // Set frame format: 8 data bits, 1 stop bit, no parity
    // URSEL bit is for ATmega32 to select UCSRC register
    UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

// Transmit a single character via USART
void USART_Transmit(unsigned char data) {
    // Wait for empty transmit buffer
    while (!(UCSRA & (1 << UDRE)));
    // Put data into buffer, send the data
    UDR = data;
}

// Send a null-terminated string via USART
void USART_SendString(const char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

// Receive a single character via USART
unsigned char USART_Receive(void) {
    // Wait for data to be received
    while (!(UCSRA & (1 << RXC)));
    // Get and return received data from buffer
    return UDR;
}

// Receive a string from USART until newline, carriage return, or buffer full
// Returns number of characters received
int USART_ReceiveString(char* buffer, int max_len, unsigned long timeout_ms) {
    int i = 0;
    // (void)timeout_ms; // Suppress unused variable warning if not implementing full timeout

    while (i < max_len - 1) { // Leave space for null terminator
        if (UCSRA & (1 << RXC)) { // Data received
            char received_char = UDR;
            buffer[i++] = received_char;
            // Re-enabled the newline/carriage return check for line-based responses
            if (received_char == '\n' || received_char == '\r') {
                break;
            }
        }
        _delay_us(100); // Small delay to prevent busy-waiting too hard
    }
    buffer[i] = '\0'; // Null-terminate the string
    return i; // Return number of characters received
}

// --- HC-05 Control Functions ---

void HC05_EnterATMode() {
    HC05_KEY_DDR |= (1 << HC05_KEY_PIN);
    HC05_KEY_PORT |= (1 << HC05_KEY_PIN);
    _delay_ms(100);

    USART_Init(MYUBRR_AT);
    _delay_ms(500);
}

void HC05_ExitATMode() {
    HC05_KEY_PORT &= ~(1 << HC05_KEY_PIN);
    _delay_ms(100);

    USART_Init(MYUBRR_DATA);
    _delay_ms(500);
}

// --- Main Program ---
int main(void) {
    // Set HC-05 KEY/EN pin as output and low initially (Data Mode)
    HC05_KEY_DDR |= (1 << HC05_KEY_PIN);
    HC05_KEY_PORT &= ~(1 << HC05_KEY_PIN);

    // Set STATUS_LED_PIN as output and low initially
    STATUS_LED_DDR |= (1 << STATUS_LED_PIN);
    STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN); // Turn off LED initially
    

    // Start USART in data mode baud rate
    USART_Init(MYUBRR_DATA);

    char hc05_response[100]; // Buffer to store HC-05's AT command response
    char app_message[128];   // Buffer to send to the mobile app

    while (1) {
        // --- 1. Enter AT Command Mode ---
        HC05_EnterATMode();
        _delay_ms(100);

        // --- 2. Send "AT" command to HC-05 ---
        USART_SendString("AT\r\n");
        _delay_ms(50); // Short delay for HC-05 to process

        // --- 3. Store the "AT" response ---
        int bytes_received = USART_ReceiveString(hc05_response, sizeof(hc05_response), 1000);
        if (bytes_received > 0) {
            hc05_response[bytes_received] = '\0'; // Null-terminate
        } else {
            strcpy(hc05_response, "NO RESPONSE\r\n"); // Default if no response
        }
        _delay_ms(100);

        // --- Check response and control LED ---
        // Use strstr to check for "OK" within the response string
        if (strstr(hc05_response, "OK") != NULL) {
            STATUS_LED_PORT |= (1 << STATUS_LED_PIN); // Turn PB1 HIGH (LED ON)
        } else {
            STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN); // Turn PB1 LOW (LED OFF)
        }
        _delay_ms(500); // Keep LED state for a moment

        // --- 4. Exit AT Command Mode ---
        HC05_ExitATMode();
        _delay_ms(100);
        STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN); // Turn PB1 LOW (LED OFF)

        // --- 5. Send the response to the mobile app ---
        sprintf(app_message, "AT_RESP:%s\r\n", hc05_response);
        USART_SendString(app_message);
        _delay_ms(2000); // Wait 2 seconds before repeating
    }

    return 0;
}