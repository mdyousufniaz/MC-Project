#include <avr/io.h>
#define F_CPU 8000000UL // Define CPU frequency as 8MHz
#include <util/delay.h>
#include <stdlib.h> // For atoi()
#include <string.h> // For string manipulation (not strictly needed for this simple protocol)
#include <stdio.h>  // For sprintf (not strictly needed for this simple protocol)
#include <avr/interrupt.h> // For sei() and ISR()

// --- Define CPU frequency ---


// --- USART Baud Rate Definition ---
// IMPORTANT: This MUST match your HC-05's configured data mode baud rate.
// For controlling from an app, the HC-05 should be in Data Mode.
#define HC05_DATA_BAUD 9600

// Calculate UBRR value for USART
#define MYUBRR_DATA (F_CPU/16/HC05_DATA_BAUD)-1

// --- Servo Motor Pin Definition ---
#define SERVO_PWM_DDR DDRD
#define SERVO_PWM_PORT PORTD
#define SERVO_PWM_PIN PD5 // Connect Servo signal pin to PD5 (OC1A for Timer1 PWM)

// --- Global variables for USART receive buffer and new OCR1A value ---
// These are volatile because they are modified within an ISR and read in main.
volatile char rx_buffer[6]; // Max 4 digits for 2500 + newline + null terminator (e.g., "2500\n\0")
volatile uint8_t rx_buffer_idx = 0;
volatile uint8_t data_received_flag = 0; // Flag to indicate a complete value is received
volatile uint16_t new_ocr1a_value = 1499; // Initial servo position (neutral)

// --- Function to Initialize Servo Motor PWM ---
void init_servo_pwm() {
    // Set Servo PWM pin (PD5) as output
    SERVO_PWM_DDR |= (1 << SERVO_PWM_PIN);

    // Configure Timer1 for Fast PWM mode with ICR1 as TOP (Mode 14)
    // WGM13=1, WGM12=1, WGM11=1, WGM10=0
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    // Set Timer1 prescaler to 8 (CS11 = 1)
    // With F_CPU = 8MHz and Prescaler = 8, Timer1 increments every 1 microsecond.
    // (8,000,000 Hz / 8 = 1,000,000 ticks/second)
    TCCR1B |= (1 << CS11);

    // Set ICR1 for a 20ms (50Hz) PWM period
    // T_PWM = (1 + ICR1) * Prescaler / F_CPU
    // 0.02s = (1 + ICR1) * 8 / 8,000,000 Hz
    // 0.02s = (1 + ICR1) / 1,000,000 Hz
    // 1 + ICR1 = 20,000
    // ICR1 = 19999
    ICR1 = 19999; // TOP value for 50Hz PWM

    // Configure OC1A (PD5) to clear on Compare Match, set at TOP (Non-inverting mode)
    // COM1A1=1, COM1A0=0
    TCCR1A |= (1 << COM1A1);
    OCR1A =1199;
}

// --- Function to Initialize USART ---
void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSRB = (1 << RXEN) | (1 << TXEN);
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (3 << UCSZ0); // URSEL for ATmega32, UCSZ0/UCSZ1 for 8-bit data

    // Enable USART Receive Complete Interrupt
    UCSRB |= (1 << RXCIE);
}

// --- USART Receive Complete Interrupt Service Routine (ISR) ---
// This ISR is called automatically when a new byte is received via USART.
ISR(USART_RXC_vect) {
    char received_char = UDR; // Read the incoming byte

    // Check for newline or carriage return as end-of-message delimiters
    if (received_char == '\n' || received_char == '\r') {
        if (rx_buffer_idx > 0) { // Only process if there's actual data in the buffer
            rx_buffer[rx_buffer_idx] = '\0'; // Null-terminate the accumulated string
            data_received_flag = 1; // Set flag to indicate a complete message is ready
        }
        rx_buffer_idx = 0; // Reset buffer index for the next incoming value
    } else if (rx_buffer_idx < (sizeof(rx_buffer) - 1)) {
        // Store the character in the buffer if there's space
        rx_buffer[rx_buffer_idx++] = received_char;
    }
}

// --- Main Program ---
int main(void) {
    // Initialize servo PWM
    init_servo_pwm();

    // Initialize USART for communication with HC-05
    USART_Init(MYUBRR_DATA);

    // Enable global interrupts
    sei();

    while(1) {
        // // Check if a new complete value has been received from Bluetooth
        // if (data_received_flag) {
        //     // Convert the received string to an integer
        //     int received_int = atoi((char*)rx_buffer); // Cast to char* for atoi

        //     // Validate the received value to ensure it's within the desired range (1 to 2500)
        //     if (received_int >= 1 && received_int <= 2500) {
        //         new_ocr1a_value = (uint16_t)received_int; // Update the servo position
        //         OCR1A = new_ocr1a_value; // Apply the new PWM value
        //         _delay_ms(100);
        //         OCR1A = 0;
        //     }
        //     // Clear the flag to indicate that the data has been processed
        //     data_received_flag = 0;
        // }

        OCR1A = 999;
        _delay_ms(1500);

        OCR1A = 449;
        _delay_ms(1500);

        OCR1A = 1199;
        _delay_ms(1500);

        OCR1A = 1999;
        _delay_ms(1500);

        OCR1A = 1199;
        _delay_ms(1500);

        // The main loop can do other tasks here if needed,
        // but for continuous servo update, we just wait for new data via interrupt.
    }

    return 0; // This line is unreachable due to while(1)
}