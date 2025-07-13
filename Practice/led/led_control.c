#include <avr/io.h>     // Standard AVR I/O definitions (DDRx, PORTx, PINx)
#include <util/delay.h> // For _delay_ms()
#include <avr/interrupt.h> // For UART Receive Complete Interrupt

// --- Configuration Defines ---
#define F_CPU 8000000UL // CPU Clock Frequency - IMPORTANT: Match your microcontroller's clock! (e.g., 8 MHz)
#define BAUD 9600       // UART Baud Rate - IMPORTANT: Match your Bluetooth module's baud rate!
// Calculated UBRR register value for the given F_CPU and BAUD rate.
// This formula is standard for asynchronous normal mode.
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// Define the LED pin as PORTA, Pin 0 for clarity and easy modification.
#define LED_PORT PORTA
#define LED_DDR  DDRA
#define LED_PIN  PA0 // PA0 corresponds to PORTA, bit 0

// --- Global Variable ---
// This variable is used to store the character received via UART.
// 'volatile' keyword is crucial because it's modified by an ISR
// and accessed by the main loop, preventing compiler optimizations that might
// lead to incorrect behavior.
volatile char received_command = 0; // Stores the last received character. Renamed for clarity.

// --- Function Prototypes ---
void uart_init(void);
void uart_transmit(unsigned char data); // Added for optional serial feedback/debugging

// --- Main Function ---
int main(void) {
    // 1. Initialize UART for serial communication.
    uart_init();

    // 2. Configure the LED pin (PORTA.0) as an output.
    // Setting the corresponding bit in the Data Direction Register (DDR) to 1 makes it an output.
    LED_DDR |= (1 << LED_PIN); // Set PA0 as output

    // 3. Enable global interrupts.
    // This is essential for the UART Receive Complete Interrupt Service Routine (ISR) to trigger.
    sei(); // set enable interrupts (enable Global Interrupt Flag in SREG)

    // Optional: Send an initial message to the virtual terminal for debugging.
    uart_transmit('S'); // S for Start
    uart_transmit('T');
    uart_transmit('A');
    uart_transmit('R');
    uart_transmit('T');
    uart_transmit('\r'); // Carriage Return
    uart_transmit('\n'); // Newline

    // --- Main Loop ---
    // The microcontroller will continuously execute code within this loop.
    while (1) {
        // Check the value of the received_command variable.
        // This variable is updated by the UART receive interrupt.
        if (received_command == '1') {
            // If '1' is received, turn the LED ON.
            // Setting the corresponding bit in the PORT register to 1 drives the pin high.
            LED_PORT |= (1 << LED_PIN);
            // Optional: Send feedback to the serial monitor.
            uart_transmit('L'); uart_transmit('O'); uart_transmit('\r'); uart_transmit('\n');
            // Reset the command variable to 0.
            // This is important: it ensures the LED stays ON until a new command is received,
            // and prevents the '0' condition from being immediately triggered in the next loop.
            received_command = 0;
        } else if (received_command == '0') {
            // If '0' is received, turn the LED OFF.
            // Clearing the corresponding bit in the PORT register to 0 drives the pin low.
            LED_PORT &= ~(1 << LED_PIN);
            // Optional: Send feedback to the serial monitor.
            uart_transmit('L'); uart_transmit('F'); uart_transmit('\r'); uart_transmit('\n');
            // Reset the command variable to 0.
            received_command = 0;
        }

        // A small delay here is generally fine, but ensure it doesn't cause
        // the main loop to miss events if it becomes very long.
        // For this simple logic, 10ms is perfectly acceptable.
        _delay_ms(10);
    }

    // This line should theoretically never be reached in an infinite while(1) loop.
    return 0;
}

// --- UART Initialization Function ---
void uart_init(void) {
    // Set baud rate (UBRR registers). UBRRH is the high byte, UBRRL is the low byte.
    // Shifting UBRR_VALUE by 8 bits gets the high byte.
    UBRRH = (unsigned char)(UBRR_VALUE >> 8);
    UBRRL = (unsigned char)UBRR_VALUE;

    // Enable the UART Receiver (RXEN) and Transmitter (TXEN).
    // Also, enable the Receive Complete Interrupt (RXCIE) so that the ISR fires when data arrives.
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

    // Set frame format: 8 data bits, 1 stop bit, no parity.
    // For ATmega32, the URSEL bit must be set when writing to UCSRC, as it shares
    // an I/O address with UBRRH. (3 << UCSZ0) sets UCSZ1 and UCSZ0 to 1, for 8-bit data.
    UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

// --- UART Transmit Function (Blocking) ---
// This function sends a single character via UART. It waits until the transmit buffer
// is empty before putting new data into it.
void uart_transmit(unsigned char data) {
    // Wait for the Transmit Buffer Empty (UDRE) flag to be set.
    // UDRE is set when the UDR (UART Data Register) is ready to accept new data.
    while (!(UCSRA & (1 << UDRE))) {
        // Do nothing, just busy-wait.
    }
    // Put data into the UDR register. This automatically starts the transmission.
    UDR = data;
}

// --- UART Receive Complete Interrupt Service Routine (ISR) ---
// This ISR is automatically called by the hardware when a complete byte of data
// has been received by the UART.
ISR(USART_RXC_vect) { // For ATmega32, the vector name is USART_RXC_vect.
    // Read the received data from the UDR (UART Data Register) and store it.
    // Reading UDR automatically clears the RXC flag (Receive Complete).
    received_command = UDR;
}