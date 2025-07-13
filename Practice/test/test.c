#include <avr/io.h>
// #define F_CPU 1000000UL
// #include <util/delay.h>

// int main(void)
// {
//     unsigned char c = 0;
//     DDRB = 1;

//     while (1)
//     {
//         PORTB = c;
//         c = c ? 0 : 1;
//         _delay_ms(1000);
//     }

//     return 0;
// }

// // avrdude/avrdude.exe -C"avrdude/avrdude.conf" -v -pm32 -cusbasp -U flash:w:your_program.hex:i -U lfuse:w:0xE4:m -U hfuse:w:0x99:m

#define F_CPU 8000000UL // Define CPU frequency as 8MHz (Common default for ATmega32)
                        // IMPORTANT: Ensure this matches your actual ATmega32 hardware clock speed.
                        // If using internal 1MHz, change to 1000000UL.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // Required for ISR
#include <stdio.h>         // Required for sprintf

// --- UART Configuration ---
#define BAUD 9600
#define MYUBRR (F_CPU / 16 / BAUD) - 1

// --- Ultrasonic Sensor Pin Definitions ---
// Trigger pin: PD3
#define ULTRASONIC_TRIG_PORT PORTD
#define ULTRASONIC_TRIG_DDR DDRD
#define ULTRASONIC_TRIG_PIN PD3

// Echo pin: PD2 (External Interrupt 0 - INT0)
#define ULTRASONIC_ECHO_DDR DDRD
#define ULTRASONIC_ECHO_PIN PD2

// --- Global Variables for Ultrasonic Measurement ---
volatile int pulse = 0; // Stores the raw timer count for the pulse duration
volatile int i = 0;     // Flag to track edge detection in ISR (0 for first edge, 1 for second)
volatile uint8_t new_distance_available = 0; // Flag set by INT0 ISR when a new measurement is ready

char dist_buffer[20]; // Buffer for distance string

// --- UART Initialization ---
void UART_init(void) {
    // Set baud rate
    UBRRH = (unsigned char)((MYUBRR) >> 8);
    UBRRL = (unsigned char)MYUBRR;
    // Enable receiver and transmitter
    UCSRB = (1 << RXEN) | (1 << TXEN);
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

// --- UART Receive Character (Non-blocking) ---
// Returns the received character if available, otherwise returns 0.
unsigned char UART_receive(void) { // Renamed to match your original, but it's non-blocking
    if (UCSRA & (1 << RXC)) { // Check if data is received
        return UDR; // Get and return received data from buffer
    }
    return 0; // No data available
}

// --- UART Transmit Character ---
void UART_transmit(char data) {
    while (!(UCSRA & (1 << UDRE))); // Wait for empty transmit buffer
    UDR = data; // Put data into buffer, sends the data
}

// --- UART Print String ---
void UART_print_string(char* str) {
    while (*str != '\0') {
        UART_transmit(*str);
        str++;
    }
}

// --- Motor control functions ---
void motor_stop() { PORTA = 0x00; } // All motors off
void motor_forward() { PORTA = 5; }
void motor_backward() { PORTA = 0xA; }
void motor_turn_left() { PORTA = 4; }
void motor_turn_right() { PORTA = 1; }

// --- Function to Initialize Timer0 for Periodic Triggering ---
// This will set up Timer0 to overflow periodically and trigger an interrupt.
volatile uint8_t timer0_overflow_count = 0;
#define ULTRASONIC_TRIGGER_INTERVAL_MS 100 // Trigger ultrasonic every 100ms
// Calculate Timer0 overflow period: (256 counts * Prescaler) / F_CPU_Hz
// For F_CPU = 8MHz, Prescaler = 1024: (256 * 1024) / 8,000,000 = 0.032768 seconds = ~32.768 ms
#define TIMER0_OVERFLOW_PERIOD_MS (256.0 * 1024.0 / F_CPU * 1000.0)
// Calculate how many overflows are needed for the desired interval
#define OVERFLOWS_PER_TRIGGER (uint8_t)(ULTRASONIC_TRIGGER_INTERVAL_MS / TIMER0_OVERFLOW_PERIOD_MS)

void init_timer0() {
    // Configure Timer0 for Normal Mode (WGM01:0 = 00)
    TCCR0 = 0;
    // Set Timer0 prescaler to 1024 (CS02=1, CS00=1)
    TCCR0 |= (1 << CS02) | (1 << CS00);
    // Enable Timer0 Overflow Interrupt
    TIMSK |= (1 << TOIE0);
}

// --- Interrupt Service Routine for External Interrupt 0 (INT0) ---
// This ISR is triggered by changes on the Echo pin (PD2).
ISR(INT0_vect) {
    if (i == 0) { // First edge (rising edge of echo pulse)
        TCNT1 = 0;          // Reset Timer1 counter
        TCCR1B |= (1 << CS10); // Start Timer1 with no prescaler (1 tick per microsecond)
        i = 1;              // Set flag to indicate we are waiting for the falling edge
    } else { // Second edge (falling edge of echo pulse)
        TCCR1B = 0;         // Stop Timer1
        pulse = TCNT1;      // Store the captured Timer1 value (pulse duration)
        TCNT1 = 0;          // Reset Timer1 for the next measurement
        i = 0;              // Reset edge counter
        new_distance_available = 1; // Signal that a new measurement is ready
    }
}

// --- Timer0 Overflow Interrupt Service Routine ---
// This ISR will periodically trigger the ultrasonic sensor.
ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
    if (timer0_overflow_count >= OVERFLOWS_PER_TRIGGER) {
        timer0_overflow_count = 0; // Reset counter

        // Only trigger if the previous measurement is complete (i == 0)
        // This prevents triggering while an echo pulse is still being measured.
        if (i == 0) {
            PORTD |= (1 << ULTRASONIC_TRIG_PIN); // Set Trig pin high
            _delay_us(15);                       // Keep high for 15us
            PORTD &= ~(1 << ULTRASONIC_TRIG_PIN); // Set Trig pin low
        }
    }
}

int main(void) {
    // --- Motor Control Setup ---
    DDRA = 0xFF; // Set PORTA as output for motor control
    PORTA = 0x00; // Initialize all PORTA pins low (motors off)

    // --- UART Setup ---
    UART_init(); // Initialize UART for serial communication

    // --- Ultrasonic Sensor Pin Setup ---
    ULTRASONIC_TRIG_DDR |= (1 << ULTRASONIC_TRIG_PIN); // Set PD3 (Trig) as output
    // Corrected: Set PD2 (Echo) as input. Use &= ~ to clear a bit.
    ULTRASONIC_ECHO_DDR &= ~(1 << ULTRASONIC_ECHO_PIN);

    // --- External Interrupt 0 (INT0) Setup on PD2 ---
    GICR |= (1 << INT0);    // Enable External Interrupt 0
    MCUCR |= (1 << ISC00);  // Trigger INT0 on ANY logical change (rising or falling edge)

    // --- Timer0 Setup for Periodic Ultrasonic Triggering ---
    init_timer0();

    sei(); // Enable global interrupts (essential for ISRs to work)

    uint16_t current_distance = 0; // Variable to store the calculated distance

    while (1) {
        // --- Motor Control Logic (Non-blocking) ---
        // Continuously check for commands without waiting
        unsigned char cmd = UART_receive(); // UART_receive is now non-blocking
        if (cmd != 0) { // If a command was received (not 0)
            switch (cmd) {
                case 'F': motor_forward(); break;
                case 'B': motor_backward(); break;
                case 'L': motor_turn_left(); break;
                case 'R': motor_turn_right(); break;
                case 'S': motor_stop(); break; // Stop command
                default: motor_stop(); break;
            }
        }

        // --- Check for New Distance Measurement (Non-blocking) ---
        // This part only processes if a new measurement is ready.
        if (new_distance_available) {
            // Disable interrupts briefly to read 'pulse' and 'new_distance_available' atomically.
            // This prevents the ISR from modifying 'pulse' while main is reading it.
            cli();
            current_distance = pulse / 58 / 8; // Calculate distance in centimeters
            new_distance_available = 0;    // Clear the flag
            sei(); // Re-enable interrupts

            // Print distance to Virtual Terminal
            sprintf(dist_buffer, "%u cm", current_distance);
            UART_print_string(dist_buffer);
        }

        // A small delay here is generally fine, but keep it minimal
        // as the main loop is now designed to be highly responsive.
        // If you need maximum responsiveness, you can remove this delay,
        // but be aware of potential busy-waiting if there's nothing else to do.
        _delay_ms(10);
    }

    return 0;
}
