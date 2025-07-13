// #include <avr/io.h>
// // #define F_CPU 16000000UL

// #include <util/delay.h>

// int main(void)
// {
//     DDRA = 0xFF;
//     unsigned char modes[] = {0x00, 0x06, 0x28, 0x1E, 0x2D};
//     unsigned char index = 0;
//     while (1)
//     {
//         PORTA = modes[index];
//         _delay_ms(100);
//         index = (index + 1) % 5;
//     }
// }

// void motor_stop() { PORTA = 0x00; } // All motors off
// void motor_forward() { PORTA = 0x1D; } // Based on your truth table
// void motor_backward() { PORTA = 0x2E; }
// void motor_turn_left() { // Left Bwd, Right Fwd
//     PORTA = 0x1E; // WAS 0x3C (Brake all)
// }

// void motor_turn_right() { // Left Fwd, Right Bwd
//     PORTA = 0x2D; // WAS 0x23 (Off all)
// }

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD)-1

void UART_init(void) {
    UBRRH = (unsigned char)((MYUBRR)>>8);
    UBRRL = (unsigned char)MYUBRR;
    UCSRB = (1<<RXEN)|(1<<TXEN);
    UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1); // 8-bit, 1 stop bit, no parity
}

unsigned char UART_receive(void) {
    while (!(UCSRA & (1<<RXC))); // Wait for data to be received
    return UDR; // Get and return received data from buffer
}

// Motor control functions
void motor_stop() { PORTA = 0x00; } // All motors off
void motor_forward() { PORTA = 5; } // Based on your truth table
void motor_backward() { PORTA = 0xA; }
void motor_turn_left() { // Left Bwd, Right Fwd
    PORTA = 4; // WAS 0x3C (Brake all)
}

void motor_turn_right() { // Left Fwd, Right Bwd
    PORTA = 1; // WAS 0x23 (Off all)
}
// Add pivot functions if needed

int main(void) {
    DDRA = 0xFF; // Set PORTA as output
    PORTA = 0x00; // Initialize all PORTA pins low
    UART_init();

    while (1) {
        unsigned char command = UART_receive(); // Wait for a command

        switch (command) {
            case 'F': motor_forward(); break;
            case 'B': motor_backward(); break;
            case 'L': motor_turn_left(); break;
            case 'R': motor_turn_right(); break;
            case 'S': motor_stop(); break; // Stop command
            // Add other cases for pivot, etc.
            default: motor_stop(); break; // Default to stop if unknown command
        }
    }
    return 0;
}