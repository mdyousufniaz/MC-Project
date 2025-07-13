#define F_CPU 8000000UL // Define CPU frequency as 8MHz
#include <avr/io.h>
#include <util/delay.h>

// --- Servo Motor Pin Definition ---
#define SERVO_PWM_DDR DDRD
#define SERVO_PWM_PORT PORTD
#define SERVO_PWM_PIN PD5 // Connect Servo signal pin to PD5 (OC1A for Timer1 PWM)

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
    TCCR1B |= (1 << CS11); // Changed from CS10 to CS11

    // Set ICR1 for a 20ms (50Hz) PWM period
    // T_PWM = (1 + ICR1) * Prescaler / F_CPU
    // 0.02s = (1 + ICR1) * 8 / 8,000,000 Hz
    // 0.02s = (1 + ICR1) / 1,000,000 Hz
    // 1 + ICR1 = 20,000
    // ICR1 = 19999
    ICR1 = 19999; // TOP value for 50Hz PWM - Remains the same

    // Configure OC1A (PD5) to clear on Compare Match, set at TOP (Non-inverting mode)
    // COM1A1=1, COM1A0=0
    TCCR1A |= (1 << COM1A1);
    OCR1A = 1499; // Set an initial position (e.g., neutral)
}
// --- Main Program ---
int main(void) {
    // Initialize servo PWM
    init_servo_pwm();

    while(1)
    {
        OCR1A = 999; // Corresponds to ~1.0ms pulse (0 degrees in your previous mapping)
        _delay_ms(1500);

        OCR1A = 1499; // Corresponds to ~1.5ms pulse (90 degrees / neutral)
        _delay_ms(1500);

        OCR1A = 1999; // Corresponds to ~2.0ms pulse (180 degrees)
        _delay_ms(1500);

        OCR1A = 1499; // Corresponds to ~1.5ms pulse (90 degrees / neutral)
        _delay_ms(1500);
    }

    return 0;
}
