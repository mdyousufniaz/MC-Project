#define F_CPU 8000000UL // Define CPU frequency as 8MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // Required for ISR, sei()
#include <stdio.h>         // Required for sprintf
#include <stdbool.h>       // Required for bool type
#include <stdlib.h>        // Required for atoi
#include <string.h>        // Required for strcmp, strcpy

// --- USART Configuration ---
#define HC05_DATA_BAUD 9600
#define MYUBRR_DATA (F_CPU / 16 / HC05_DATA_BAUD) - 1

// --- Servo Motor Pin Definition (Timer1, OC1A) ---
#define SERVO_PWM_DDR DDRD
#define SERVO_PWM_PIN PD5 // Connect Servo signal pin to PD5 (OC1A for Timer1 PWM)

// --- DC Motor PWM Pin Definition (Timer0, OC0) ---
// Connect this to your L298N's ENA and/or ENB pin(s)
// (If using both ENA/ENB, tie them together and connect to this pin)
#define MOTOR_PWM_DDR DDRB
#define MOTOR_PWM_PIN PB3 // Connect Motor PWM to PB3 (OC0 for Timer0 PWM)

// --- Ultrasonic Sensor Pin Definitions ---
// Trigger pin: PD3
#define ULTRASONIC_TRIG_PORT PORTD
#define ULTRASONIC_TRIG_DDR DDRD
#define ULTRASONIC_TRIG_PIN PD3

// Echo pin: PD2 (External Interrupt 0 - INT0)
#define ULTRASONIC_ECHO_DDR DDRD
#define ULTRASONIC_ECHO_PIN PD2

// --- Motor Direction Control Pins (Assuming L298N or similar) ---
// Adjust these based on your actual motor driver connections to PORTA
#define MOTOR_DDR DDRA
#define MOTOR_PORT PORTA
// Example: IN1=PA0, IN2=PA1, IN3=PA2, IN4=PA3
#define M1_IN1 PA0 // Motor 1 (Left) Forward
#define M1_IN2 PA1 // Motor 1 (Left) Backward
#define M2_IN1 PA2 // Motor 2 (Right) Forward
#define M2_IN2 PA3 // Motor 2 (Right) Backward

// --- Automation Thresholds ---
#define OBSTACLE_THRESHOLD_CM 25
#define AUTOMATION_CLEAR_FACTOR 0.5

// --- Global Variables for USART Receive ---
volatile char received_command[15];
volatile uint8_t rx_buffer_idx = 0;
volatile bool new_command_available = false;

// --- Global Variables for Ultrasonic Measurement (USING TIMER2) ---
volatile uint16_t pulse_duration_us = 0;
volatile uint16_t timer2_overflow_counter = 0;
volatile uint8_t echo_edge_count = 0;
volatile bool new_distance_available = false;
volatile uint16_t current_front_distance_cm = 0;

// --- Global Variables for Motor Speed Control ---
volatile uint8_t motor_speed_percent = 0; // 0-100%

// --- Global Variables for Automation Mode ---
volatile bool automation_active = false;

// --- Automation States ---
typedef enum {
    AUTOMATION_STATE_IDLE,
    AUTOMATION_STATE_FORWARD,
    AUTOMATION_STATE_SCANNING,
    AUTOMATION_STATE_TURNING_LEFT,
    AUTOMATION_STATE_TURNING_RIGHT
} automation_state_t;

volatile automation_state_t current_automation_state = AUTOMATION_STATE_IDLE;

// --- Global Variables for Servo Control ---
#define SERVO_LEFT_POS 2099
#define SERVO_CENTER_POS 1199
#define SERVO_RIGHT_POS 449

// --- Function Prototypes ---
void USART_Init(unsigned int ubrr);
void USART_Transmit(char data);
void USART_SendString(const char *str);
void init_servo_pwm_timer();
void servo_enable_pwm_output();
void servo_disable_pwm_output();
void set_servo_position(uint16_t ocr_value);
void init_motor_pwm_timer0(); // New function for DC motor PWM
void set_motor_speed(uint8_t speed_percent); // New function to set motor speed
void init_timer2_ultrasonic_pulse_measure();
void init_external_interrupt0();
void motor_stop();
void motor_forward();
void motor_backward();
void motor_turn_left();
void motor_turn_right();
uint16_t get_distance_cm(void);

// --- USART Initialization ---
void USART_Init(unsigned int ubrr) {
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    UCSRC = (1 << URSEL) | (3 << UCSZ0);
}

// --- USART Transmit Character ---
void USART_Transmit(char data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

// --- USART Print String ---
void USART_SendString(const char* str) {
    while (*str != '\0') {
        USART_Transmit(*str);
        str++;
    }
}

// --- Servo Motor PWM Initialization (Timer1) ---
void init_servo_pwm_timer() {
    SERVO_PWM_DDR |= (1 << SERVO_PWM_PIN);
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);
    TCCR1B |= (1 << CS11); // Prescaler to 8 (1 tick = 1 microsecond)
    ICR1 = 19999;          // TOP for 50Hz (20ms) period
}

// --- Enable OC1A PWM Output ---
void servo_enable_pwm_output() {
    TCCR1A |= (1 << COM1A1);
}

// --- Disable OC1A PWM Output ---
void servo_disable_pwm_output() {
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
}

// --- Set Servo Position and then turn off PWM ---
void set_servo_position(uint16_t ocr_value) {
    // if (ocr_value < SERVO_LEFT_POS) ocr_value = SERVO_LEFT_POS;
    // if (ocr_value > SERVO_RIGHT_POS) ocr_value = SERVO_RIGHT_POS;

    servo_enable_pwm_output();
    OCR1A = ocr_value;
    _delay_ms(200);
    servo_disable_pwm_output();
}

// --- DC Motor PWM Initialization (Timer0) ---
void init_motor_pwm_timer0() {
    MOTOR_PWM_DDR |= (1 << MOTOR_PWM_PIN); // Set PB3 (OC0) as output
    // Configure Timer0 for Fast PWM mode (WGM01, WGM00)
    TCCR0 |= (1 << WGM01) | (1 << WGM00);
    // Set non-inverting mode for OC0 (PWM output is high when TCNT0 < OCR0)
    TCCR0 |= (1 << COM01);
    // Set Prescaler to 8. This gives a PWM frequency of F_CPU / (8 * 256) = 8MHz / 2048 = ~3.9 kHz
    TCCR0 |= (1 << CS01);
    // Initial speed to 0%
    OCR0 = 0;
}

// --- Set DC Motor Speed (0-100%) ---
void set_motor_speed(uint8_t speed_percent) {
    if (speed_percent > 100) speed_percent = 100;
    motor_speed_percent = speed_percent; // Store for automation/manual mode consistency
    // Calculate OCR0 value based on percentage (0-255 for 8-bit timer)
    // OCR0 = (speed_percent * 255) / 100;
    OCR0 = (uint8_t)(((uint16_t)speed_percent * 255) / 100); // Ensures correct calculation
}


// --- Initialize Timer2 for Ultrasonic Pulse Measurement ---
void init_timer2_ultrasonic_pulse_measure() {
    TCCR2 = 0;   // Clear Timer2 settings (Normal mode by default)
    ASSR &= ~(1 << AS2); // Ensure Timer2 uses system clock
}

// --- Initialize External Interrupt 0 (INT0) for Echo Pin (PD2) ---
void init_external_interrupt0() {
    ULTRASONIC_ECHO_DDR &= ~(1 << ULTRASONIC_ECHO_PIN); // Set PD2 (Echo) as input
    GICR |= (1 << INT0);     // Enable External Interrupt 0
    MCUCR |= (1 << ISC00);   // Trigger INT0 on ANY logical change (rising or falling edge)
}

// --- Motor direction control functions ---
void motor_stop() {
    MOTOR_PORT = 0x00; // Turn off direction pins
    OCR0 = 0;
}
void motor_forward() {
    MOTOR_PORT = (1 << M1_IN1) | (1 << M2_IN1); // Left Forward, Right Forward
    set_motor_speed(motor_speed_percent); // Apply current speed
}
void motor_backward() {
    MOTOR_PORT = (1 << M1_IN2) | (1 << M2_IN2); // Left Backward, Right Backward
    set_motor_speed(motor_speed_percent); // Apply current speed
}

void motor_turn_left() {
    MOTOR_PORT = (1 << M1_IN2) | (1 << M2_IN1); // Left Backward, Right Forward (Spins Left)
    set_motor_speed(motor_speed_percent); // Apply current speed
}

void motor_turn_soft_left() {
    MOTOR_PORT = (1 << M2_IN1);// | (1 << M2_IN1); // Left Backward, Right Forward (Spins Left)
    set_motor_speed(motor_speed_percent); // Apply current speed
}

void motor_turn_right() {
    MOTOR_PORT = (1 << M1_IN1) | (1 << M2_IN2); // Left Forward, Right Backward (Spins Right)
    set_motor_speed(motor_speed_percent); // Apply current speed
}

void motor_turn_soft_right() {
    MOTOR_PORT = (1 << M1_IN1);// | (1 << M2_IN2); // Left Forward, Right Backward (Spins Right)
    set_motor_speed(motor_speed_percent); // Apply current speed
}

// --- Get Latest Distance Measurement ---
uint16_t get_distance_cm(void) {
    uint16_t dist;
    // cli(); // Disable interrupts for atomic read
    
    _delay_ms(1000); // Wait for 100ms
    ULTRASONIC_TRIG_PORT |= (1 << ULTRASONIC_TRIG_PIN); // Set Trig pin high
    _delay_us(15);                                      // Keep high for 15us
    ULTRASONIC_TRIG_PORT &= ~(1 << ULTRASONIC_TRIG_PIN); // Set Trig pin low
     
    while(!new_distance_available);

    cli();
    dist = pulse_duration_us / 58;
    new_distance_available = false;
    sei();
    // sei(); // Re-enable interrupts
    return dist;
}

// --- ISRs ---

// USART Receive Complete Interrupt Service Routine (ISR)
ISR(USART_RXC_vect) {
    char received_char = UDR;
    if (received_char == '\n' || received_char == '\r') {
        if (rx_buffer_idx > 0) {
            received_command[rx_buffer_idx] = '\0';
            new_command_available = true;
        }
        rx_buffer_idx = 0;
    } else if (rx_buffer_idx < (sizeof(received_command) - 1)) {
        received_command[rx_buffer_idx++] = received_char;
    }
}

// External Interrupt 0 (INT0) ISR for Ultrasonic Echo Pin (PD2)
ISR(INT0_vect) {
    if (echo_edge_count == 0) { // First edge (rising edge of echo pulse)
        TCNT2 = 0;                     // Reset Timer2 counter
        timer2_overflow_counter = 0;   // Reset overflow counter
        TIMSK |= (1 << TOIE2);         // Enable Timer2 Overflow Interrupt
        TCCR2 |= (1 << CS21);          // Start Timer2 with prescaler /8 (1 tick = 1 microsecond)
        echo_edge_count = 1;           // Set flag to indicate we are waiting for the falling edge
    } else { // Second edge (falling edge of echo pulse)
        TCCR2 = 0;                     // Stop Timer2
        TIMSK &= ~(1 << TOIE2);        // Disable Timer2 Overflow Interrupt

        pulse_duration_us = (uint16_t)timer2_overflow_counter * 256 + TCNT2;

        echo_edge_count = 0;
        new_distance_available = true;
    }
}

// Timer2 Overflow Interrupt Service Routine for Ultrasonic Pulse Measurement
ISR(TIMER2_OVF_vect) {
    timer2_overflow_counter++;
}

uint16_t measure_distance_at_position(uint16_t servo_pos);

// NEW FUNCTION IMPLEMENTATION
uint16_t measure_distance_at_position(uint16_t servo_pos) {
    set_servo_position(servo_pos); // Move servo and wait for it to settle
    _delay_ms(300); // Give it a bit more time to settle and for any old echoes to clear.

    // Prepare for a new measurement
    cli(); // Disable interrupts during setup for atomic operation
    new_distance_available = false; // Clear the flag, we're waiting for a NEW one
    sei(); // Re-enable interrupts

    // Trigger the ultrasonic sensor
    ULTRASONIC_TRIG_PORT |= (1 << ULTRASONIC_TRIG_PIN); // Set Trig pin high
    _delay_us(15);                                      // Keep high for 15us
    ULTRASONIC_TRIG_PORT &= ~(1 << ULTRASONIC_TRIG_PIN); // Set Trig pin low

    // Wait for the new distance measurement to be available (blocking wait for THIS ping)
    uint16_t timeout_counter = 0;
    const uint16_t MEASUREMENT_TIMEOUT_MS = 200; // Max time for echo (e.g., ~3.5m = 20ms round trip)

    // Wait with a timeout to prevent infinite loop if no echo returns
    while (!new_distance_available && timeout_counter < MEASUREMENT_TIMEOUT_MS) {
        _delay_ms(1); // Wait for the ISR to process the echo
        timeout_counter++;
    }

    if (new_distance_available) { // Check if we actually got a new reading
        cli();
        uint16_t measured_distance = pulse_duration_us / 58; // Read the value from the ISR
        new_distance_available = false; // Reset flag for the *next* measurement
        sei();
        return measured_distance;
    } else {
        // Handle timeout or no measurement (e.g., sensor out of range, blocked, or error)
        return 9999; // Return a very large value to indicate no valid measurement
    }
}

// --- Main Program ---
int main(void) {
    // --- Initialize Peripherals ---
    MOTOR_DDR = 0xFF; // Set PORTA as output for motor direction control
    motor_stop();     // Initialize motors off (direction and speed)

    // Ultrasonic Trigger Pin setup (PD3)
    ULTRASONIC_TRIG_DDR |= (1 << ULTRASONIC_TRIG_PIN);

    init_servo_pwm_timer();
    servo_disable_pwm_output(); // Ensure servo PWM is off from the start
    set_servo_position(SERVO_CENTER_POS); // Move to center and turn off PWM

    init_motor_pwm_timer0(); // Initialize Timer0 for DC Motor PWM
    set_motor_speed(70);      // Set initial motor speed to 0%

    USART_Init(MYUBRR_DATA);
    init_timer2_ultrasonic_pulse_measure();
    init_external_interrupt0();

    sei(); // Enable global interrupts

    uint16_t left_distance_cm = 0;
    uint16_t right_distance_cm = 0;
    uint16_t max_clear_distance = 0;
    uint16_t target_forward_distance = 0;

    // For Ultrasonic Triggering (moved to polling in main loop)
    const unsigned long ULTRASONIC_TRIGGER_DELAY_MS = 100; // Trigger every 100ms

    // Main loop: Continuously check flags and execute logic
    while (1) {
        // --- 1. Process Incoming Bluetooth Commands ---
        if (new_command_available) {
            cli(); // Disable interrupts for atomic access to received_command
            char cmd_copy[sizeof(received_command)];
            strcpy(cmd_copy, (char*)received_command);
            new_command_available = false; // Clear flag
            sei(); // Re-enable interrupts

            // Check if command is a speed setting (e.g., "Speed:50\n")
            if (strncmp(cmd_copy, "Speed:", 6) == 0) {
                // Extract number after "Speed:"
                uint8_t speed_val = atoi(cmd_copy + 6);
                set_motor_speed(speed_val);
                char speed_ack[20];
                sprintf(speed_ack, "SpeedSet:%u\r\n", speed_val);
                USART_SendString(speed_ack);
            }
            // Handle "Automate" command to toggle automation mode
            else if (strcmp(cmd_copy, "Automate") == 0) {
                automation_active = !automation_active;
                motor_stop(); // Always stop motors and reset speed when changing automation state
                set_servo_position(SERVO_CENTER_POS);
                if (automation_active) {
                    current_automation_state = AUTOMATION_STATE_FORWARD;
                    USART_SendString("Automation ON\r\n");
                    // set_motor_speed(50); // Set default speed for automation, e.g., 50%
                } else {
                    current_automation_state = AUTOMATION_STATE_IDLE;
                    USART_SendString("Automation OFF\r\n");
                }
            } else if (!automation_active) {
                // --- Manual Control Mode Logic ---
                // Process only if automation is NOT active
                if (strcmp(cmd_copy, "Forward") == 0) {
                    motor_forward();
                } else if (strcmp(cmd_copy, "Backward") == 0) {
                    motor_backward();
                } else if (strcmp(cmd_copy, "Left") == 0) {
                    motor_turn_left();
                } else if (strcmp(cmd_copy, "SoftLeft") == 0) {
                    motor_turn_soft_left();
                } else if (strcmp(cmd_copy, "Right") == 0) {
                    motor_turn_right();
                } else if (strcmp(cmd_copy, "SoftRight") == 0) {
                    motor_turn_soft_right();
                } else if (strcmp(cmd_copy, "Stop") == 0) {
                    motor_stop();
                }
            }
        }

        // --- 2. Ultrasonic Triggering (Polling in Main Loop) ---
        // This replaces the Timer0_OVF_vect for triggering
        // Using _delay_ms directly here makes the main loop blocking for 100ms.
        // For a non-blocking approach, you'd need a simple millis() implementation.
        // For this example, we'll use a simple blocking delay for the trigger.
        _delay_ms(ULTRASONIC_TRIGGER_DELAY_MS); // Wait for 100ms

        // Only trigger if no measurement is currently in progress (echo_edge_count == 0)
        if (echo_edge_count == 0) {
            ULTRASONIC_TRIG_PORT |= (1 << ULTRASONIC_TRIG_PIN); // Set Trig pin high
            _delay_us(15);                                      // Keep high for 15us
            ULTRASONIC_TRIG_PORT &= ~(1 << ULTRASONIC_TRIG_PIN); // Set Trig pin low
        }


        // --- 3. Process New Distance Measurement ---
        if (new_distance_available) {
            cli();
            current_front_distance_cm = pulse_duration_us / 58;
            new_distance_available = false;
            sei();

            char dist_str[15];
            sprintf(dist_str, "Dist:%u cm\r\n", current_front_distance_cm);
            USART_SendString(dist_str);
        }

        // --- 4. Automation Mode Logic ---
        if (automation_active) {
            switch (current_automation_state) {
                case AUTOMATION_STATE_FORWARD:
                    motor_forward(); // Will apply `motor_speed_percent` set in main loop or by app
                    if (current_front_distance_cm > 0 && current_front_distance_cm <= OBSTACLE_THRESHOLD_CM) {
                        motor_stop();
                        USART_SendString("Obstacle! Scanning...\r\n");
                        current_automation_state = AUTOMATION_STATE_SCANNING;
                    }
                    break;

                case AUTOMATION_STATE_SCANNING:
                    motor_stop();

                    left_distance_cm = measure_distance_at_position(SERVO_LEFT_POS);
                    USART_SendString("LeftDist:");
                    char left_dist_str[10];
                    sprintf(left_dist_str, "%u cm\r\n", left_distance_cm);
                    USART_SendString(left_dist_str);
                    
                    right_distance_cm = measure_distance_at_position(SERVO_RIGHT_POS);
                    USART_SendString("RightDist:");
                    char right_dist_str[10];
                    sprintf(right_dist_str, "%u cm\r\n", right_distance_cm);
                    USART_SendString(right_dist_str);

                    
                    // while(1) ;

                    set_servo_position(SERVO_CENTER_POS);
                    _delay_ms(1500); // Give time for echo to return

                    if (left_distance_cm > right_distance_cm) {
                        USART_SendString("Turning Left.\r\n");
                        motor_turn_soft_left();
                        current_automation_state = AUTOMATION_STATE_TURNING_LEFT;
                        max_clear_distance = left_distance_cm;
                    } else {
                        USART_SendString("Turning Right.\r\n");
                        motor_turn_soft_right();
                        current_automation_state = AUTOMATION_STATE_TURNING_RIGHT;
                        max_clear_distance = right_distance_cm;
                    }
                    target_forward_distance = (uint16_t)(max_clear_distance * AUTOMATION_CLEAR_FACTOR);
                    set_servo_position(SERVO_CENTER_POS);
                    _delay_ms(200);
                    break;

                case AUTOMATION_STATE_TURNING_LEFT:
                    motor_turn_soft_left(); // Ensure motor continues turning with current speed
                    if (current_front_distance_cm >= target_forward_distance && current_front_distance_cm > OBSTACLE_THRESHOLD_CM) {
                        USART_SendString("Path clear ahead. Moving Forward.\r\n");
                        current_automation_state = AUTOMATION_STATE_FORWARD;
                        motor_stop();
                        _delay_ms(500);
                    }
                    break;

                case AUTOMATION_STATE_TURNING_RIGHT:
                    motor_turn_soft_right(); // Ensure motor continues turning with current speed
                    if (current_front_distance_cm >= target_forward_distance && current_front_distance_cm > OBSTACLE_THRESHOLD_CM) {
                        USART_SendString("Path clear ahead. Moving Forward.\r\n");
                        current_automation_state = AUTOMATION_STATE_FORWARD;

                         motor_stop();
                        _delay_ms(500);
                    }
                    break;

                case AUTOMATION_STATE_IDLE:
                    motor_stop();
                    break;
            }
        }
        // No additional _delay_ms(10) at the end of the loop, as the USS trigger delay
        // already provides a blocking delay.
    }
    return 0;
}