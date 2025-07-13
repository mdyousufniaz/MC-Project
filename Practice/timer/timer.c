#include <avr/io.h>
#include <avr/interrupt.h>

volatile int overflowCount;

ISR(TIMER0_OVF_vect)
{
    if (++overflowCount == 3907) {
        PORTB = PORTB ? 0 : 1;
        overflowCount = 0;
    }
}

int main(void)
{
    overflowCount = 0;
    DDRB = 1;
    PORTB = 0;

    TCCR0 = 1;
    TIMSK = 1;

    sei();
    while(1) {}
    return  0;
}