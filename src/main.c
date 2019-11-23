#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define AH_BIT PD3
#define AL_BIT PD2
#define AH_PORT PORTD
#define AL_PORT PORTD

#define CH_BIT PD5
#define CL_BIT PB1
#define CH_PORT PORTD
#define CL_PORT PORTB

#define RED_LED_BIT PC3
#define RED_LED_PORT PORTC

#define GREEN_LED_BIT PC2
#define GREEN_LED_PORT PORTC

#define RC_IN_BIT PB0
#define RC_IN_PIN PINB

#define RED_ON() (RED_LED_PORT &= ~(1 << RED_LED_BIT))
#define RED_OFF() (RED_LED_PORT |= (1 << RED_LED_BIT))
#define GREEN_ON() (GREEN_LED_PORT &= ~(1 << GREEN_LED_BIT))
#define GREEN_OFF() (GREEN_LED_PORT |= (1 << GREEN_LED_BIT))


// 1000us
#define PULSE_WIDTH_FLOOR 16000
// 2000us
#define PULSE_WIDTH_CEIL 32000

#define PULSE_WIDTH_MAX 35000 
#define PULSE_WIDTH_MIN 13000

#define PULSE_WIDTH_NEUTRAL ((PULSE_WIDTH_CEIL + PULSE_WIDTH_FLOOR) / 2)
#define PULSE_WIDRH_DEADBAND 1000

void io_init() {
    DDRB &= ~(_BV(RC_IN_BIT));
    DDRC |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT);

    GREEN_OFF();
    RED_OFF();
}

void icp_timer_init() {
    // enable icp interrupt and overflow interrupt
    TIMSK |= _BV(TICIE1) | _BV(TOIE1);

    // set the interrupt capture edge to high, and start the timer with no pre-scaler
    TCCR1B |=  _BV(ICES1) | _BV(CS10);
}

uint8_t overflow_count = 0;

void set_power(int16_t);

ISR (TIMER1_CAPT_vect) {
    // 1 = waiting for high, 0 = waiting for low
    static uint8_t edge = 1;

    if (edge) {
        // reset counter
        TCNT1 = 0;
        // change to low interrupt

        TCCR1B &= ~_BV(ICES1);
    } else if (overflow_count) {
        // pulse lasted more than 1 overflow (1/1600000*2^16 seconds = 4096us) 
        set_duty(0);
    } else {
        int32_t time = TCNT1;
        int32_t duty = 0;

        if (time < PULSE_WIDTH_MIN || time > PULSE_WIDTH_MAX) {
            duty = 0;
        } else if (time >= PULSE_WIDTH_CEIL) {
            duty = 255;
        } else if (time <= PULSE_WIDTH_FLOOR) {
            duty = 0;
        } else {
            duty = 510 * (PULSE_WIDTH_NEUTRAL - time) / (PULSE_WIDTH_CEIL - PULSE_WIDTH_FLOOR);
        }

        set_power(duty);
    }

    overflow_count = 0;

    // toggle interrupt edge
    TCCR1B ^= _BV(ICES1);

    edge = !edge;
}

// 1/16000000*2^16 = ~4ms, normal period is 20ms, timeout after 100ms
ISR (TIMER1_OVF_vect) {
    if (overflow_count > 25)
        set_power(0);
    else
        overflow_count = 0;
}

void pwm_timer_init() {
    // enable interrupt for overflow and comp match on timer 2 (For output pwm)
    TIMSK |= _BV(TOIE2) | _BV(OCIE2);

    // start timer 2 with no prescaler
    TCCR2 |= _BV(CS20);
}


enum {
    DIR_FWD,
    DIR_STOPPED,
    DIR_REV
} direction = DIR_STOPPED;

// output pwm on
ISR (TIMER2_OVF_vect) {
    switch (direction) {
        case DIR_FWD:
            GREEN_OFF();
            RED_ON();
            break;

        case DIR_STOPPED:
            GREEN_OFF();
            RED_OFF();
            break;

        case DIR_REV:
            RED_OFF();
            GREEN_ON();
            break;
    }
}

// output pwn off
ISR (TIMER2_COMP_vect) {
    RED_OFF();
    GREEN_OFF();
}

void set_duty(uint8_t duty) {
    OCR2 = duty;
}

void set_power(int16_t power) {
    cli();
    if (power > 0) {
        direction = DIR_FWD;
        set_duty(power);
    } else if (power < 0) {
        direction = DIR_REV;
        set_duty(-power);
    } else {
        direction = DIR_STOPPED;
        set_duty(0);
    }
    sei();
}

int main() {
    io_init();
    icp_timer_init();
    pwm_timer_init();

    sei();

    set_duty(0);

    while (1) {}
}
