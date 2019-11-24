#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// straight from afro_nfet.inc from simonk
#define AnFET 3
#define ApFET 2

#define AnFET_port PORTD
#define ApFET_port PORTD

#define CnFET 5
#define CpFET 1

#define CnFET_port PORTD
#define CpFET_port PORTB

#define ANFET_ON() (AnFET_port |= _BV(AnFET))
#define ANFET_OFF() (AnFET_port &= ~_BV(AnFET))

#define APFET_ON() (ApFET_port &= ~_BV(ApFET))
#define APFET_OFF() (ApFET_port |= _BV(ApFET))

#define CNFET_ON() (CnFET_port |= _BV(CnFET))
#define CNFET_OFF() (CnFET_port &= ~_BV(CnFET))

#define CPFET_ON() (CpFET_port &= ~_BV(CpFET))
#define CPFET_OFF() (CpFET_port |= _BV(CpFET))

#define RED_LED_BIT PC3
#define RED_LED_PORT PORTC

#define GREEN_LED_BIT PC2
#define GREEN_LED_PORT PORTC

#define RC_IN_BIT PB0
#define RC_IN_PIN PINB

#define RED_ON() (RED_LED_PORT &= ~_BV(RED_LED_BIT))
#define RED_OFF() (RED_LED_PORT |= _BV(RED_LED_BIT))
#define GREEN_ON() (GREEN_LED_PORT &= ~_BV(GREEN_LED_BIT))
#define GREEN_OFF() (GREEN_LED_PORT |= _BV(GREEN_LED_BIT))

// 1000us
#define PULSE_WIDTH_FLOOR 16000
// 2000us
#define PULSE_WIDTH_CEIL 32000

#define PULSE_WIDTH_MAX 35000 
#define PULSE_WIDTH_MIN 13000

#define PULSE_WIDTH_NEUTRAL 24000
#define PULSE_WIDTH_DEADBAND 800

// will enable within deadband
#define BRAKE_ENABLED
// 0 to 255
#define BRAKE_POWER 255

void io_init() {
    // todo make this more configurable
    PORTB = _BV(CpFET);
    DDRB = _BV(CpFET);

    PORTC = _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT);
    DDRC = _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT);

    PORTD = _BV(ApFET);
    DDRD = _BV(AnFET) | _BV(CnFET) | _BV(ApFET);
}

void icp_timer_init() {
    // enable icp interrupt and overflow interrupt
    TIMSK |= _BV(TICIE1) | _BV(TOIE1);

    // set the interrupt capture edge to high, and start the timer with no pre-scaler
    TCCR1B |=  _BV(ICES1) | _BV(CS10);
}

volatile uint8_t overflow_count = 0;

void set_power(int16_t);

ISR (TIMER1_CAPT_vect) {
    // 1 = waiting for high, 0 = waiting for low
    static uint8_t edge = 1;

    if (edge) {
        // reset counter
        TCNT1 = 0;
    } else if (overflow_count) {
        // pulse lasted more than 1 overflow (1/1600000*2^16 seconds = 4096us) 
        set_power(0);
    } else {
        int32_t time = TCNT1;
        int32_t duty = 0;

        if (
            time < PULSE_WIDTH_MIN || time > PULSE_WIDTH_MAX // out of range
            #ifdef PULSE_WIDTH_DEADBAND
            || (time > (PULSE_WIDTH_NEUTRAL - PULSE_WIDTH_DEADBAND/2) && time < (PULSE_WIDTH_NEUTRAL + PULSE_WIDTH_DEADBAND/2))
            #endif
        ) {
            duty = 0;
        } else if (time >= PULSE_WIDTH_CEIL) {
            duty = 255;
        } else if (time <= PULSE_WIDTH_FLOOR) {
            duty = -255;
        } else {
            duty = 510 * (time - PULSE_WIDTH_NEUTRAL) / (PULSE_WIDTH_CEIL - PULSE_WIDTH_FLOOR);
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
        overflow_count++;
}

void pwm_timer_init() {
    // enable interrupt for overflow and comp match on timer 2 (For output pwm)
    TIMSK |= _BV(TOIE2) | _BV(OCIE2);

    // start timer 2 with no prescaler
    TCCR2 |= _BV(CS20);
}

enum Direction {
    DIR_FWD,
    DIR_STOPPED,
    DIR_REV
};

volatile uint8_t duty_buffer = 0;

volatile enum Direction direction_buffer = DIR_STOPPED;
volatile enum Direction current_direction = DIR_STOPPED;

// output pwm on
ISR (TIMER2_OVF_vect) {
    #ifdef BRAKE_ENABLED
    OCR2 = direction_buffer == DIR_STOPPED ? BRAKE_POWER : duty_buffer;
    #else
    OCR2 = duty_buffer;
    #endif

    if (direction_buffer != current_direction) {
        // changing dircetions, turn everything off
        ANFET_OFF();
        APFET_OFF();
        CNFET_OFF();
        CPFET_OFF();
    }

    switch (direction_buffer) {
        case DIR_FWD:
            CNFET_ON(); // c low on (chopped)
            APFET_ON(); // a high on

            RED_ON();
            break;

        case DIR_STOPPED:
            GREEN_OFF();
            RED_OFF();

            #ifdef BRAKE_ENABLED
            CNFET_ON();
            ANFET_ON();
            #endif
            break;

        case DIR_REV:
            ANFET_ON(); // a low on (chopped)
            CPFET_ON(); // c high on

            GREEN_ON();
            break;
    }

    current_direction = direction_buffer;
}

// output pwn off
ISR (TIMER2_COMP_vect) {
    switch (current_direction) {
        case DIR_FWD:
            CNFET_OFF(); // c low off (chopped)
            RED_OFF();
            break;

        case DIR_STOPPED:
            #ifdef BRAKE_ENABLED
            CNFET_OFF();
            ANFET_OFF();
            #endif
            break;

        case DIR_REV:
            ANFET_OFF(); // a low off (chopped)
            GREEN_OFF();
            break;
    }
}

void set_duty(uint8_t duty) {
    duty_buffer = duty;
}

void set_power(int16_t power) {
    cli();
    if (power > 0) {
        direction_buffer = DIR_FWD;
        set_duty(power);
    } else if (power < 0) {
        direction_buffer = DIR_REV;
        set_duty(-power);
    } else {
        direction_buffer = DIR_STOPPED;
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
