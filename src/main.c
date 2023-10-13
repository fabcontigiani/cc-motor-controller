#include <LiquidCrystal.h>
#include <LiquidCrystal_Config.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

// Pines
#define P1 PD2      // P1 -> PD2: Encendido/apagado (interrupción externa)
#define P2 PD3      // P2 -> PD3: Alterna modo configuración (polling)
#define P3 PD4      // P3 -> PD4: Alterna T1/T2 en modo configuración (polling)
#define PIN_PWM PD5 // PinPWM -> 0C0B: Señal PWM de salida
#define RV1 PC0     // RV1 -> ADC0: Modificar tiempos T1/T2
#define RV2 PC1     // RV2 -> ADC1: Modificar ciclo útil
// Display LCD -> PB0:5

#define BOUNCE_DELAY 10 // ms
#define PWM_PERIOD 125  // 125 * 64 * (1/16MHz) = 500us -> f = 2kHz
#define LCD_WIDTH 16

uint8_t P1State;
uint8_t lastP1State = 1;
uint8_t P2State;
uint8_t lastP2State = 1;
uint8_t P3State;
uint8_t lastP3State = 1;
uint8_t motorIsOn = 0; // 0 = OFF; 1 = Stage 1; 2 = Stage 2
uint8_t lastMotorState = 0;
uint8_t configuringT1 = 1;
uint8_t duration1 = 20; // * 100ms
uint8_t duration2 = 39; // * 100ms
uint8_t dutyCycle1 = 40;
uint8_t dutyCycle2 = 95;
uint16_t pot1;
uint16_t pot2;
volatile uint8_t flag = 0;
volatile uint8_t counter = 0;
char buffer[LCD_WIDTH];

void start_motor();
void stop_motor();
void configuration_mode();
void write_buffer_to_row(int);
void updateLCD_configurationMode();
void updateLCD_normalMode();

int main(void) {

    DDRD = (1 << PIN_PWM); // Establece OC0B como salida
    DDRB = 0xFF;           // Establece Puerto B como salida
    PORTD = (1 << P1) | (1 << P2) | (1 << P3);

    // Configurar la solicitud de interrupción externa
    EICRA = (1 << ISC01); // Flanco descendente activa la interrupción
    EIMSK = (1 << INT0);  // Habilitar PD2 como interrupción externa
    sei();                // Habilitar interrupciones globales

    TCCR0A = (1 << WGM00) | (1 << WGM01); // Modo Fast PWM
    TCCR0B = (1 << WGM02);                // con tope OCR0A
    OCR0A = PWM_PERIOD - 1;
    OCR0B = dutyCycle1;

    TCCR1B = (1 << WGM12);
    TIMSK1 = (1 << OCIE1A);
    OCR1A = 6249; // 6250 * 256 * (1/16MHz) = 100ms

    liquidCrystal_init();

    updateLCD_normalMode();
    while (1) {
        if (flag) {
            if (motorIsOn)
                stop_motor();
            else
                start_motor();
            flag = 0;
        }

        if (motorIsOn != lastMotorState)
            updateLCD_normalMode();
        lastMotorState = motorIsOn;

        P2State = (PIND & (1 << P2));
        if (!motorIsOn && P2State != lastP2State) {
            if (!P2State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                    configuration_mode();
            }
        }
        lastP2State = P2State;
    }

    return 0;
}

// Rutina de servicio de interrupción para la interrupción externa 0 (PD2)
ISR(INT0_vect) {
    _delay_ms(BOUNCE_DELAY);
    if (!(PIND & (1 << P1))) // Comprobar si P1 sigue en BAJO
        flag = 1; // Establecer la bandera para indicar la interrupción
}

ISR(TIMER1_COMPA_vect) {
    counter++;
    switch (motorIsOn) {
    case 1:
        if (counter >= duration1) {
            OCR0B = dutyCycle2;
            motorIsOn = 2;
            counter = 0;
        }
        break;
    case 2:
        if (counter >= duration2) {
            OCR0B = dutyCycle1;
            motorIsOn = 1;
            counter = 0;
        }
        break;
    default:
        break;
    }
}

void start_motor() {
    TCCR0A |= (1 << COM0B1);             // Modo PWM no invertido
    TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler 64. Inicia conteo
    TCCR1B |= (1 << CS12);               // Prescaler 256. Inicia conteo
    motorIsOn = 1;
}

void stop_motor() {
    TCCR0A &= ~(1 << COM0B1);              // Desconecta OC0B
    TCCR0B &= ~(1 << CS01) & ~(1 << CS00); // Prescaler 0. Detiene conteo
    TCNT0 = 0;                             // Reinicia contador
    TCCR1B &= ~(1 << CS12);                // Prescaler 0. Detiene conteo
    TCNT1 = 0;                             // Reinicia contador
    PORTD &= ~(1 << PIN_PWM);              // Apaga motor
    OCR0B = dutyCycle1;
    counter = 0;
    motorIsOn = 0;
}

void configuration_mode() {
    lastP2State = 0;
    updateLCD_configurationMode();
    while (1) {
        if (flag)
            flag = 0; // do nothing

        P2State = (PIND & (1 << P2));
        if (P2State != lastP2State) {
            if (!P2State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                    break;
            }
        }
        lastP2State = P2State;

        P3State = (PIND & (1 << P3));
        if (P3State != lastP3State) {
            if (!P3State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P3))) {
                    configuringT1 ^= 1;
                    updateLCD_configurationMode();
                }
            }
        }
        lastP3State = P3State;
    }
    updateLCD_normalMode();
}

void write_buffer_to_row(int row) {
    liquidCrystal_setCursor(0, row);
    liquidCrystal_print(buffer);
    liquidCrystal_noCursor();
}

void updateLCD_configurationMode() {
    if (configuringT1) {
        sprintf(&buffer[0], "TIME %d: %-2.1fs", 1, duration1 / 10.0);
        write_buffer_to_row(0);
        sprintf(&buffer[0], "DUTY %d: %2d%%", 1, dutyCycle1);
        write_buffer_to_row(1);
    } else { // configuring T2
        sprintf(&buffer[0], "TIME %d: %-2.1fs", 2, duration2 / 10.0);
        write_buffer_to_row(0);
        sprintf(&buffer[0], "DUTY %d: %2d%%", 2, dutyCycle2);
        write_buffer_to_row(1);
    }
}

void updateLCD_normalMode() {
    if (motorIsOn) {
        sprintf(&buffer[0], "MOTOR: ON ");
        write_buffer_to_row(0);
        sprintf(&buffer[0], "STATUS: STAGE %d", motorIsOn);
        write_buffer_to_row(1);
    } else { // motor is off
        sprintf(&buffer[0], "MOTOR: OFF");
        write_buffer_to_row(0);
        sprintf(&buffer[0], "STATUS: STANDBY");
        write_buffer_to_row(1);
    }
}