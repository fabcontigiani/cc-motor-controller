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
#define PWM_PERIOD 124  // (124 + 1) * 64 * (1/16MHz) = 500us -> f = 2kHz
#define LCD_WIDTH 16

uint8_t P1State;
uint8_t lastP1State = 1;
uint8_t P2State;
uint8_t lastP2State = 1;
uint8_t P3State;
uint8_t lastP3State = 1;
uint8_t motorIsOn = 0;
uint16_t pot1;
uint16_t pot2;
volatile uint8_t flag = 0;
volatile uint8_t dutyCicle = 50;
char buffer[LCD_WIDTH];

void start_motor();
void stop_motor();
int configuration_mode();

int main(void) {

    DDRD = (1 << PIN_PWM); // Establece OC0B como salida
    DDRB = 0xFF;           // Establece Puerto B como salida
    PORTD = (1 << P1) | (1 << P2) | (1 << P3);

    // Configurar la solicitud de interrupción externa
    EICRA = (1 << ISC01); // Flanco descendente activa la interrupción
    EIMSK = (1 << INT0);  // Habilitar PD2 como interrupción externa
    sei();                // Habilitar interrupciones globales

    OCR0A = PWM_PERIOD;
    OCR0B = dutyCicle;

    TCCR0A = (1 << WGM00) | (1 << WGM01); // Modo Fast PWM
    TCCR0B = (1 << WGM02);                // con tope OCR0A

    liquidCrystal_init();
    liquidCrystal_cursor(0, 0);
    liquidCrystal_noCursor();
    // liquidCrystal_print("Hello world!");

    configuration_mode();

    while (1) {
        if (flag) {
            if (motorIsOn)
                stop_motor();
            else
                start_motor();
            flag = 0;
        }

        _delay_ms(10);
        dutyCicle += 1;
        if (dutyCicle >= 118) {
            dutyCicle = 50;
        }
        OCR0B = dutyCicle;

        P2State = (PIND & (1 << P2));
        if (P2State != lastP2State) {
            if (!P2State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2))) {
                    if (configuration_mode())
                        continue;
                }
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

void start_motor() {
    TCCR0A |= (1 << COM0B1);             // Modo PWM no invertido
    TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler 64. Inicia conteo.
    motorIsOn = 1;
}

void stop_motor() {
    TCCR0A &= ~(1 << COM0B1);              // Desconecta OC0B
    TCCR0B &= ~(1 << CS01) & ~(1 << CS00); // Prescaler 0. Detiene conteo.
    PORTD &= ~(1 << PIN_PWM);              // Apaga motor
    motorIsOn = 0;
}

int configuration_mode() {
    sprintf(&buffer[0], "Configuring: T%1d", 1);
    liquidCrystal_print(buffer);
    while (1) {
        if (flag)
            return 1;

        P2State = (PIND & (1 << P2));
        if (P2State != lastP2State) {
            if (!P2State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2))) {
                    break;
                }
            }
        }
        lastP2State = P2State;

        P3State = (PIND & (1 << P3));
        if (P3State != lastP3State) {
            if (!P3State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P3))) {
                    ; // TODO toggle between T1 and T2
                }
            }
        }
        lastP3State = P3State;
    }
    return 0;
}