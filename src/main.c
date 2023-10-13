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
uint8_t motorIsOn = 0;
uint8_t configuringT1 = 1;
uint8_t duration1 = 10;
uint8_t duration2 = 20;
uint8_t dutyCycle1 = 40;
uint8_t dutyCycle2 = 95;
uint16_t pot1;
uint16_t pot2;
volatile uint8_t flag = 0;
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

    OCR0A = PWM_PERIOD - 1;
    OCR0B = dutyCycle1;

    TCCR0A = (1 << WGM00) | (1 << WGM01); // Modo Fast PWM
    TCCR0B = (1 << WGM02);                // con tope OCR0A

    liquidCrystal_init();

    updateLCD_normalMode();
    while (1) {
        if (flag) {
            if (motorIsOn)
                stop_motor();
            else
                start_motor();
            updateLCD_normalMode();
            flag = 0;
        }

        P2State = (PIND & (1 << P2));
        if (P2State != lastP2State) {
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
    sprintf(&buffer[0], "CONFIGURING: T%-2d", (configuringT1) ? 1 : 2);
    write_buffer_to_row(0);
    if (configuringT1)
        sprintf(&buffer[0], "TIME:%2d DUTY:%%%2d", duration1, dutyCycle1);
    else
        sprintf(&buffer[0], "TIME:%2d DUTY:%%%2d", duration2, dutyCycle2);
    write_buffer_to_row(1);
}

void updateLCD_normalMode() {
    sprintf(&buffer[0], "%-16s", "NORMAL MODE");
    write_buffer_to_row(0);
    sprintf(&buffer[0], "MOTOR: %-9s", (motorIsOn) ? "ON" : "OFF");
    write_buffer_to_row(1);
}