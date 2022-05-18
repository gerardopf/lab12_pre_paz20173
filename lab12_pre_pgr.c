/*
 * Lab12
 * Gerardo Paz Fuentes
 * Potenciómetro en RA0
 * Botón SLEEP en RB0
 * Botón WAKE UP en RB1
 * Botón SAVE en RB2
 * 
 * Leds ADC en puerto C
 * Leds EEPROM en puerto D
 *
 * Fecha de creación: 03/05/22
 */

#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>         // registros del PIC
#include <stdio.h>
#include <stdlib.h>

/* -------------CONSTANTES ------------ */
#define _XTAL_FREQ 4000000

/* ------------- VARIABLES ------------ */
uint8_t address = 0, cont = 0, mode = 0;

/* ------------- PROTOTIPOS DE FUNCIÓN ------------ */
void setup(void);
uint8_t read(uint8_t address);
void write(uint8_t address, uint8_t data);

/* -------------CÓDIGO PRINCIPAL  ------------ */

void __interrupt() isr (void){
    if(ADIF){
        PIR1bits.ADIF = 0;      // limpiar bandera
        cont = ADRESH;
        PORTC = cont;         // mostrar valor
        mode = 0;
    }
    if(RBIF){
        if(PORTBbits.RB0){ // SLEEP
            SLEEP();
            mode = 1;
        }
        else if(PORTBbits.RB1){    // WAKE UP
            mode = 0;
        }
        else if(PORTBbits.RB2){    // SAVE
            write(address, cont);
            mode = 0;
        }
        INTCONbits.RBIF = 0;
    }
    
    return;
}

void main(void) {
    setup();
    while (1) {
        if(ADCON0bits.GO == 0){
            __delay_us(50);     
            ADCON0bits.GO = 1;  // comenzar otra conversión
        }
        PORTD = read(address);
        PORTE = mode;
    }
}

uint8_t read(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;   // lectura
    EECON1bits.RD;          // obtener dato de lectura
    return EEDAT;           // regresa el dato
}

void write(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;   // escritura
    EECON1bits.WREN = 1;    // habilitar escritura
    
    INTCONbits.GIE = 0;     // deshabilitar interrupciones
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;      // iniciar escritura
    
    EECON1bits.WREN = 0;    // deshabilitar escritura
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;     // volver a habilitar interrupciones
}


void setup(void) {
    ANSEL = 0b00000001; // AN0
    ANSELH = 0;

    TRISA = 0b00000001; // AN0 in
    TRISB = 0b00000111; // RB0, RB1, RB2 in
    TRISC = 0;  // C out
    TRISD = 0;  // D out
    TRISE = 0;  // E out
    
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTE = 0;
    PORTD = 0;  // puertos limpios

    // INTERRUPCIONES
    INTCONbits.GIE = 1; // globales
    INTCONbits.PEIE = 1;    // periféricos
    PIE1bits.ADIE = 1;      // ADC
    PIR1bits.ADIF = 0;      // ADC bandera
    IOCB = 0b00000111;      // on change botones
    INTCONbits.RBIE = 1;    // on change
    INTCONbits.RBIF = 0;    // on change bandera

    // Oscilador 
    OSCCONbits.IRCF = 0b110; // 4MHz
    OSCCONbits.SCS = 1; // Oscilador interno.
    
    // ADC
    ADCON1bits.ADFM = 0;    // justificado izquierda
    ADCON1bits.VCFG0 = 0;   // VDD
    ADCON1bits.VCFG1 = 0;   // VSS
    
    ADCON0bits.ADCS = 0b01; // Fosc/8
    ADCON0bits.CHS = 0;     // AN0
    ADCON0bits.ADON = 1;    // habilitar módulo ADC
    __delay_us(50);         // tiempo para que cargue el capacitor
    

    return;
}