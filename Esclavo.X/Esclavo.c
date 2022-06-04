/*
 * File:   Proyecto2_Esclavo.c
 * Author: Danika Andrino y Lucia de la Rosa
 * 
 *
 */

// CONFIG1
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

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000
         // Constante para definir largo de mensaje e iteraciones al enviarlo por el serial
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor maximo de entrada del potenciometro
#define IN_MAX2 127              // Valor maximo de entrada del potenciometro
#define OUT_MIN 31              // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 158            // Valor maximo de ancho de pulso de señal PWM
#define OUT_MIN2 0              // Valor minimo de ancho de pulso de señal PWM
#define FLAG_SPI 0xFFF


/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0;        // Variables para programar
uint8_t enviar = 0;
uint8_t valor = 0;
uint8_t CCPR_3 = 0;
uint8_t CCPR_4 = 0;
char val_temporal = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){
        PORTAbits.RA7 = ~PORTAbits.RA7;
        
        val_temporal = SSPBUF;
        if (val_temporal != FLAG_SPI){  // revisa si es un dato
            valor = val_temporal;       //
            enviar = 0b00000001 & valor; //Se utiliza unicamente el primer bit
            if (enviar == 1){
                CCPR_3 = map(valor, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_3>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_3 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_3>>1 & 0b1;

            }
            else if(enviar == 0){
                CCPR_4 = map(valor, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR_4>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR_4 & 0b11; // Guardamos los 2 bits menos significativos en DC1B
            }
        __delay_ms(10);
        
        }
        PIR1bits.SSPIF = 0;             // Limpiamos bandera de interrupci n?
    }
    
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        
        
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0xF;                // AN0, AN1 y AN2 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    TRISA = 0xF;                // AN0, AN1 y AN2 como entrada
    TRISAbits.TRISA5 = 1;
    PORTA = 0;
    TRISB = 0;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0;
    TRISAbits.TRISA7 = 0;

    OSCCONbits.IRCF = 0b0100;   //1MHz
    OSCCONbits.SCS = 1;
    
   // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 39;                  // periodo de 2ms

    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM

    CCPR1L = 31>>2;
    CCP1CONbits.DC1B = 31 & 0b11; //0.5ms ancho de pulso, 25% duty cycle

    TRISCbits.TRISC1 = 1;       // Deshabilitamos salida de CCP1
    CCP2CON = 0;                // Apagamos CCP1
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 31>>2;
    CCP2CONbits.DC2B0 = 31 & 0b01; //0.5ms ancho de pulso, 25% duty cycle
    CCP2CONbits.DC2B1 = (31 & 0b10)>>1;
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;       // Habilitamos salida de PWM
    PORTC = 0;

    TRISCbits.TRISC4 = 1;         // -> SDI y SCK entradas, SD0 como salida
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC5 = 0;
    PORTC = 0;
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS entrada o salida
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj

    PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
    INTCONbits.RBIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    

}
// función para el mapeo
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
