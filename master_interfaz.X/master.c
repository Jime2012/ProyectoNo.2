/*
 * File:   Proyecto2_Maestro.c
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

//#define _XTAL_FREQ 4000000
//#define IN_MIN 0                // Valor minimo de entrada del potenciometro
//#define IN_MAX 127              // Valor máximo de entrada del potenciometro
//#define OUT_MIN 0               // Valor minimo de ancho de pulso de señal PWM
//#define OUT_MAX 804 
#define _XTAL_FREQ 1000000
         // Constante para definir largo de mensaje e iteraciones al enviarlo por el serial
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255            // Valor maximo de entrada del potenciometro
#define OUT_MIN 31              // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 158            // Valor maximo de ancho de pulso de señal PWM
#define OUT_MIN2 0              // Valor minimo de ancho de pulso de señal PWM

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
uint8_t valor = 0;// Contador que envía el maestro al esclavo
unsigned short CCPR = 0;        // Variables para la programacion
uint8_t pot4 = 0;
uint8_t pot3 = 0;
uint8_t pot2 = 0;
uint8_t pot1 = 0;
uint8_t flag = 0;
unsigned short CCPR_2 = 0;
uint8_t modo = 1;
uint8_t interfaz = 1;
uint8_t enviar = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){                      // Interrupción ADC
        if(modo == 1){
            if(ADCON0bits.CHS == 0){            // Verificamos sea AN0 el canal seleccionado
                pot1 = ADRESH;
                CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;

            }
            else if(ADCON0bits.CHS == 1){            // Verificamos sea AN0 el canal seleccionado
                pot2 = ADRESH;
                CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B

            }
            else if(ADCON0bits.CHS == 2){            // Verificamos sea AN0 el canal seleccionado
                pot3 = ADRESH;
            }
            else if(ADCON0bits.CHS == 3){            // Verificamos sea AN0 el canal seleccionado
                pot4 = ADRESH;
            }
        }
        PIR1bits.ADIF = 0;                  // Limpiamos bandera de interrupción
    }
        
    else if(PIR1bits.RCIF){
        interfaz = RCREG;
    }
    else if(INTCONbits.RBIF){            // Fue interrupción del PORTB  
        if(!PORTBbits.RB0){                     // Verificamos si fue RB0 quien generó la interrupción
            modo++;
            if(modo >= 4)
                modo = 1;
            else if(modo == 0)
                modo = 1;
        }
        if(modo == 1){
            if(!PORTBbits.RB1){                     // Verificamos si fue RB0 quien generó la interrupci
                write_EEPROM(0x01, pot1); //Escribir cuando nos dormimos
                write_EEPROM(0x02, pot2); //Escribir cuando nos dormimos
                write_EEPROM(0x03, pot3); //Escribir cuando nos dormimos
                write_EEPROM(0x04, pot4); //Escribir cuando nos dormimos
            }
            else if(!PORTBbits.RB2){                     // Verificamos si fue RB0 quien generó la interrupción
                write_EEPROM(0x05, pot1); //Escribir cuando nos dormimos
                write_EEPROM(0x06, pot2); //Escribir cuando nos dormimos
                write_EEPROM(0x07, pot3); //Escribir cuando nos dormimos
                write_EEPROM(0x08, pot4); //Escribir cuando nos dormimos
            }
        }
        else if(modo == 2){
            if(!PORTBbits.RB1){                     // Verificamos si fue RB0 quien generó la interrupción
                pot1 = read_EEPROM(0x01);
                pot2 = read_EEPROM(0x02); // Mostrar siempre la lectura del eeprom en el puerto C
                pot3 = read_EEPROM(0x03); // Mostrar siempre la lectura del eeprom en el puerto C
                pot4 = read_EEPROM(0x04); // Mostrar siempre la lectura del eeprom en el puerto C

                
                
            }
            else if(!PORTBbits.RB2){                     // Verificamos si fue RB0 quien generó la interrupción
                pot1 = read_EEPROM(0x05);
                pot2 = read_EEPROM(0x06); // Mostrar siempre la lectura del eeprom en el puerto C
                pot3 = read_EEPROM(0x07); // Mostrar siempre la lectura del eeprom en el puerto C
                pot4 = read_EEPROM(0x08); // Mostrar siempre la lectura del eeprom en el puerto C
                
            }
  
        }
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción del puerto B
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
       
        __delay_ms(10);
        
        PORTAbits.RA7 = 1;      // Deshabilitamos el ss del esclavo
        __delay_ms(10);         // Delay para que el PIC pueda detectar el cambio en el pin
        PORTAbits.RA7 = 0;      // habilitamos nuevamente el escalvo

        if(flag == 1){
            SSPBUF = 0b00000001 | pot3;// se deja el primer bit en 1
            while(!SSPSTATbits.BF){}// Esperamos a que termine el envio
            flag = 0;
        }
        else if(flag == 0){
            SSPBUF = 0b11111110 & pot4; //Se deja el primer  bit en cero
            while(!SSPSTATbits.BF){}// Esperamos a que termine el envio
            flag = 1;
        }

        PORTAbits.RA7 = 1;      // Deshabilitamos el ss del esclavo
        __delay_ms(10);         // Esperamos un tiempo para que el PIC pueda detectar el cambio en el pin
        PORTAbits.RA7 = 0;      // habilitamos nuevamente el escalvo
        
        if(modo == 1){
                CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;
                
                CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                
            if(ADCON0bits.GO == 0){             // No hay proceso de conversion
                // *usando mas de un canal analógico
                if(ADCON0bits.CHS == 0b0000)
                    ADCON0bits.CHS = 0b0001;    // Cambio de canal 1
                else if(ADCON0bits.CHS == 0b0001)
                    ADCON0bits.CHS = 0b0010;    // Cambio de canal 2
                else if(ADCON0bits.CHS == 0b0010)
                    ADCON0bits.CHS = 0b0011;    // Cambio de canal 2
                else if(ADCON0bits.CHS == 0b0011)
                    ADCON0bits.CHS = 0b0000;    // Cambio de canal 2
                __delay_us(40);                 // Tiempo adquisición
                ADCON0bits.GO = 1;              // proceso de conversión
            }
            PORTDbits.RD7 = 1;//se enciende led de modo
            PORTDbits.RD6 = 0;
            PORTDbits.RD5 = 0;
        }
        else if(modo == 2){
                CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;
                
                CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                
            PORTDbits.RD6 = 1;//se enciende led de modo
            PORTDbits.RD7 = 0;
            PORTDbits.RD5 = 0;
        }
        else if(modo == 3){
            PORTDbits.RD5 = 1;//se enciende led de modo
            PORTDbits.RD6 = 0;
            PORTDbits.RD7 = 0;
            
           enviar = 0b00000011 & interfaz;
            if (enviar == 0){
                pot1 = interfaz;
                CCPR_2 = map(pot1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b1; // Guardamos los 2 bits menos significativos en DC1B
                CCP2CONbits.DC2B1 = CCPR_2>>1 & 0b1;
            }
            else if(enviar == 1){
                pot2 = interfaz; // Mostrar siempre la lectura del eeprom en el puerto C
                CCPR = map(pot2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
            }
            else if(enviar == 2){
                pot3 = interfaz;
            }
            else if(enviar == 0b00000011){
                pot4 = interfaz;
            }
        }
    }
    return;
  
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0xF;                // AN0, AN1 y AN2 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    TRISA = 0xF;                // AN0, AN1, AN2 y AN3 como entrada
    PORTA = 0;
    TRISB = 0b00011111;
    TRISD = 0;
    PORTD = 0;


    OSCCONbits.IRCF = 0b0100;   //1MHz
    OSCCONbits.SCS = 1;
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b00;     // Fosc/2
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time

    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    PR2 = 39;                  // periodo de 2ms

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
    TRISCbits.TRISC3 = 0;       // Habilitamos salida de PWM
    PORTC = 0;
    
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0b00011111; 
    INTCONbits.RBIE = 1;   
    IOCB = 0b00011111;         
    INTCONbits.RBIF = 0;
    
     // Configs de Maestro
    TRISCbits.TRISC4 = 1;         // -> SDI entrada, SCK y SD0 como salida
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC5 = 0;
    PORTC = 0;

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = valor;              // Enviamos un dato inicial            // Enviamos un dato inicial

    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIE1bits.RCIE = 1;
}
// función para el mapeo
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;  // Lectura en la EEPROM
    EECON1bits.RD = 1;       // Conseguimos dato de la EEPROM
    return EEDAT;              // Regresamos ese dato leido 
}

//Función para escribir
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0; // Escritura en la EEPROM
    EECON1bits.WREN = 1;  // Habilitamos la escritura a la EEPROM
    
    INTCONbits.GIE = 0;    // Deshabilitamos las interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;    // Se inicia la escritura
    __delay_ms(100);
    EECON1bits.WREN = 0;     // se deshabilita escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;   // Habilitamos las interrupciones
}