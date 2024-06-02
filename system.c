
/*
 * File:   system.c
 * Author: james
 *
 * Created on March 25, 2024, 10:44 AM
 */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "serialusM2M.h"
#include "system.h"          /* variables/params used by system.c             */


void init_compteur_temps_match()
{
    CPT_TEMPS_MATCH.actif = false;
    CPT_TEMPS_MATCH.t_ms = 0;
}

void init_system (void)
{
    
    init_clock();
  
    // Config des Timers
    config_timer_5ms();
    config_timer_10ms();
    config_timer_20ms();
    config_timer_100ms();
    config_timer_200ms();
   
    // Config des modules
    ConfigMapping ();
    ConfigPorts ();
    ConfigInterrupt ();
    ConfigPWM();
    //ConfigADC();

    
  //  InitUART(UART_XBEE, 115200);
    InitUART(UART_AX12, 1000000);
    serialusM2M_init();
    
    delay_ms(1000);
    

    
    TIMER_5ms   = ACTIVE;
    TIMER_10ms  = ACTIVE;
    TIMER_20ms  = ACTIVE;
    TIMER_100ms = ACTIVE;
    init_compteur_temps_match();

}





/******************************************************************************/
/***************************** CONFIGURATIONS TIMERS **************************/
/******************************************************************************/


/**
 * Configuration du Timer de 5 ms pour l'asservissement
 *        /!\  ATTENTION : TIMER SAFE   /!\                 //
 * /!\ INTERDICTION D'UTILISER DES FONCTIONS BLOQUANTES /!\  //
 * /!\ OU DE DESACTIVER LE DECOMPTE DU TIMER PENDANT   /!\  //
 * /!\ L'INTERRUPTION, MERCI !                         /!\  //     
 */
void config_timer_5ms()
{
    //Timer de 5,00052846 ms

    TIMER_5ms = DESACTIVE;
    T1CONbits.TCS = 0;          //clock sur FCY
    T1CONbits.TGATE = 0;        //Mode Gate désactivé
    T1CONbits.TCKPS = 0b10;     //Divise la fréquence par 64 (prescaler 1:64)
    TMR1 = 0x00;                //Reset du timer

    // FOSC = 80 MhZ, FCY = FOSC/2 = 40 Mhz 
    // T = 0, 250 ns
    // Presacler 1 : 64 => T = 3,4 µs
    // 3127 * 3,4µs = 5 ms
    PR1 = 3126;                  //décompte de 5ms

    FLAG_TIMER_5ms = 0;         //Clear flag interrupt timer
    IEC0bits.T1IE = 1;          //Enable Timer1 interrupt
}


/**
 * Configuration du Timer de 10 ms pour l'évitement
 *        /!\  ATTENTION : TIMER SAFE   /!\                 //
 * /!\ INTERDICTION D'UTILISER DES FONCTIONS BLOQUANTES /!\  //
 * /!\ OU DE DESACTIVER LE DECOMPTE DU TIMER PENDANT   /!\  //
 * /!\ L'INTERRUPTION, MERCI (PAS D'AX12) !            /!\  //       
 */
void config_timer_10ms() //Timer autom
{
    //Timer de 10,00106 ms

    TIMER_10ms = DESACTIVE;
    T2CONbits.TCS = 0;          //clock sur FCY
    T2CONbits.T32 = 0;          //Timer 16 bits
    T2CONbits.TGATE = 0;        //Mode Gate désactivé
    T2CONbits.TCKPS = 0b11;     //Divise la fréquence par 256 (prescaler 1:256)
    TMR2 = 0x00;                //Reset du timer

    // FOSC = 80 MhZ, FCY = FOSC/2 = 40 MHz
    // T = 0, 250 ns
    // Presacler 1 : 256 => T = 6,4 µs
    // 1563 * 6,4 µs = 10 ms
    PR2 = 1563;                 //décompte de 10ms

    FLAG_TIMER_10ms = 0;        //Clear flag interrupt timer
    IEC0bits.T2IE = 1;          //Enable Timer2 interrupt
}

/**
 * Configuration du Timer de 20 ms pour l'autom
 *      NB : TIMER NON SAFE 
 *  POSSIBILITE D'UTILISER DES FONCTIONS BLOQUANTES
 *  UTILISE POUR DES TACHES NON CRITIQUES     
 */
void config_timer_20ms()
{
    //Timer de 20,00211 ms

    TIMER_10ms = DESACTIVE;
    T3CONbits.TCS = 0;          //clock sur FCY
    T3CONbits.TGATE = 0;        //Mode Gate désactivé
    T3CONbits.TCKPS = 0b11;     //Divise la fréquence par 256 (prescaler 1:256)
    TMR3 = 0x00;                //Reset du timer

    // FOSC = 80 MhZ, FCY = FOSC/2 = 40 MHz
    // T = 0, 250 ns
    // Presacler 1 : 256 => T = 6,4 µs
    // 1563 * 6,4 µs = 10 ms
    PR3 = 3126;                 //décompte de 20ms

    FLAG_TIMER_20ms = 0;        //Clear flag interrupt timer
    IEC0bits.T3IE = 1;          //Enable Timer3 interrupt
}

/**
 * Configuration du Timer de 100 ms pour le scheduler temps de match
 *      NB : TIMER NON SAFE 
 *  POSSIBILITE D'UTILISER DES FONCTIONS BLOQUANTES
 *  UTILISE POUR DES TACHES NON CRITIQUES     
 */
void config_timer_100ms()
{
    //Timer de 99,99777 ms

    TIMER_10ms = DESACTIVE;
    T4CONbits.TCS = 0;          //clock sur FCY
    T4CONbits.T32 = 0;          //Timer 16 bits
    T4CONbits.TGATE = 0;        //Mode Gate désactivé
    T4CONbits.TCKPS = 0b11;     //Divise la fréquence par 256 (prescaler 1:256)
    TMR4 = 0x00;                //Reset du timer

    // FOSC = 80 MhZ, FCY = FOSC/2 = 40 MHz
    // T = 0, 250 ns
    // Presacler 1 : 256 => T = 6,4 µs
    // 1563 * 6,4 µs = 10 ms
    PR4 = 15628;                //décompte de 100ms

    FLAG_TIMER_100ms = 0;       //Clear flag interrupt timer
    IEC1bits.T4IE = 1;          //Enable Timer4 interrupt
}


/**
 * Configuration du Timer de 200 ms pour la liaison serie (debug + serialus)
 *      NB : TIMER NON SAFE 
 *  POSSIBILITE D'UTILISER DES FONCTIONS BLOQUANTES
 *  UTILISE POUR DES TACHES NON CRITIQUES     
 */
void config_timer_200ms() 
{
    //Timer de 200,00194 ms
    TIMER_200ms = DESACTIVE;
    T5CONbits.TCS = 0;          //clock sur FCY
    T5CONbits.TGATE = 0;        //Mode Gate désactivé
    T5CONbits.TCKPS = 0b11;     //Divise la fréquence par 256 (prescaler 1:256)
    TMR4 = 0x00;                //Reset du timer

    PR4 = 31257;                //décompte de 200ms

    //IPC6bits.T4IP = 0x03;     //priorité à 3 --> définit dans la table des interrupt
    FLAG_TIMER_200ms = 0;       //Clear flag interrupt timer
    IEC1bits.T5IE = 1;          //Enable Timer4 interrupt
}


void init_clock(void)
{
    //Tunage de la fréquence : Ftune = 8,0056625
    OSCTUNbits.TUN = 0;        //SEMBLE NE PAS FONCTIONNER ....

    //New Osc = FRC sans PLL
    OSCCONbits.NOSC = 0b000;
    OSCCONbits.OSWEN = 1;
    while(OSCCONbits.OSWEN != 0);


    // Configure PLL prescaler, PLL postscaler, PLL divisor
    // Fext = Fin * M / (N1 * N2)
    // F =  80,017 142 MHz

    PLLFBD = 150; // M = 152
    CLKDIVbits.PLLPOST= 0b00; // N2 = 2 
    CLKDIVbits.PLLPRE= 5; // N1 = 7

    CLKDIVbits.DOZE = 0b000; //FRC = 1:1 FRC

    //On switch sur la nouvelle clock avec PLL
    OSCCONbits.NOSC = 0b001;
    OSCCONbits.OSWEN = 1;
    while(OSCCONbits.OSWEN != 0);

    // Wait for PLL to lock
    while(OSCCONbits.LOCK!=1);
}




/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration funtions, reset source evaluation          */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c.                                          */
/*                                                                            */
/******************************************************************************/

/* Refer to the device Family Reference Manual Oscillator section for
information about available oscillator configurations.  Typically
this would involve configuring the oscillator tuning register or clock
switching useing the compiler's __builtin_write_OSCCON functions.
Refer to the C Compiler for PIC24 MCUs and dsPIC DSCs User Guide in the
compiler installation directory /doc folder for documentation on the
__builtin functions.*/

void ConfigMapping(void){
     // using RP pins
     // Mapping UART1 : RaspberryPi (RPI)
	_U1RXR	= 0x05;		// IN	: UART1 RX on RP5
	_RP6R	= 0x03;		// OUT	: UART1 TX on RP6

	// Mapping UART2 : AX18
	_U2RXR	= 0x03;         // IN	: UART2 RX on RP3
	_RP2R	= 0x05;         // OUT	: UART2 TX on RP2

        // Mapping QEI1 : ENCODER
	_QEB1R	= 0x0C;         // IN	: QEB1 on RP12 
	_QEA1R	= 0x0E;         // IN	: QEA1 on RP14

	
}

void ConfigPorts (void)
{
	// Desactivation  CAN
	_ADON		= 0;

	// Desactivation of analogs inputs
	_PCFG0		= 1;
	_PCFG1		= 1;
	_PCFG2		= 1;
	_PCFG3		= 1;
	_PCFG4		= 1;
	_PCFG5		= 1;
	_PCFG6		= 1;
	_PCFG7		= 1;
	_PCFG8		= 1; 
    
    
   

    // Set all AN pins to digital
    AD1PCFGL = 0xFFFF;
    // Disable ADC module to configure its pins as digital I/O
    AD1CON1bits.ADON = 0;

	// Desactivation PWM
	_PEN1L		= 0;
	_PEN1H		= 0;
	_PEN2L		= 0;
	_PEN2H		= 0;
	_PEN3L		= 0;
	_PEN3H		= 0;


    
    // _TRIS** = 1; ==> 	INPUT  
    // _TRIS** = 0; ==> OUTPUT

	//_CNxIE  : interrupt on pin 	| _CN6PUE : pull-up on pin
    // FROM THE TABLE 4-3: CHANGE NOTIFICATION REGISTER MAP FOR dsPIC33FJ128MC204/804
    
	// Config PORTA
	_TRISA0		= 0;	_CN2IE	= 0;	_CN2PUE		= 0;	// OUT : (DIGIT) : INHIBIT_AX18
	_TRISA1		= 0;	_CN3IE	= 0;	_CN3PUE		= 0;	// OUT : (DIGIT) : DIR_UART_AX18
    _TRISA2     = 1;    _CN30IE	= 0;	_CN30PUE	= 0;    // IN  : (DIGIT) : SCK2
	_TRISA3		= 1;	_CN29IE	= 0;	_CN29PUE	= 0;	// IN  : (DIGIT) : OUT1
	_TRISA4		= 1;	_CN0IE	= 0;	_CN0PUE		= 0;	// IN  : (DIGIT) : CAPT5
    _TRISA7		= 0;                                        // OUT : (DIGIT) : SENS MOTOR
	_TRISA8		= 1;                                        // IN  : (DIGIT) : SCK1
	_TRISA9		= 1;                                        // IN  : (DIGIT) : CAPT6
	_TRISA10	= 0;                                        // OUT : (DIGIT) : EXTRA1

	// Config PORTB
	_TRISB0		= 0;	_CN4IE	= 0;	_CN4PUE		= 0;	// RES : PGED1 PICKIT PROG
	_TRISB1		= 0;	_CN5IE	= 0;	_CN5PUE		= 0;	// RES : PGEC1 PICKIT PROG
	_TRISB2		= 0;	_CN6IE	= 0;	_CN6PUE		= 0;	// OUT : (UART)  : TX AX18          
	_TRISB3		= 1;	_CN7IE	= 0;	_CN7PUE		= 0;	// IN  : (UART)  : RX AX18          
	_TRISB4		= 1;	_CN1IE	= 0;	_CN1PUE		= 0;	// IN  : (DIGIT) : CAPT4
	_TRISB5		= 1;	_CN27IE	= 0;	_CN27PUE	= 0;	// IN  : (UART)  : RX RPI
	_TRISB6		= 0;	_CN24IE	= 0;	_CN24PUE	= 0;	// OUT : (UART)  : TX RP1
	_TRISB7		= 0;	_CN23IE	= 0;	_CN23PUE	= 0;	// OUT : (DIGIT) : PUMPB1
	_TRISB8		= 0;	_CN22IE	= 0;	_CN22PUE	= 0;	// OUT : (DIGIT) : PUMPB2
	_TRISB9		= 0;	_CN21IE	= 0;	_CN21PUE	= 0;	// OUT : (DIGIT) : PUMPB3
	_TRISB10	= 0;	_CN16IE	= 0;	_CN16PUE	= 0;	// OUT : (DIGIT) : EXTRA2
	_TRISB11	= 0;	_CN15IE	= 0;	_CN15PUE	= 0;	// OUT : (PWM)   : PWM1L3 (PWM_Turb2_L)
	_TRISB12	= 1;	_CN14IE	= 0;	_CN14PUE	= 0;	// IN  : (QEB1)  : ENCODER B
	_TRISB13	= 0;	_CN13IE	= 0;	_CN13PUE	= 0;	// OUT : (PWM)   : PWM1L2 (PWM_Turb1_L)
	_TRISB14	= 1;	_CN12IE	= 0;	_CN12PUE	= 0;	// IN  : (QEA1)  : ENCODER A
	_TRISB15	= 0;	_CN11IE	= 0;	_CN11PUE	= 0;	// OUT : (PWM)   : PWM1L1 (PWM_Turb3_L)

	// Config PORTC
	_TRISC0		= 1;	_CN8IE	= 0;	_CN8PUE		= 0;	// IN  : (DIGIT) : OUT3 
	_TRISC1		= 1;	_CN9IE	= 0;	_CN9PUE		= 0;	// IN  : (DIGIT) : SCK3
	_TRISC2		= 1;	_CN10IE	= 0;	_CN10PUE	= 0;	// IN  : (DIGIT) : OUT2
	_TRISC3		= 1;	_CN28IE	= 0;	_CN28PUE	= 0;	// IN  : (DIGIT) : CAPT1 // change to out for test on led
	_TRISC4		= 1;	_CN25IE	= 0;	_CN25PUE	= 0;	// IN  : (DIGIT) : CAPT2
	_TRISC5		= 1;	_CN26IE	= 0;	_CN26PUE	= 0;	// IN  : (DIGIT) : CAPT3
    
	_TRISC6		= 0;	_CN18IE	= 0;	_CN18PUE	= 0;	// OUT : (DIGIT) : PUMPS3
	_TRISC7		= 0;	_CN17IE	= 0;	_CN17PUE	= 0;	// OUT : (PWM)   : PWM2L1  (PWM_Servo_L)
	_TRISC8		= 0;	_CN20IE	= 0;	_CN20PUE	= 0;	// OUT : (DIGIT) : PUMPS2
	_TRISC9		= 0;	_CN19IE	= 0;	_CN19PUE	= 0;	// OUT : (DIGIT) : PUMPS1

	
	// INITIALISATION AT STATE LOW	0

	// Init PORTA
	_LATA0		= 0;	
	_LATA1		= 0;	
	_LATA3		= 0;	
	_LATA4		= 0;	
	_LATA7		= 0;	
	_LATA8		= 0;	
	_LATA9		= 0;	
	_LATA10		= 0;	

	// Init PORTB
	_LATB0		= 0;	
	_LATB1		= 0;	
	_LATB2		= 0;	
	_LATB3		= 0;	
	_LATB4		= 0;	
	_LATB5		= 0;	
	_LATB6		= 0;	
	_LATB7		= 0;	
	_LATB8		= 0;	
	_LATB9		= 0;	
	_LATB10		= 0;	
	_LATB11		= 0;	
	_LATB12		= 0;	
	_LATB13		= 0;	
	_LATB14		= 0;	
	_LATB15		= 0;	

	// Init PORTC
	_LATC0		= 0;	
	_LATC1		= 0;	
	_LATC2		= 0;	
	_LATC3		= 0;	
	_LATC4		= 0;	
	_LATC5		= 0;	
	_LATC6		= 0;	
	_LATC7		= 0;	
	_LATC8		= 0;	
	_LATC9		= 0;	
}


// FROM THE TABLE 4-4: INTERRUPT CONTROLLER REGISTER MAP

/*
Interrupt Occurs: 
 * When an event triggers an interrupt, the microcontroller hardware 
    sets the corresponding flag in the IFS register.
 
Interrupt Check:
 * The microcontroller checks if the interrupt is enabled by looking at 
 * the corresponding bit in the IEC register. If the interrupt is disabled,
 * the event is ignored; otherwise, the microcontroller proceeds.
 * 
Interrupt Service Routine (ISR): 
  If the interrupt is enabled and its priority level allows it, 
  the microcontroller pauses its current operations to execute the ISR associated with the interrupt.
 
Flag Clearing: Within the ISR, software typically clears the interrupt flag 
 * in the IFS register to indicate the interrupt has been handled. 
 * This clearing prevents the ISR from being called again for the same event.
 * 
 */

void ConfigInterrupt (void)
{
    // INTCON2: Interrupt Control Register 2 Configuration
    INTCON2bits.ALTIVT = 0; // Use primary interrupt vector table.
    INTCON2bits.INT0EP = 0; // Configure INT0 to trigger on rising edge.

    // Clear all interrupt flags to ensure no interrupts are pending before configuration.
    // IFS0: Interrupt Flag Status Register 
    
    IFS0bits.DMA1IF = 0;    // Clear DMA channel 1 interrupt flag.
    IFS0bits.AD1IF = 0;     // Clear ADC1 interrupt flag.
    IFS0bits.U1TXIF = 0;    // Clear UART1 Transmit interrupt flag.
  
	IFS0bits.SPI1IF		= 0;			//
	IFS0bits.SPI1EIF	= 0;			//
	IFS0bits.T3IF		= 0;			//
	IFS0bits.T2IF		= 0;			// ... (include all other flag clearings here)
	IFS0bits.OC2IF		= 0;			//
	IFS0bits.IC2IF		= 0;			//
	IFS0bits.DMA0IF		= 0;			//
	IFS0bits.T1IF		= 0;			//
	IFS0bits.OC1IF		= 0;			//
	IFS0bits.IC1IF		= 0;			//
	IFS0bits.INT0IF		= 0;			//

	// IFS1: Interrupt Flag Status Register 1
    
    IFS1bits.U2TXIF     = 0;            // Clear UART2 Transmit interrupt flag
	IFS1bits.U2RXIF		= 0;			//
	IFS1bits.INT2IF		= 0;			//
	IFS1bits.T5IF		= 0;			//
	IFS1bits.T4IF		= 0;			//
	IFS1bits.OC4IF		= 0;			//
	IFS1bits.OC3IF		= 0;			//
	IFS1bits.DMA2IF		= 0;			//
	IFS1bits.IC8IF		= 0;			//
	IFS1bits.IC7IF		= 0;			//
	IFS1bits.INT1IF		= 0;			//
	IFS1bits.CNIF		= 0;			//
	IFS1bits.CMIF		= 0;			//
	IFS1bits.MI2C1IF	= 0;			//
	IFS1bits.SI2C1IF	= 0;			//

	// IFS2: Interrupt Flag Status Register 2
    
	IFS2bits.DMA4IF		= 0;			//
	IFS2bits.PMPIF		= 0;			//
	IFS2bits.DMA3IF		= 0;			//
	IFS2bits.C1IF		= 0;			//
	IFS2bits.C1RXIF		= 0;			//
	IFS2bits.SPI2IF		= 0;			//
	IFS2bits.SPI2EIF	= 0;			//

	// IFS3: Interrupt Flag Status Register 3
    
	IFS3bits.FLTA1IF	= 0;			//
	IFS3bits.RTCIF		= 0;			//
	IFS3bits.DMA5IF		= 0;			//
	IFS3bits.QEI1IF		= 0;			//
	IFS3bits.PWM1IF		= 0;			//

	// IFS4: Interrupt Flag Status Register 4
	IFS4bits.DAC1LIF	= 0;			//
	IFS4bits.DAC1RIF	= 0;			//
	IFS4bits.QEI2IF		= 0;			//
	IFS4bits.FLTA2IF	= 0;			//
	IFS4bits.PWM2IF		= 0;			//
	IFS4bits.C1TXIF		= 0;			//
	IFS4bits.DMA7IF		= 0;			//
	IFS4bits.DMA6IF		= 0;			//
	IFS4bits.CRCIF		= 0;			//
	IFS4bits.U2EIF		= 0;			//
	IFS4bits.U1EIF		= 0;			//

    // Configure interrupt priorities. Lower numbers are higher priority.
    // IPC0: Interrupt Priority Control Register 0
    
    IPC0bits.T1IP       = PRIO_INTER_TIMER1; // Set Timer1 interrupt priority.
	IPC0bits.OC1IP		= 0x00;                     //
	IPC0bits.IC1IP		= 0x00;                     //
	IPC0bits.INT0IP		= 0x00;                     //Pas de INT0

	// IPC1
	IPC1bits.T2IP		= PRIO_INTER_TIMER2;		// Timer2
	IPC1bits.OC2IP		= 0x00;                     //
	IPC1bits.IC2IP		= 0x00;                     //
	IPC1bits.DMA0IP		= 0x00;                     //

	// IPC2
	IPC2bits.U1RXIP		= PRIO_INTER_UART1_RX;		// UART1 RX
	IPC2bits.SPI1IP		= 0x00;                     //
	IPC2bits.SPI1EIP	= 0x00;                     //
	IPC2bits.T3IP		= PRIO_INTER_TIMER3;		// Timer 3

	// IPC3
	IPC3bits.DMA1IP		= 0x00;                     //
	IPC3bits.AD1IP		= 0x00;                     //
	IPC3bits.U1TXIP		= 0x00;                     //

	// IPC4
	IPC4bits.CNIP		= 0x00;                     //
	IPC4bits.MI2C1IP	= 0x00;                       //
	IPC4bits.SI2C1IP	= 0x00;                 // 

	// IPC5
	IPC5bits.IC8IP		= 0x00;                     //
	IPC5bits.IC7IP		= 0x00;                     //
	IPC5bits.INT1IP		= 0x00;                     //

	// IPC6
	IPC6bits.T4IP		= PRIO_INTER_TIMER4;		// Timer 4
	IPC6bits.OC4IP		= 0x00;                     //
	IPC6bits.OC3IP		= 0x00;                     //
	IPC6bits.DMA2IP		= 0x00;                     //

	// IPC7
	IPC7bits.U2TXIP		= PRIO_INTER_UART2_TX;
	IPC7bits.U2RXIP		= PRIO_INTER_UART2_RX;		// UART2 RX
	IPC7bits.INT2IP		= 0x00;                     //
	IPC7bits.T5IP		= PRIO_INTER_TIMER5;		//

	// IPC8
	IPC8bits.C1IP		= 0x00;                     //
	IPC8bits.C1RXIP		= 0x00;                     //
	IPC8bits.SPI2IP		= 0x00;                     //
	IPC8bits.SPI2EIP	= 0x00;                     //

	// IPC9
	IPC9bits.DMA3IP		= 0x00;                     //

	// IPC10
	// PAS de IPC10

	// IPC11
	IPC11bits.DMA4IP	= 0x00;                     //
	IPC11bits.PMPIP		= 0x00;                     //

	// IPC12
	// PAS de IPC12

	// IPC13
	// PAS de IPC13

	// IPC14
	IPC14bits.QEI1IP	= PRIO_INTER_QEI1;          // Overflow sur QEI1 -> Codeur Droit
	IPC14bits.PWM1IP	= 0x00;                     //

	// IPC15
	IPC15bits.FLTA1IP	= 0x00;                     //
	IPC15bits.RTCIP		= 0x00;                     //
	IPC15bits.DMA5IP	= 0x00;                     //

	// IPC16
	IPC16bits.U2EIP		= 0x00;                     //
	IPC16bits.U1EIP		= 0x00;                     //
	IPC16bits.CRCIP		= 0x00;                     //

	// IPC17
	IPC17bits.C1TXIP	= 0x00;                     //
	IPC17bits.DMA7IP	= 0x00;                     //
	IPC17bits.DMA6IP	= 0x00;                     //

	// IPC18
	IPC18bits.QEI2IP	= 0x00;                      // 
	IPC18bits.FLTA2IP	= 0x00;                     //
	IPC18bits.PWM2IP	= 0x00;                     //

	// IPC19
	IPC19bits.DAC1LIP	= 0x00;                     //
	IPC19bits.DAC1RIP	= 0x00;                     //


	// Enable interrupts as needed for the peripherals being used.
    // IEC0: Interrupt Enable Control Register 0

	IEC0bits.DMA1IE		= 0;                        // NC
	IEC0bits.AD1IE		= 0;                    	// NC
	IEC0bits.U1TXIE		= 0;                        // NC
	IEC0bits.U1RXIE		= ACTIV_INTER_UART1_RX;     // Interruption RX sur UART1
	IEC0bits.SPI1IE		= 0;                        // NC
	IEC0bits.SPI1EIE	= 0;                        // NC
	IEC0bits.T3IE		= ACTIV_INTER_TIMER3;		// Interruption sur Timer3
	IEC0bits.T2IE		= ACTIV_INTER_TIMER2;		// Interruption sur Timer2
	IEC0bits.OC2IE		= 0;                        // NC
	IEC0bits.IC2IE		= 0;                        // NC
	IEC0bits.DMA0IE		= 0;                        // NC
	IEC0bits.T1IE		= ACTIV_INTER_TIMER1;		// Interruption Timer1
	IEC0bits.OC1IE		= 0;                        // NC
	IEC0bits.IC1IE		= 0;                        // NC
	IEC0bits.INT0IE		= 0;                        // NC

	// IEC1
	IEC1bits.U2TXIE		= ACTIV_INTER_UART2_TX;     // Interruption TX sur UART2
	IEC1bits.U2RXIE		= ACTIV_INTER_UART2_RX;		// Interruption RX sur UART2
	IEC1bits.INT2IE		= 0;                    	// NC
	IEC1bits.T5IE		= ACTIV_INTER_TIMER5;    	// Interruption sur Timer5
	IEC1bits.T4IE		= ACTIV_INTER_TIMER4;		// Interruption sur Timer4
	IEC1bits.OC4IE		= 0;                        // NC
	IEC1bits.OC3IE		= 0;                        // NC
	IEC1bits.DMA2IE		= 0;                        // NC
	IEC1bits.IC8IE		= 0;                        // NC
	IEC1bits.IC7IE		= 0;                        // NC
	IEC1bits.INT1IE		= 0;                        // NC
	IEC1bits.CNIE		= 0;                        // NC
	IEC1bits.MI2C1IE	= 0;                        // Interruption I2C MAITRE
	IEC1bits.SI2C1IE	= 0;                        // Interruption I2C ESCLAVE

	// IEC2
	IEC2bits.DMA4IE		= 0;                        // NC
	IEC2bits.PMPIE		= 0;                        // NC
	IEC2bits.DMA3IE		= 0;                        // NC
	IEC2bits.C1IE		= 0;                        // NC
	IEC2bits.C1RXIE		= 0;                        // NC
	IEC2bits.SPI2IE		= 0;                        // NC
	IEC2bits.SPI2EIE	= 0;                        // NC

	// IEC3
	IEC3bits.FLTA1IE	= 0;                        // NC
	IEC3bits.RTCIE		= 0;                        // NC
	IEC3bits.DMA5IE		= 0;                        // NC
	IEC3bits.QEI1IE		= ACTIV_INTER_QEI1;         // Interrupt Codeur Droit
	IEC3bits.PWM1IE		= 0;                        // NC

	// IEC4
	IEC4bits.DAC1LIE	= 0;                        // NC
	IEC4bits.DAC1RIE	= 0;                        // NC
	IEC4bits.QEI2IE		= 0;                        // NC
	IEC4bits.FLTA2IE	= 0;                        // NC
	IEC4bits.PWM2IE		= 0;                        // NC
	IEC4bits.C1TXIE		= 0;                        // NC
	IEC4bits.DMA7IE		= 0;            			// NC
	IEC4bits.DMA6IE		= 0;                    	// NC
	IEC4bits.CRCIE		= 0;        				// NC
	IEC4bits.U2EIE		= 0;                		// NC
	IEC4bits.U1EIE		= 0;                        // NC
}