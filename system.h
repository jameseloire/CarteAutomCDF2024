

#ifndef SYSTEM_H
#define	SYSTEM_H

#ifdef	__cplusplus
extern "C" {
#endif

/******************************************************************************/
/******************************** INCLUDES ************************************/
/******************************************************************************/
    
    #include <libpic30.h>
    #include <p33FJ128MC804.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <stdarg.h>
    #include <stdbool.h>
   // #include "codeurs.h"
    #include <math.h>
    #include "Uart.h"
    #include "interrupts.h"
    //#include "asserv.h"
  //  #include "pwm.h"
    #include "Config_robots.h"
   // #include "fonctions_deplacements.h"
    #include "gestion_AX12.h"
    #include "serialusM2M.h"
    #include "autom.h"
  //  #include "Serialus.h"
  //  #include "evitement.h"
   // #include "RF_uart.h"


/******************************************************************************/
/***************************** Defines ****************************************/
/******************************************************************************/


    //Macro pour les tempos
#define FOSC        80000000LL     //80017142LL      //80000000
#define FCY             (FOSC/2)
#define delay_us(x) __delay32(((x*FCY)/1000000L))   // delays x us
#define delay_ms(x) __delay32(((x*FCY)/1000L))      // delays x ms


/******************************************************************************/
/********************************* Timers *************************************/
/******************************************************************************/
 
#define ACTIVE                  0b1
#define DESACTIVE               0b0

#define DIGITAL                 1
#define ANALOGIQUE              0
    
#define ETAT_HAUT               1
#define ETAT_BAS                0

#define TIMER_5ms               T1CONbits.TON
#define TIMER_10ms              T2CONbits.TON
#define TIMER_20ms              T3CONbits.TON
#define TIMER_100ms             T4CONbits.TON
#define TIMER_200ms             T5CONbits.TON

#define FLAG_TIMER_5ms          IFS0bits.T1IF
#define FLAG_TIMER_10ms         IFS0bits.T2IF
#define FLAG_TIMER_20ms         IFS0bits.T3IF
#define FLAG_TIMER_100ms        IFS1bits.T4IF
#define FLAG_TIMER_200ms        IFS1bits.T5IF

// Pressure Sensor Pairs
#define PRESSURE_SENSOR_SCK1    PORTAbits.RA8 // SCK1
#define PRESSURE_SENSOR_OUT1    PORTAbits.RA3 // OUT1

#define PRESSURE_SENSOR_SCK2    PORTAbits.RA2 // SCK2
#define PRESSURE_SENSOR_OUT2    PORTCbits.RC2
    
#define PRESSURE_SENSOR_SCK3    PORTCbits.RC0
#define PRESSURE_SENSOR_OUT3    PORTCbits.RC1

// Presence Sensors
#define SENSOR_PRESENCE1    PORTCbits.RC3 // Previously CAPTEUR1
#define SENSOR_PRESENCE2    PORTCbits.RC4 // Previously CAPTEUR2
#define SENSOR_PRESENCE3    PORTCbits.RC5 // Previously CAPTEUR3

// Color Sensors
#define SENSOR_COLOR1       PORTBbits.RB4 // Previously CAPTEUR4
#define SENSOR_COLOR2       PORTAbits.RA4 // Previously CAPTEUR5
#define SENSOR_COLOR3       PORTAbits.RA9 // Previously CAPT6

// UART for AX18
#define UART_AX18_TX        PORTBbits.RB2
#define UART_AX18_RX        PORTBbits.RB3

// UART for Raspberry Pi (RPi)
#define UART_RPI_TX         PORTBbits.RB6
#define UART_RPI_RX         PORTBbits.RB5

  
#define ENCODER_B           PORTBbits.RB12
#define ENCODER_A           PORTBbits.RB14

    
    // Pumps on PORTB as PUMP_ARM
#define PUMP_ARM1           PORTBbits.RB7
#define PUMP_ARM2           PORTBbits.RB8
#define PUMP_ARM3           PORTBbits.RB9

// Pumps on PORTC as PUMP_STOCK
#define PUMP_STOCK1         PORTCbits.RC9
#define PUMP_STOCK2         PORTCbits.RC8
#define PUMP_STOCK3         PORTCbits.RC6
    
    
    // PWM outputs for turbines on PORTB
#define PWM_TURBINE1        PORTBbits.RB15 // PWM1L1 (PWM_Turb3_L)
#define PWM_TURBINE2        PORTBbits.RB13 // PWM1L2 (PWM_Turb1_L)
#define PWM_TURBINE3        PORTBbits.RB11 // PWM1L3 (PWM_Turb2_L)

// PWM output for servo on PORTC
#define SERVO               PORTCbits.RC7 // PWM2L1 (PWM_Servo_L)
    
    
    
#define INHIBIT_AX12        PORTAbits.RA0
#define DIR_UART_AX12       PORTAbits.RA1


/******************************************************************************/
/******************************* Interruptions  *******************************/
/******************************************************************************/

#define PRIO_INTER_TIMER1               6   // Timer Assev
#define PRIO_INTER_TIMER2               4   // Timer Evitement
#define PRIO_INTER_TIMER3               3   // Timer Autom
#define PRIO_INTER_TIMER4               5   // Timer scheduleur
#define PRIO_INTER_TIMER5               1   // Timer Serialus


#define PRIO_INTER_UART1_RX             2    
#define PRIO_INTER_UART2_TX             4   // AX12
#define PRIO_INTER_UART2_RX             4   // AX12
    
#define PRIO_INTER_QEI1                 7   // Codeurs : prio la plus haute
#define PRIO_INTER_QEI2                 7   // Codeurs : prio la plus haute


#define ACTIV_INTER_TIMER1              1   // Timer asserv : 5 ms
#define ACTIV_INTER_TIMER2              1   // Osef : TIMER 2 et 3 sur 32 bits    
#define ACTIV_INTER_TIMER3              1   // Timer 90 secondes : fin de match
#define ACTIV_INTER_TIMER4              1   // Timer Autom : 10 ms
#define ACTIV_INTER_TIMER5              1   // Timer debug : 200 ms

    
#define ACTIV_INTER_UART1_TX            1   // Uart Xbee : Tx
#define ACTIV_INTER_UART1_RX            1   // Uart XBEE : Rx
#define ACTIV_INTER_UART2_TX            1   // UART AX12 : Acquittement trame envoyée
#define ACTIV_INTER_UART2_RX            1   // UART AX12
    
#define ACTIV_INTER_QEI1                1   // Codeurs
#define ACTIV_INTER_QEI2                1   // Codeurs

/******************************************************************************/
/********************** DEFINITION DES STRUCTURES *****************************/
/******************************************************************************/

typedef struct
{
    uint32_t t_ms   :31;
    _Bool actif     : 1;
}_compteur_temps_match;

/******************************************************************************/
/*************************** Variables Globales *******************************/
/******************************************************************************/

    /**
     * Toutes les variables appellées par la le timer d'asserv de 5 ms
     * sont définies en near 
     * Elles sont ainsi stockée dans la mémoire de 8 ko proche du cpu
     * et on y accède plus rapidement
     */
    
    /**
     * Toutes les variables dont la valeurs est succeptibles d'être modifiée
     * dans une interruption doivent être déclarée volatile !
     */
    
    
    extern volatile __attribute__((near)) _compteur_temps_match CPT_TEMPS_MATCH;

  
    extern volatile _ax12 ax12;
    extern volatile _Bool CHECK_LIMITATION_COURANT;
    
 

    

/******************************************************************************/
/****************************** Prototypes ************************************/
/******************************************************************************/

    void ConfigADC (void);

    /**
     * Fonction qui initialise tous les systèmes de la carte
     * Uart, Timer, PWM, mapping ....
     */
    void init_system (void);


    /**
     * Fonction qui rèfle l'horloge à 80Mhz
     */
    void init_clock(void);
    
    /**
     * Fonction qui initialise le compteur de temps de match à zéro
     */
    void init_compteur_temps_match();

    /**
    * Configuration du Timer de 5 ms pour l'asservissement
    *        /!\  ATTENTION : TIMER SAFE   /!\
    * /!\ INTERDICTION D'UTILISER DES FONCTIONS BLOQUANTES /!\
    * /!\ OU DE DESACTIVER LE DECOMPTE DU TIMER PENDANT   /!\
    * /!\ L'INTERRUPTION, MERCI !                         /!\       
    */
    void config_timer_5ms();


   /**
    * Configuration du Timer de 10 ms pour l'évitement
    *        /!\  ATTENTION : TIMER SAFE   /!\
    * /!\ INTERDICTION D'UTILISER DES FONCTIONS BLOQUANTES /!\
    * /!\ OU DE DESACTIVER LE DECOMPTE DU TIMER PENDANT   /!\
    * /!\ L'INTERRUPTION, MERCI (PAS D'AX12) !            /!\       
    */
    void config_timer_10ms();


    /**
    * Configuration du Timer de 20 ms pour l'autom
    *      NB : TIMER NON SAFE 
    *  POSSIBILITE D'UTILISER DES FONCTIONS BLOQUANTES
    *  UTILISE POUR DES TACHES NON CRITIQUES     
    */
    void config_timer_20ms();
    
    
    /**
    * Configuration du Timer de 100 ms pour le scheduler temps de match
    *      NB : TIMER NON SAFE 
    *  POSSIBILITE D'UTILISER DES FONCTIONS BLOQUANTES
    *  UTILISE POUR DES TACHES NON CRITIQUES     
    */
    void config_timer_100ms();
    
    
    /**
    * Configuration du Timer de 200 ms pour la liaison serie (debug + serialus)
    *      NB : TIMER NON SAFE 
    *  POSSIBILITE D'UTILISER DES FONCTIONS BLOQUANTES
    *  UTILISE POUR DES TACHES NON CRITIQUES     
    */
    void config_timer_200ms();


    /**
     * Fonction qui configure toutes les interruptions
     * Ainsi que les priorités
     */
    void ConfigInterrupt (void);


 
    void ConfigPorts (void);


 
    void ConfigMapping (void);

 
    void homologation();
    
    
    void test_AX12();
    


    void init_AX12(); 

    void setPWMOutput(char output, unsigned int percent);
    void ConfigPWM (void );
    
#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEM_H */

