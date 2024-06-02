/*
 * File:   pump.c
 * Author: james
 *
 * Created on March 31, 2024, 10:59 AM
 */


// Include the header where PORTB and PORTC are defined, e.g., xc.h or your specific device header.
#include "xc.h"
#include "system.h"
#include "pump.h"
// Make sure to include the microcontroller's specific header file that defines LATA and LATC


// Implementations of functions to turn on pumps
void turnOnPump_ARM1() {
    LATBbits.LATB7 = 1; // Assuming PUMP_ARM1 is on LATBbits.LATB7
}

void turnOnPump_ARM2() {
    LATBbits.LATB8 = 1; // Adjusted to LAT register
}

void turnOnPump_ARM3() {
    LATBbits.LATB9 = 1; // Adjusted to LAT register
}

void turnOnPump_STOCK1() {
    LATCbits.LATC9 = 1; // Adjusted to LAT register
}

void turnOnPump_STOCK2() {
    LATCbits.LATC8 = 1; // Adjusted to LAT register
}

void turnOnPump_STOCK3() {
    LATCbits.LATC6 = 1; // Adjusted to LAT register
}

// Implementations of functions to turn off pumps
void turnOffPump_ARM1() {
    LATBbits.LATB7 = 0;
}

void turnOffPump_ARM2() {
    LATBbits.LATB8 = 0;
}

void turnOffPump_ARM3(void) {
    LATBbits.LATB9 = 0;
}

void turnOffPump_STOCK1(void) {
    LATCbits.LATC9 = 0;
}

void turnOffPump_STOCK2(void) {
    LATCbits.LATC8 = 0;
}

void turnOffPump_STOCK3(void) {
    LATCbits.LATC6 = 0;
}


