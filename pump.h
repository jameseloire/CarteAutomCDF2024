
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H
#endif	


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */



#include <xc.h> // include processor files - each processor file is guarded.  

#include "system.h"



// Declaration of functions to turn on pumps
void turnOnPump_ARM1();
void turnOnPump_ARM2();
void turnOnPump_ARM3();
void turnOnPump_STOCK1();
void turnOnPump_STOCK2();
void turnOnPump_STOCK3();

// Declaration of functions to turn off pumps
void turnOffPump_ARM1();
void turnOffPump_ARM2();
void turnOffPump_ARM3();
void turnOffPump_STOCK1();
void turnOffPump_STOCK2();
void turnOffPump_STOCK3();




