
 
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H
#endif
#include <xc.h>

#include "system.h"
#include "pump.h"// include processor files - each processor file is guarded.

#ifdef	__cplusplus
extern "C" 
#endif
 
    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code.
 
//#define FACE_A
#define FACE_B



void readSensors();
void test_pump();
void destock_plant(bool A1, bool A2, bool A3);
void init_ax();
void start_turb(bool A1, bool A2, bool A3);
void stop_turb(bool A1, bool A2, bool A3);
void open_servo();
void close_servo();
void stock_plant(bool A1, bool A2, bool A3);
void hands_up();
void hands_down();
void depose_jard(bool A1, bool A2, bool A3);



#define delay_pompe 100
#define VIT_AX 200
#define VIT_AX_BIELLE 500
#define DELAY_AUTOM 1000
#define DELAY_ACCEL_TURB 3
#define TURB_MAX 2000



#ifdef FACE_A
#undef FACE_B


#define ID_BIELLE 1
#define ID_TOP1 2
#define ID_BTM1 3
#define ID_TOP2 4
#define ID_BTM2 5
#define ID_TOP3 6
#define ID_BTM3 7
 
#define POS_BIELLE_UP 580 //caleur test a changer
#define POS_BIELLE_MID 312
#define POS_BIELLE_DOWN 245
#define POS_BIELLE_DEPOSE 260
#define POS_BIELLE_PUSH 400
#define POS_BIELLE_PULL 320


#define POS_TOP1_STOCK 760
#define POS_TOP1_MID 467
#define POS_TOP1_GRAB 201
#define POS_TOP1_DEPOSE 430


#define POS_BTM1_STOCK 210
#define POS_BTM1_MID 220
#define POS_BTM1_GRAB 212
#define POS_BTM1_DEPOSE 377


#define POS_TOP2_STOCK 760
#define POS_TOP2_MID 467
#define POS_TOP2_GRAB 194
#define POS_TOP2_DEPOSE 430
 
#define POS_BTM2_STOCK 222
#define POS_BTM2_MID 220
#define POS_BTM2_GRAB 208
#define POS_BTM2_DEPOSE 377
 
#define POS_TOP3_STOCK 760
#define POS_TOP3_MID 467
#define POS_TOP3_GRAB 202
#define POS_TOP3_DEPOSE 415


#define POS_BTM3_STOCK 196
#define POS_BTM3_MID 220
#define POS_BTM3_GRAB 202
#define POS_BTM3_DEPOSE 376


#endif





#ifdef FACE_B

#undef FACE_A

#define ID_BIELLE 1
#define ID_TOP1 2
#define ID_BTM1 23
#define ID_TOP2 4
#define ID_BTM2 5
#define ID_TOP3 6
#define ID_BTM3 7
 
#define POS_BIELLE_UP 560  //caleur test a changer
#define POS_BIELLE_MID 400
#define POS_BIELLE_DOWN 244
#define POS_BIELLE_DEPOSE 260
#define POS_BIELLE_PUSH 400
#define POS_BIELLE_PULL 320

/* valeurs de A
580
312
245
*/
 
#define POS_TOP1_STOCK 785
#define POS_TOP1_MID 500
#define POS_TOP1_GRAB 215
#define POS_TOP1_DEPOSE 430

 
#define POS_BTM1_STOCK 430
#define POS_BTM1_MID 350
#define POS_BTM1_GRAB 220
#define POS_BTM1_DEPOSE 377

 
#define POS_TOP2_STOCK 785
#define POS_TOP2_MID 500
#define POS_TOP2_GRAB 215
#define POS_TOP2_DEPOSE 430

 
#define POS_BTM2_STOCK 430
#define POS_BTM2_MID 350
#define POS_BTM2_GRAB 220
#define POS_BTM2_DEPOSE 377

 
#define POS_TOP3_STOCK 785
#define POS_TOP3_MID 500
#define POS_TOP3_GRAB 215
#define POS_TOP3_DEPOSE 415

 
#define POS_BTM3_STOCK 430
#define POS_BTM3_MID 350
#define POS_BTM3_GRAB 220
#define POS_BTM3_DEPOSE 376 


#endif 
    



