/*
 * File:   pmw.c
 * Author: james
 *
 * Created on April 1, 2024, 4:08 PM
 */



#include"system.h"

// configuration of the pwm register

void ConfigPWM (void)
{
	
	P1TCONbits.PTEN		= 1;		//PWM1 ON
    P2TCONbits.PTEN     = 1;        //PWM2 ON

    P1TCONbits.PTSIDL       = 0;            //cpu run in idle
    P1TCONbits.PTOPS        = 0;            //Postscaller 1 : 1 (no need )
    P1TCONbits.PTCKPS       = 0b11;            //Prescaller  1 : 64 controling 50hz servo 
	P1TCONbits.PTMOD        = 0b00;	    	// Single event mode using pwm only when need
   
     
    P2TCONbits.PTSIDL       = 0;            //cpu run in idle
    P2TCONbits.PTOPS        = 0;            //Postscaller 1 : 1 (no need )
    P2TCONbits.PTCKPS       = 0b11;            //Prescaller  1 : 64 controling 50hz servo 
	P2TCONbits.PTMOD        = 0b00;		// Single event mode using pwm only when need
    
    
    //P1TMRbits.PTMR =0;
    //P1TMRbits.PTDIR =01; // counting up
        P1TPER     = 10417;		//F=60Hz  15 bits PTPER = FCY/F*prescale*postscale -1 
    // 40Mz/50*64*1 -1 = 12500
    
   
    //P2TMRbits.PTMR =0;
    //P2TMRbits.PTDIR =01; // counting up
    P2TPER     = 12500;		//F=50Hz  15 bits PTPER = FCY/F*prescale*postscale -1 
    // 40Mz/50*64*1 -1 = 12500
   
    
    //P1SECMP=0;  // no output compare inactive
    //P2SECMP=0;
    
    

	PWM1CON1bits.PMOD1      = 1;		//  indepedant mode             
	PWM1CON1bits.PMOD2      = 1;		//              
    PWM1CON1bits.PMOD3      = 1;        //          

    PWM2CON1bits.PMOD1      = 1;
    
	
	PWM1CON1bits.PEN1H	= 0;		//PWM1H1 
    PWM1CON1bits.PEN1L	= 1;		//PWM1L1 PWM_Turb3_L
    
	PWM1CON1bits.PEN2H	= 0;		//PWM1H2
	PWM1CON1bits.PEN2L	= 1;		//PWM1L2 PWM_Turb1_L
    
	PWM1CON1bits.PEN3H	= 0;		//PWM1H3 
	PWM1CON1bits.PEN3L	= 1;		//PWM1L3 PWM_Turb2_L 

	PWM2CON1bits.PEN1H	= 0;		//PWM2H1 
	PWM2CON1bits.PEN1L	= 1;		//PWM2L1  PWM_Servo_L
    
    PWM1CON2bits.IUE    = 1; // imediate update no transient pwm while udpade
    PWM2CON2bits.IUE    = 1;
    
    //PWM1CON2bits.SEVOPS = 0; // 1:1 poscale
    //PWM2CON2bits.SEVOPS = 0; // 1:1 poscale ( no use here)
    
    //PWM1CON2bits.OSYNC = 1; //synchronized output override wrt to TMR
    //PWM1CON2bits.UDIS = 0; // enable regular update of duty cycle
    
    //PWM2CON2bits.OSYNC = 1; //synchronized output override wrt to TMR
    //PWM2CON2bits.UDIS = 0; // enable regular update of duty cycle
    
    
    // no dead time as using servo (not Hbridge)
    //P1DTCON1 = 0;
    //P1DTCON2 = 0;
    
    //P2DTCON1 = 0;
    //P2DTCON2 = 0;
    
    // no fault pin use 
   
    //P1FLTACON = 0;
    //P2FLTACON = 0;
    
    // override control reg
    
    //P2OVDCONbits.POUT1H = 0;
    //P2OVDCONbits.POUT1L = 0; // inactive
 
    //P2OVDCONbits.POVD1L = 1; // control by the pwm generator
    //P2OVDCONbits.POVD1H = 1;
    
    
    //P1OVDCONbits.POUT1H = 0;
    //P1OVDCONbits.POUT1L = 0; // inactive
 
    //P1OVDCONbits.POVD1L = 1; // control by the pwm generator
    //P1OVDCONbits.POVD1H = 1;
    
    //P1OVDCONbits.POUT2H = 0;
    //P1OVDCONbits.POUT2L = 0; // inactive
 
    //P1OVDCONbits.POVD2L = 1; // control by the pwm generator
    //P1OVDCONbits.POVD2H = 1;
    
    
    //P1OVDCONbits.POUT3H = 0;
    //P1OVDCONbits.POUT3L = 0; // inactive
 
    //P1OVDCONbits.POVD3L = 1; // control by the pwm generator
    //P1OVDCONbits.POVD3H = 1;
    
    
    P1DC1 = 0; // servo operate in 1->2ms on 50hz so 20ms so 12500/20 = 625 ticks/ms 
    // here we set to 0% so 1ms
    
    P1DC2 = 0; // servo operate in 1->2ms on 50hz so 20ms so 12500/20 = 625 ticks/ms 
    // here we set to 0% so 1ms
    
    P1DC3 = 0; // servo operate in 1->2ms on 50hz so 20ms so 12500/20 = 625 ticks/ms 
    // here we set to 0% so 1ms
    
    P2DC1 = 0; // servo operate in 1->2ms on 50hz so 20ms so 12500/20 = 625 ticks/ms 
    // here we set to 0% so 1ms

}


void setPWMOutput(char output,  uint16_t percent) {
     double dutyCycleValue;

    if (percent > 100){
        
        percent = 100; // Cap the percentage at 100
    }
    dutyCycleValue = 1 + (percent *( (5000) / 100)); // 0%-100% maps to 0ms-ms
    
    switch(output) {
        case '3':
            
            P1DC1 = (uint16_t) dutyCycleValue; 
           
            break;
        case '1':
           
            P1DC2 = (uint16_t) dutyCycleValue; 
            break;
        case '2':
           
            P1DC3 = (uint16_t) dutyCycleValue; 
            break;
        case '4':
           
            P2DC1 = (uint16_t) dutyCycleValue;
            break;
        default:
            // Handle invalid output
            break;
    }

  
}
