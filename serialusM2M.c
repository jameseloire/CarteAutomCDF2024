// serialusM2M.c

#include "serialusM2M.h"
#include "Uart.h" // Include your UART code here
#include "system.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

SerialusM2M serialusM2M = {
    .head = 0,
    .tail = 0,
    .carriageReturnReceived = false,
    .FLAG_CHECK_CAPT = false
};





void serialusM2M_init()
{
    InitUART(UART_XBEE, 1000000);
    serialusM2M.clignotement_en_cours = false;
}

void serialusM2M_receive()
{
    // Code to read incoming serial data and place it in the circular buffer
    // You can use your UART functions here to read characters.
    // For example:

    while (U1STAbits.URXDA == 1)
    {                                         // U1STAbits.URXDA checks if data is available
        volatile char receivedChar = U1RXREG; // Read a character from UART1
        PutcUART(UART_XBEE, receivedChar);

        serialusM2M.buffer[serialusM2M.head] = receivedChar;
        serialusM2M.head = (serialusM2M.head + 1) % BUFFER_SIZE;

        // Check for '\r' (carriage return) to mark the end of a command
        if (receivedChar == '\r')
        {
            // putchar('\r');
            putchar('\n');
            serialusM2M.carriageReturnReceived = true;
        }
    }
}

void reset_rx_Buffer()
{
    while (U1STAbits.URXDA)
    {
        (void)U1RXREG; // Read and discard the character
    }
}

void serialusM2M_process()
{
    // Process the data in the circular buffer

    while (serialusM2M.head != serialusM2M.tail)
    {
        volatile char currentChar = serialusM2M.buffer[serialusM2M.tail];
        serialusM2M.tail = (serialusM2M.tail + 1) % BUFFER_SIZE;

        // Check for '\r' (carriage return) to mark the end of a command
        if (currentChar == '\r')
        {
            // Process the complete command here or set a flag to indicate a complete command
            if (serialusM2M.carriageReturnReceived)
            {
                // printf(serialusM2M.buffer);
                //  Complete command received, process it
                char id0 = serialusM2M.buffer[0];
                char id1;
                char id2;
                int taille_buffer = serialusM2M.tail;

                switch (id0)
                {
                case '0'://test
                    printf("goodtest\n");
                    break;
                    
                case '4': // AX
                    id1 = serialusM2M.buffer[1];
                    switch (id1)
                    {
                    case '0': ////AX_set
                        id2 = serialusM2M.buffer[2];
                        switch (id2)
                        {
                        case '0':
                            if ((taille_buffer) != 13)
                            {
                                printf("erreur taille char AX %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_set_ang();
                            break;
                        case '1':
                            if ((taille_buffer) != 14)
                            {
                                printf("erreur taille char AX %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_set_pos();
                            break;
                        case '2': 
                            if ((taille_buffer) != 7)
                            {
                                printf("erreur taille char AX %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_set_led();
                            break;
                        case '3': // 
                            if ((taille_buffer) != 7)
                            {
                                printf("erreur taille char %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_set_torque();
                            break;
                        case '4': // ()
                            if ((taille_buffer) != 5)
                            {
                                printf("erreur taille char %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_set_alim();
                            break;

                        default:
                            printf("non reconu char3 AXset");
                            break;
                        }
                        break;

                    case '1': ////AX_get
                        id2 = serialusM2M.buffer[2];
                        switch (id2)
                        {
                        case '0':
                            if ((taille_buffer) != 6)
                            {
                                printf("erreur taille char AX %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_get_pos();
                            break;
                        case '1':
                            if ((taille_buffer) != 6)
                            {
                                printf("erreur taille char AX %d \n", taille_buffer);
                                break;
                            }
                            else if (!sontdesdigits(serialusM2M.buffer))
                            {
                                printf("erreur Sont des digits \n");
                                break;
                            }
                            AX_get_ping();
                            break;


                        default:
                            printf("non reconu char3 AXget");
                            break;

                        }
                        break;

                    default:
                            printf("non reconu char2 AX \n");
                            break;
                    }
                    break;
                 
                    
                      case '5':
                        //printf("you are here \n");
                         id1 = serialusM2M.buffer[1];
                        switch (id1)
                        { 
                        case '0': ////////////////////////////////////////////////////////test pomp
                            test_pompes();
                            //printf("test pompes \n");
                            break;
                            
                        case '1':
                            //printf("test pompinettes \n");
                            test_pompinettes();
                            break;
                            
                        case '2':
                            test_esc();
                            //printf("test esc \n");
                            break;
                            
                        case '3':
                            //printf("test servo \n");
                            test_servo();
                            break;
                           
                                
                            
                        default  :
                            printf ("Mauvais id1 test autom /n");
                            break;
                        }
                                
                                      
                                
                        
                        break; //break case 5
                        
                      
                    case '6':
                        //printf("you are here \n");
                         id1 = serialusM2M.buffer[1];
                        switch (id1)
                        { 
                        case '0': ////////////////////////////////////////////////////////test pomp
                            init_ax();
                            printf("initAX \n");
                            break;
                            
                        case '1':
                            
                            _start_turb();
                           
                            break;
                            
                        case '2':
                            readSensors();
                            
                            break;
                            
                        case '3':
                            
                            _stop_turb();
                            break;
                            
                       case '4':
                            
                            open_servo();
                             printf("open servo \n");
                            break;
                           
                        case '5':
                            
                            close_servo();
                             printf("close servo \n");
                            break;
                        
                        case '6':
                            
                            _stock_plant();
                             printf("stock \n");
                            break;
                            
                            
                        case '7':
                            
                             _destock_plant();
                             printf("destock");
                            break;
                           
                            
                        case '8':
                            
                             _start_sensor();
                            // printf("captint to be coded and print removed \n");
                            break;
                           
                        case '9':
                            start_match();
                            
                            break;
                            
                                    
                            
                        default  :
                            printf ("Mauvais id1  autom /n");
                            break;
                        }
                                
                                      
                                
                        
                        break; //break case 6
                     
                        
                    case '7':
                        
                        //printf("you are here \n");
                         id1 = serialusM2M.buffer[1];
                        switch (id1)
                        { 
                        case '0': ////////////////////////////////////////////////////////test pomp
                            turnOnPump_ARM1();
                            turnOnPump_ARM2();
                            turnOnPump_ARM3();
                            printf("pump on\n");
                            break;
                            
                        case '1': ////////////////////////////////////////////////////////test pomp
                            turnOffPump_ARM1();
                            turnOffPump_ARM2();
                            turnOffPump_ARM3();
                            printf("pump off\n");
                            break;
                        
                            
                        case '2':
                            
                            _hands_up();
                            break;
                            
                            
                        case '3':
                            _depose_jard();
                            break;
                                
                        case '4':
                            
                            _hands_down();
                            break;              
           
                        
                        break; //break case 7
                       
                        }
                             
                            
                default:
                    printf("connais pas \n");
                    break;
                }

                // Clear the flag
                serialusM2M.carriageReturnReceived = false;

                // Reset the circular buffer
                serialusM2M.head = 0;
                serialusM2M.tail = 0;
                reset_rx_Buffer();
            }
        }
    }
}


void AX_set_ang(){
    uint8_t id = 0;
    int16_t angle =0;
    uint16_t v = 0;

    char charID[3];
    char charAngle[4];
    char charV[5];

    // Extract id, angle, and v substrings from the buffer
    strncpy(charID, serialusM2M.buffer + 3, 2);
    strncpy(charAngle, serialusM2M.buffer + 5, 3);
    strncpy(charV, serialusM2M.buffer + 8, 4);

    // Null-terminate the substrings
    charID[2] = '\0';
    charAngle[3] = '\0';
    charV[4] = '\0';

    // Convert the substrings to double

    id = (uint8_t) atoi(charID);
    angle = (int16_t) atoi(charAngle);
    v = (uint16_t) atoi(charV);

    if (!(id >= 0 && id <= ID_MAX_AX12 && abs(angle) <= 270 && v >= 0 && v <= 1023))
    {
        printf("data inchoerante ID: %i, angle: %i , v: %i \n ", (uint8_t)id, (int16_t)angle, (uint16_t)v);
    }
    else
    {

        synchro_AX12(id, (float)angle, v, SANS_ATTENTE);
        print_erreur_ax12();

    }

}

void AX_set_pos(){

    uint8_t id = 0;
    int16_t position =0;
    uint16_t v = 0;

    char charID[3];
    char charPosition[5];
    char charV[5];

    // Extract id, angle, and v substrings from the buffer
    strncpy(charID, serialusM2M.buffer + 3, 2);
    strncpy(charPosition, serialusM2M.buffer + 5, 4);
    strncpy(charV, serialusM2M.buffer + 9, 4);

    // Null-terminate the substrings
    charID[2] = '\0';
    charPosition[4] = '\0';
    charV[4] = '\0';

    // Convert the substrings to double

    id = (uint8_t) atoi(charID);
    position = (int16_t) atoi(charPosition);
    v = (uint16_t) atoi(charV);

    if (!((id >= 0 && id <= ID_MAX_AX12 && position>=0 && position <= 1023 && v >= 0 && v <= 1023)))
    {
        printf("data inchoerante ID: %d, position: %d , v: %d \n ", id, position, v);
    }
    else
    {

        angle_AX12(id, position, v, SANS_ATTENTE);
        print_erreur_ax12();

    }


}

void AX_set_led(){
    uint8_t id = 0;
    uint8_t state = 0;

    char charID[3];
    char charState[2];

    strncpy(charID, serialusM2M.buffer + 3, 2);
    strncpy(charState, serialusM2M.buffer + 5, 1);

    id = (uint8_t) atoi(charID);
    state = (uint8_t) atoi(charState);

    if (state){

        allumer_LED_AX12((uint8_t)id);
        print_erreur_ax12();
    }
    else if (!state){

        eteindre_LED_AX12((uint8_t) id);
        print_erreur_ax12();
    }

    else {
        printf("on//off AX Led error\n");
    }
}

void AX_set_torque(){
    uint8_t id = 0;
    uint8_t state = 0;

    char charID[3];
    char charState[2];

    strncpy(charID, serialusM2M.buffer + 3, 2);
    strncpy(charState, serialusM2M.buffer + 5, 1);

    id = (uint8_t) atoi(charID);
    state = (uint8_t) atoi(charState);

    if (state){

        torque_enable_ax12((uint8_t)id, true);
        print_erreur_ax12();
    }
    else if (!state){

        torque_enable_ax12((uint8_t)id, false);
        print_erreur_ax12();
    }

    else {
        printf("on//off AX torque Error \n");
    }

}

void AX_set_alim(){

    uint8_t state = 0;

    char charState[2];

    strncpy(charState, serialusM2M.buffer + 3, 1);

    state = (uint8_t) atoi(charState);

    if (state){
        printf("iAX -> on \n");
        INHIBIT_AX12 = ALLUME;
    }
    else if (!state){
        printf("AX->  off \n");
        INHIBIT_AX12 = ETEINT;
    }

    else {
        printf("on//off AX alim Error \n");
    }

}

void AX_get_pos(){
    int16_t id = 0;

    char charID[3];

    strncpy(charID, serialusM2M.buffer + 3, 2);

    id = (int16_t) atoi(charID);
    
    print_position_ax12(id, read_data(id, LIRE_POSITION_ACTU));
     printf("\n");


}

void AX_get_ping(){
    int16_t id = 0;

    char charID[3];

    strncpy(charID, serialusM2M.buffer + 3, 2);

    id = (int16_t) atoi(charID);

     // si c'est un id
    id = check_id_ax12_m2m(id);
    if (id == TOUS_LES_AX12)
    {
        uint8_t id = 0;
        for (id = 0 ; id < ID_MAX_AX12 ; id++)
        {
            if (Ping(id) == REPONSE_OK)
            {
                print_ping(id);
            }
            
        }
    printf("\n");
    }
    else if (id != -1)
    {
        if (Ping(id) == REPONSE_OK)
        {
            print_ping(id);
            printf("\n");
        }
        else 
            printf("NC");//Not Conected
            printf("\n");
    }
} 


void print_erreur_ax12()
{
    switch(ax12.erreur)
    {
        case PAS_D_ERREUR :  
            printf("0\n");
            break;
        case TIME_OUT :
            printf("1\n");
            break;
        case ERREUR_CS :
            printf("2\n");
            break;
        case LIMITATION_DE_COURANT :
            printf("3\n");
            break;
        case AUTRE_ERREUR :
            printf("5\n");
        default :
            break;
    }  
}

void print_position_ax12(uint8_t id, int16_t position)
{
    printf(" ID : %d =%d \r", id, position);
}

void print_ping(uint8_t id)
{
    printf("ID : %d \r", id);
}

int16_t check_id_ax12_m2m(int16_t id)
{
    if (!(id >= 0 && id <= ID_MAX_AX12))
            id = -1;
    
    else if (id==99)
    {
        id = TOUS_LES_AX12;
    }
    else 
        id =-1;
    
    if (id == -1)
        print_abort("4\n ");
    
    return id;
}


//////////////////////////////////////////////////////////////test autom////////////////////////////////////////////////////////////

void test_pompes(){
    printf("pomp on");
    turnOnPump_ARM1();
    turnOnPump_ARM2();
    turnOnPump_ARM3();
    
    delay_ms(10000);
   
    turnOffPump_ARM1();
    turnOffPump_ARM2();
    turnOffPump_ARM3();
      printf("pomp off \n");
}

void test_pompinettes(){
    printf("pompinettes on");
    turnOnPump_STOCK1();
    turnOnPump_STOCK2();
    turnOnPump_STOCK3();
    
    delay_ms(30000);
    
    turnOffPump_STOCK1();
    turnOffPump_STOCK2();
    turnOffPump_STOCK3(); 
    printf("pompinettes off \n");
}

void test_esc(){
    printf("turbine on");
              int i;
        for (i=500;i<3000;++i){
             P1DC1 = (uint16_t) i; 
             P1DC2 = (uint16_t) i; 
             P1DC3 = (uint16_t) i; 
            delay_ms(3);
      } 
       printf("turbine full on");        
              delay_ms(5000);
          
        for (i=3000;i>499;--i){
             P1DC1 = (uint16_t) i; 
             P1DC2 = (uint16_t) i; 
             P1DC3 = (uint16_t) i; 
            delay_ms(3);
       
      } 
               printf("turbine off \n");
}

void test_servo(){

    

    printf("servo 3000");     
             P2DC1 = (uint16_t) 3000;  
             delay_ms(2000);


    printf("servo 450");      
             P2DC1 = (uint16_t) 500;  
             delay_ms(2000);

      
 printf("servo 0 \n");
}
////////////////////////////////////////////////autom


void _start_turb(){
     int16_t State1 = 0;
     int16_t State2 = 0;
     int16_t State3 = 0;
     
    char charState1[2];
    char charState2[2];
    char charState3[2];
    
    charState1[1] = '\0';
    charState2[1] = '\0';
    charState3[1] = '\0';

    strncpy(charState1, serialusM2M.buffer + 2, 1);
    strncpy(charState2, serialusM2M.buffer + 3, 1);
    strncpy(charState3, serialusM2M.buffer + 4, 1);
    
    State1 = (int16_t) atoi(charState1);
    State2 = (int16_t) atoi(charState2);
    State3 = (int16_t) atoi(charState3);
    


    //insert new start turbine function 
    start_turb(State1,State2,State3);
   printf("turbine on %d %d %d \n",State1,State2,State3);
    
   
}

void _read_sensor_int(){
      int16_t State1 = 0;
   
    char charState1[2];
    
    
    charState1[1] = '\0';
   

    strncpy(charState1, serialusM2M.buffer + 2, 1);
   
    
    State1 = (int16_t) atoi(charState1);
   
    
    //insert new get capteur function 
    
    printf("capt  \n");
}


void _stop_turb(){
    int16_t State1 = 0;
    int16_t State2 = 0;
    int16_t State3 = 0;
     
    char charState1[2];
    char charState2[2];
    char charState3[2];
    
    charState1[1] = '\0';
    charState2[1] = '\0';
    charState3[1] = '\0';

    strncpy(charState1, serialusM2M.buffer + 2, 1);
    strncpy(charState2, serialusM2M.buffer + 3, 1);
    strncpy(charState3, serialusM2M.buffer + 4, 1);
    
    State1 = (int16_t) atoi(charState1);
    State2 = (int16_t) atoi(charState2);
    State3 = (int16_t) atoi(charState3);
    

    
    //insert new stop turbine function 
    stop_turb(State1,State2,State3);
    printf("turbine off %d %d %d \n",State1,State2,State3);
    
}



void _destock_plant(){
     int16_t State1 = 0;
     int16_t State2 = 0;
     int16_t State3 = 0;
     
    char charState1[2];
    char charState2[2];
    char charState3[2];
    
    charState1[1] = '\0';
    charState2[1] = '\0';
    charState3[1] = '\0';

    strncpy(charState1, serialusM2M.buffer + 2, 1);
    strncpy(charState2, serialusM2M.buffer + 3, 1);
    strncpy(charState3, serialusM2M.buffer + 4, 1);
    
    State1 = (int16_t) atoi(charState1);
    State2 = (int16_t) atoi(charState2);
    State3 = (int16_t) atoi(charState3);
    
    
    //insert new destock 
    destock_plant(State1,State2,State3);
    printf("destock %d %d %d \n",State1,State2,State3);
}

void _stock_plant(){
     int16_t State1 = 0;
     int16_t State2 = 0;
     int16_t State3 = 0;
     
    char charState1[2];
    char charState2[2];
    char charState3[2];
    
    charState1[1] = '\0';
    charState2[1] = '\0';
    charState3[1] = '\0';

    strncpy(charState1, serialusM2M.buffer + 2, 1);
    strncpy(charState2, serialusM2M.buffer + 3, 1);
    strncpy(charState3, serialusM2M.buffer + 4, 1);
    
    State1 = (int16_t) atoi(charState1);
    State2 = (int16_t) atoi(charState2);
    State3 = (int16_t) atoi(charState3);
    
    
    //insert new destock 
    stock_plant(State1,State2,State3);
    printf("stock %d %d %d \n",State1,State2,State3);
}


void _start_sensor(){
    
    serialusM2M.counter_capt = 0;
    serialusM2M.FLAG_CHECK_CAPT = true;
    
}

void _hands_up(){
    
    hands_up();
    printf("hands up");
    
    
   
}


void _hands_down(){
    
    hands_up();
    printf("hands down");
    
    
   
}


void _depose_jard(){
     int16_t State1 = 0;
     int16_t State2 = 0;
     int16_t State3 = 0;
     
    char charState1[2];
    char charState2[2];
    char charState3[2];
    
    charState1[1] = '\0';
    charState2[1] = '\0';
    charState3[1] = '\0';

    strncpy(charState1, serialusM2M.buffer + 2, 1);
    strncpy(charState2, serialusM2M.buffer + 3, 1);
    strncpy(charState3, serialusM2M.buffer + 4, 1);
    
    State1 = (int16_t) atoi(charState1);
    State2 = (int16_t) atoi(charState2);
    State3 = (int16_t) atoi(charState3);
    
    
    //insert new destock 
    depose_jard(State1,State2,State3);
    printf("depose jard %d %d %d \n",State1,State2,State3);
    
}





////////////////////////////////////////////////////////////////////////////////////serialus func
bool sontdesdigits(const char *str)
{

    for (int i = 0; str[i] != '\r'; i++)
    {
        if (!isdigit(str[i]))
        {
            printf("error::");
            PutcUART(UART_XBEE, str[i]);
            return false;
        }
    }
    return true;
}
void print_abort(char* raison)
{
    printf("\n\rAbort : %s", raison);
}

    
void reset_overflow_error()
{
    U1STAbits.OERR = 0;
    printf("overflow error");
}


void start_match(){
     CPT_TEMPS_MATCH.actif = 1;
     printf("here we go again \n");
}