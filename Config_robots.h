 /******************************************************************************/
/************** Carte principale Robot 1 : DSPIC33FJ128MC804*******************/
/******************************************************************************/
/* Fichier 	: Config_robots.h
 * Auteur  	: Quentin
 * Revision	: 1.0
 * Date		: 4 décembre 2014, 12:52
 *******************************************************************************
 *  Fichier qui permet très facilemet d'adapter les paramètres pour changer
 *  de robots.
 *
 ******************************************************************************/

#ifndef CONFIG_ROBOTS_H
#define	CONFIG_ROBOTS_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#include "system.h"

/******************************************************************************/
/***************************** Defines ****************************************/
/******************************************************************************/

#define PETIT_ROBOT
//#define GROS_ROBOT // <======= la pour indiquer si on est sur le petit ou le gros 
    
//#define NO_SERIALUS       // Commanter la ligne pour avoir le serialus, décommanter pour supprimer le serialus
  #define DEBUG_ACTIF
    
//#define MODULE_RF 
//#define ARDUINO
    
#define _Pi                             3.14159265359
   
    
#ifdef GROS_ROBOT
    //#define CARTE_V1    
    #define CARTE_V2

    /**************************************************************************/
    /****************************** ODOMETRIE *********************************/
    /**************************************************************************/
    #define _ENTRAXE_MM                  300.05   //306.2 //308.6     //Pas assez tourné -> augmenter entraxe
    #define _DIAMETRE_ROUE_CODEUSE       52.2544 //51.68 //52.9 //52.91 //52.92 //51.971
    #define _PERIMETRE_ROUE_MM           (_DIAMETRE_ROUE_CODEUSE * _Pi) //194.455      // Si distance parcourue trop grande -> Augmenter la taille des roues

        //+COEF_D -> + a droite
    #define  _COEF_G                    (double) 1.00       //0.99985//0.9975 //SI tourne trop a droite : augmenter
    //1.00531 474
    #define  _COEF_D                    (double) 1.0014         //si tourne trop a gauche augmenter


    /**************************************************************************/
    /******************************** ASSERV **********************************/
    /**************************************************************************/
        //Association distance - Vitesse - Accélérations max
    #define _VITESSE_CONSIGNE_MAX_MM     4.
    #define _VITESSE_DISTANCE_MIN        0.75
    #define _VITESSE_MAX_MM_TENSION      5. //6.
    #define _DISTANCE_CONSIGNE_MM        500. 

    #define _ACC_POSITION_CONSIGNE       2.5 //5. //3.
    #define _DCC_POSITION_CONSIGNE       3. //8
    #define _ACC_POSITION_MIN            1.5
    #define _DCC_POSITION_MIN            1.75 //1.5

    #define _COEF_FREINAGE              1.25

        //Association Angle - Vitesse - Accélérations max
    #define _VITESSE_ANGLE_MAX           0.02 //0.02
    #define _VITESSE_ANGLE_MIN           0.005
    #define _ORIENTATION_CONSIGNE_DEG    90.

    #define _ACC_ORIENTATION_CONSIGNE    3. //2.
    #define _DCC_ORIENTATION_CONSIGNE    6. //6. //8.
    #define _ACC_ORIENTATION_MIN         2.
    #define _DCC_ORIENTATION_MIN         4.

        //Autres réglages asserv
    #define _SEUIL_IMMOBILITE            100ULL//2500ULL //100ULL //200
    #define _MAX_ERREUR_INTEGRALLE_V     7500. //7500 pas bon //4500.
    #define _MAX_E_INTEGRALLE_BRAKE      500.

    /**************************************************************************/
    /********************************* PID ************************************/
    /**************************************************************************/
   
    //PID
    #define _VITESSE_DIS_KP              0.14  //0.4 //0.45 //0.60 //0.80 //0.46
    #define _VITESSE_DIS_KI              0.02 //0.02
    #define _VITESSE_DIS_KD              0.7 //0.35 //0.32

    #define _POSITION_KP                 1.
    #define _POSITION_KI                 0.
    #define _POSITION_KD                 0.

    #define _ORIENTATION_KP              1.
    #define _ORIENTATION_KI              0.
    #define _ORIENTATION_KD              0.

    #define KP_BRAKE                    0.04 //0.04 //0.09
    #define KI_BRAKE                    0.0015 //0.0015 //0.0015
    #define KD_BRAKE                    0.1 //0.08    //0.8

    /**************************************************************************/
    /******************************* TENSIONS *********************************/
    /**************************************************************************/

    //Tensions d'alimentations
    #define _TENSION_MOTEUR_DROIT        24L
    #define _TENSION_MOTEUR_GAUCHE       24L

    #define _TENSION_MOTEUR_X            12L

    #define _TENSION_SORTIE_PTN          34L 

    #define _CKECK_LIMITATION_COURANT    true 


    /**************************************************************************/
    /*************************** CONFIGS MOTEURS ******************************/
    /**************************************************************************/


    //Paramètres codeurs, et moteurs
    #define _RAPPORT_REDUCTION           1U

    #define _CODEUR_D_NB_IMP             3600U
    #define _CODEUR_G_NB_IMP             3600U

    #define _RESOLUTION_LOGICIELLE       1LL

    //PWM

    
    
    #ifdef CARTE_V2
        #define _SENS_ROT_D                  1L
        #define _SENS_ROT_G                  -1L

        #define _AVANCER_MOTEUR_D            1
        #define _RECULER_MOTEUR_D            0

        #define _AVANCER_MOTEUR_G            1
        #define _RECULER_MOTEUR_G            0

        #define _AVANCER_MOTEUR_X            0
        #define _RECULER_MOTEUR_X            1

        #define _AVANCER_MOTEUR_Y            0
        #define _RECULER_MOTEUR_Y            1
    #endif

    /**************************************************************************/
    /***************************** PORTS CARTE ********************************/
    /**************************************************************************/

    
    //La définition des ports relatifs à chaque carte se trouve
    //Dans le fichier system.h
    

    #define CAPT_US_AV_DROIT            CAPTEUR3
    #define CAPT_US_AV_GAUCHE           CAPTEUR1
    #define CAPT_US_AR_DROIT            CAPTEUR8
    #define CAPT_US_AR_GAUCHE           CAPTEUR2


    // Définition des standart pour la fonction évitement
    // ETAT : etat de detection du capteur (si il détecte à un : ETAT_HAUT)
    // NB : Si un capteur n'existe pas, le mettre à AUCUN et à ETAT_HAUT
    #define _CAPT_ADV_AVANT_G           CAPT_US_AV_GAUCHE            
    #define _CAPT_ADV_AVANT_D           CAPT_US_AV_DROIT
    #define _CAPT_ADV_AVANT_C           AUCUN

    #define _ETAT_ADV_AVANT_G           ETAT_HAUT
    #define _ETAT_ADV_AVANT_D           ETAT_HAUT
    #define _ETAT_ADV_AVANT_C           ETAT_HAUT

    #define _CAPT_ADV_ARRIERE_G         CAPT_US_AR_GAUCHE
    #define _CAPT_ADV_ARRIERE_C         AUCUN
    #define _CAPT_ADV_ARRIERE_D         CAPT_US_AR_DROIT

    #define _ETAT_ADV_ARRIERE_G         ETAT_HAUT
    #define _ETAT_ADV_ARRIERE_C         ETAT_HAUT
    #define _ETAT_ADV_ARRIERE_D         ETAT_HAUT

    #define INCLINOMETRE                CAPTEUR5

    #define _CAPTEUR1_ANALOGIQUE        DIGITAL
    #define _CAPTEUR2_ANALOGIQUE        DIGITAL
    #define _CAPTEUR3_ANALOGIQUE        DIGITAL


    /**************************************************************************/
    /**************************************************************************/
    /**************************************************************************/



#endif


#ifdef PETIT_ROBOT

    //#define CARTE_V1
    #define CARTE_V2

    /**************************************************************************/
    /****************************** ODOMETRIE *********************************/
    /**************************************************************************/
    #define _ENTRAXE_MM                173.5 //174.7// 173.25// 173.75 vincent //170.75 //Pas assez tourné -> augmenter entraxe
    #define _DIAMETRE_ROUE_CODEUSE       51.9715   //51.9715     //Toujours dans le sens de l'erreur : aller plus loin = diminuer D
    #define _PERIMETRE_ROUE_MM           196.0 //195.4 //194.91//195.1 Si distance parcourue trop grande -> Augmenter la taille des roues
  

    #define  _COEF_D                    (double) 1.0055//1.0017 VIN //  //1.00372     1.004 gauche x=1.003  1.0025droite //augmente = + devie a gauche
    //+COEF_D -> + a droite
    #define  _COEF_G                    (double) 1.000   //augmente = + devie a droite
    //+COEF_G -> + a gauche
    /**************************************************************************/
    /******************************** ASSERV **********************************/
    /**************************************************************************/
        //Association distance - Vitesse - Accélérations max
    #define _VITESSE_CONSIGNE_MAX_MM     2.//1.// VINCENT//2. //1. //2. //2.8 

    #define _VITESSE_DISTANCE_MIN        0.75
    #define _VITESSE_MAX_MM_TENSION      4.//2.2 //4.
    #define _DISTANCE_CONSIGNE_MM        500.

    #define _ACC_POSITION_CONSIGNE      2.5//2.5 //VIN //0.5  
    #define _DCC_POSITION_CONSIGNE      1.5//2.5 //VIN//0.5 //2.8//0.5  LA
    #define _ACC_POSITION_MIN           1. //1. VIN //1. // 
    #define _DCC_POSITION_MIN           0.25// 1. VIN//0.25 //0.25 //1. //0.25 LA

    #define _COEF_FREINAGE                1.//1.

        //Association Angle - Vitesse - Accélérations max
    #define _VITESSE_ANGLE_MAX           0.02//0.1// VIN//0.015 
    #define _VITESSE_ANGLE_MIN           0.005
    #define _ORIENTATION_CONSIGNE_DEG    90.

    #define _ACC_ORIENTATION_CONSIGNE    2. 
    #define _DCC_ORIENTATION_CONSIGNE    2.
    #define _ACC_ORIENTATION_MIN         0.5
    #define _DCC_ORIENTATION_MIN         0.5

        //Autres réglages asserv
    #define _SEUIL_IMMOBILITE               75ULL//75ULL //200
    #define _MAX_ERREUR_INTEGRALLE_V    30000.//100000.//10000. //3500.
    #define _MAX_E_INTEGRALLE_BRAKE      10000. //12000

    /**************************************************************************/
    /********************************* PID ************************************/ // PID === Present Passé Futur
    /**************************************************************************/

    //PID
    #define _VITESSE_DIS_KP            0.25//0.5//0.25  VINCENT     //0.8 //1. //0.8 //1.5 //0.8 
    #define _VITESSE_DIS_KI           0.0005// 0.005//0.0005 vin //0.08//0.082 //0.13 //0.3 //0.1 //0.05 //0.02 //0.045
    #define _VITESSE_DIS_KD             0.2//0.2// vin //0.8//0.9 //1.0 //0.8 //0.5 //0.2
    
    #define _POSITION_KP                 1.
    #define _POSITION_KI                 0.//0
    #define _POSITION_KD                 0.

    #define _ORIENTATION_KP              1.// 1.
    #define _ORIENTATION_KI              0.//0.
    #define _ORIENTATION_KD              0.

    #define KP_BRAKE                    0.04//0.04 VIN//0.03//0.09//0.058//0.04 // fait avec vincent le 16 AVRIL 2023
    #define KI_BRAKE                    0.0005//0.0005 VIN//0.0015
    #define KD_BRAKE                    0.6//0.6 VIN//0.2 //0.00333 //1.2 //0.8


    /**************************************************************************/
    /******************************* TENSIONS *********************************/
    /**************************************************************************/

    //Tensions d'alimentations
    #define _TENSION_MOTEUR_DROIT        12L
    #define _TENSION_MOTEUR_GAUCHE       12L

    #define _TENSION_MOTEUR_X            12L

    #define _TENSION_SORTIE_PTN          17L

    #define _CKECK_LIMITATION_COURANT    true 


    /**************************************************************************/
    /*************************** CONFIGS MOTEURS ******************************/
    /**************************************************************************/


    //Paramètres codeurs, et moteurs
    //Sens rotation codeur
    #define _SENS_ROT_D                  -1L
    #define _SENS_ROT_G                  1L

    #define _RAPPORT_REDUCTION           1U

    #define _CODEUR_D_NB_IMP             3600U
    #define _CODEUR_G_NB_IMP             3600U

    #define _RESOLUTION_LOGICIELLE       1LL

    //PWM : sens de rotation du moteur
    #define _AVANCER_MOTEUR_D            0
    #define _RECULER_MOTEUR_D            1

    #define _AVANCER_MOTEUR_G            0
    #define _RECULER_MOTEUR_G            1

    #define _AVANCER_MOTEUR_X            0
    #define _RECULER_MOTEUR_X            1

    #define _AVANCER_MOTEUR_Y            0
    #define _RECULER_MOTEUR_Y            1
    

    /**************************************************************************/
    /***************************** PORTS CARTE ********************************/
    /**************************************************************************/


    //La définition des ports relatif à chaque carte se trouve
    //Dans le fichier system.h

    #define CAPT_IR_ARRIERE_DROIT      AUCUN //CAPTEUR4
    #define CAPT_IR_ARRIERE_GAUCHE     AUCUN //CAPTEUR7
    #define CAPT_IR_AVANT_DROIT        AUCUN //CAPTEUR1
    #define CAPT_IR_AVANT_GAUCHE       AUCUN //CAPTEUR5


    // Définition des standart pour la fonction évitement
    // ETAT : etat de detection du capteur (si il détecte à un : ETAT_HAUT)
    // NB : Si un capteur n'existe pas, le mettre à AUCUN et à ETAT_HAUT
    #define _CAPT_ADV_AVANT_G            CAPTEUR2
    #define _CAPT_ADV_AVANT_D            CAPTEUR1
    #define _CAPT_ADV_AVANT_C            AUCUN

    #define _ETAT_ADV_AVANT_G            ETAT_HAUT
    #define _ETAT_ADV_AVANT_D            ETAT_HAUT
    #define _ETAT_ADV_AVANT_C            ETAT_HAUT

    #define _CAPT_ADV_ARRIERE_G          CAPTEUR5
    #define _CAPT_ADV_ARRIERE_C          AUCUN
    #define _CAPT_ADV_ARRIERE_D          CAPTEUR6

    #define _ETAT_ADV_ARRIERE_G          ETAT_HAUT
    #define _ETAT_ADV_ARRIERE_C          ETAT_HAUT
    #define _ETAT_ADV_ARRIERE_D          ETAT_BAS //ETAT_BAS

    #define _CAPTEUR1_ANALOGIQUE        DIGITAL
    #define _CAPTEUR2_ANALOGIQUE        DIGITAL
    #define _CAPTEUR3_ANALOGIQUE        DIGITAL


    /**************************************************************************/
    /**************************************************************************/
    /**************************************************************************/
#endif

    

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_ROBOTS_H */

