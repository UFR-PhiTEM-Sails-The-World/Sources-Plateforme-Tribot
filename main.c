#include <stm32f4xx_hal.h>
#include "init_general.h"
#include "initialisations.h"
#include "controleMoteurs.h"
#include "DeplacementRobot.h"

//#include "I2C.h"
#include <stdio.h>

// Pour les fonctions de tests
#include "Tests_Tribot.h"


//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------
//Serial pc(SERIAL_TX, SERIAL_RX);

//DigitalOut myled(LED1);
//InterruptIn button(USER_BUTTON);
//------------------------------------
// Variables Globales
//------------------------------------
short consigneVitesseMoteurBO1=0,consigneVitesseMoteurBO2=0,consigneVitesseMoteurBO3=0,consigneVitesseMoteurBO4=0; // Valeur de consigne de vitesse asservie Valeur max 600 environ asservissement instable en dessous de 25 environ.
short mesureVitesseMoteur1=0,mesureVitesseMoteur2=0,mesureVitesseMoteur3=0,mesureVitesseMoteur4=0;
//short commandeVitesseMoteur1=0,commandeVitesseMoteur2=0,commandeVitesseMoteur3=0;
int DATA[2000];
double Data_d[3000];
short indexDATA=0;
int ARRET = 0;

// Coordonnées absolues d'odométrie
double X_a = 0, Y_a = 0;
double THETA_a = 0;


void dETER_dx(char direction, short duree_test_ms, short v_mot);
void dETER_dTheta(signed char sens, short duree_test_ms, short v_mot);

int main()
{
    unsigned char result = 255;
    unsigned int dist;
    //int i = 1;
		init_general();
    printf("Hello World !\n\r");
    init_TIM1_PWMx4();
    init_TIM8_PWMx4_Servo();
    printf("Timer1 initialized !!!!!\n\r");
    //initTimer12PWM();
    printf("Timer12 initialized !!!!!\n\r");
    init_ES();
    //init_encoders_Nb_Impulsions();
    //init_Timer9_IT_Xus(10000); // Période de 1 ms pour l'asservissement
    //init_Timer10_IT_X100us(1000); //pour la trajectoire

    init_encoders_Periode();

	tests_Tribot(Deter_dx);
	
	
//	/*------------ Tests pour re determiner la valeur des distances infinidessimales dx et dTheta ------------*/
//	short t_av_test_ms = 5000, duree_test_ms=4000, v_mot=1000;
//	
//	tempo_TIM5_x_1ms(t_av_test_ms);
//	deter_dx(1, duree_test_ms, v_mot);
//	deter_dTheta(sens1, duree_test_ms, v_mot);
//	
//	
//	/*-------- Fin des tests pour re determiner la valeur des distances infinidessimales dx et dTheta --------*/
	
	// test moteurs en boucle ouverte et retour codeurs
/*
		setMotorSpeedBO(MOTOR1,0); // Arret du moteur
    setMotorSpeedBO(MOTOR2,0); // Arret du moteur
    setMotorSpeedBO(MOTOR3,0); // Arret du moteur
    while(1){
			
			printf("TIM2= %d\n\r", mesureVitesseMoteur1);
			printf("TIM3= %d\n\r", mesureVitesseMoteur2);
			printf("TIM4= %d\n\r", mesureVitesseMoteur3);
			tempo_TIM5_x_1ms(1000);
			
    }
*/

// Test moteurs asservis
/*   
		init_Timer9_IT_Xus(5000); // Période de 5 ms pour l'asservissement
    //motorsEnable();
		
    setMotorSpeedBF(MOTOR1,200);
		setMotorSpeedBF(MOTOR2,200);
    setMotorSpeedBF(MOTOR3,200);
		tempo_TIM5_x_1ms(4000);
    setMotorSpeedBF(MOTOR1,-200);
		setMotorSpeedBF(MOTOR2,-200);
    setMotorSpeedBF(MOTOR3,-200);
		tempo_TIM5_x_1ms(4000);
				
		TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
    setMotorSpeedBO(MOTOR1,0); // Arret du moteur
    setMotorSpeedBO(MOTOR2,0); // Arret du moteur
    setMotorSpeedBO(MOTOR3,0); // Arret du moteur
*/	

// Test de commande de direction du robot
/*		init_Timer9_IT_Xus(5000); // Période de 5 ms pour l'asservissement
		translationRobot(M_PI/2, 200);
		//translationPlusRotation( M_PI/2, 60, 60); //double angle, short vitesseTranslation, short vitesseAngulaire
		//tempo_TIM5_x_1ms(4000);
		while(1){
			printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);
			tempo_TIM5_x_1ms(200);
    }
			
		TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
    setMotorSpeedBO(MOTOR1,0); // Arret du moteur
    setMotorSpeedBO(MOTOR2,0); // Arret du moteur
    setMotorSpeedBO(MOTOR3,0); // Arret du moteur
*/		


/*
//Test de trajectoire 
	init_Timer9_IT_Xus(5000); // Période de 5 ms pour l'asservissement
	init_Timer10_IT_X100us(1000); //pour la trajectoire	
    
	//allerAuPoint(0, 400, 0);
		allerAuPoint(0, 500, M_PI);
    allerAuPoint(500, 500, -M_PI/2);
    allerAuPoint(500, 0, 0);
    allerAuPoint(0, 0, 0);
		
    while(1){


    }
*/				


}


//Pour l'instant on fait seulement avancer le robot en avant
void dETER_dx(char direction, short duree_test_ms, short v_mot){
	
	init_Timer9_IT_Xus(1000); // Période de 1 ms pour l'asservissement des moteurs
		setMotorSpeedBF(MOTOR1,0); 	// 1000 -> on se met bas dans la zone supposée linéaire des moteurs (4500 étant la consigne max)
		setMotorSpeedBF(MOTOR2,-v_mot); 	// 0 ->  Arret du moteur
		setMotorSpeedBF(MOTOR3,v_mot);			//Là on veut aller tout droit sur y pour trouver le dx		
	tempo_TIM5_x_1ms(duree_test_ms); //On va 4sec dans la même direction
	
	// Arret du robot après la fin du test
	TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
	setMotorSpeedBO(MOTOR1,0);
	setMotorSpeedBO(MOTOR2,0);
	setMotorSpeedBO(MOTOR3,0);
	
	// Affichage des résultats du test
	tempo_TIM5_x_1ms(10000); //On attend 10sec pour envoyer les résultats
	printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);
}

void dETER_dTheta(signed char sens, short duree_test_ms, short v_mot){
	
	init_Timer9_IT_Xus(1000); // Période de 1 ms pour l'asservissement des moteurs
		setMotorSpeedBF(MOTOR1,sens*v_mot);
		setMotorSpeedBF(MOTOR2,sens*v_mot);
		setMotorSpeedBF(MOTOR3,sens*v_mot);
	tempo_TIM5_x_1ms(duree_test_ms); //On va 4sec dans la même direction
	
	// Arret du robot après la fin du test
	TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
	setMotorSpeedBO(MOTOR1,0);
	setMotorSpeedBO(MOTOR2,0);
	setMotorSpeedBO(MOTOR3,0);
	
	// Affichage des résultats du test
	tempo_TIM5_x_1ms(10000); //On attend 10sec pour envoyer les résultats
	printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);
}

