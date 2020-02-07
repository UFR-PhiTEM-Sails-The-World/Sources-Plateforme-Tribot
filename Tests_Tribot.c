#include <stm32f4xx_hal.h>
#include "init_general.h"
#include "initialisations.h"
#include "controleMoteurs.h"
#include "DeplacementRobot.h"
#include "Tests_Tribot.h"
#include <stdio.h>


void tests_Tribot(char Test_voulu)
{	
	//A voir pour le placement de l'init de ces variables...
	/*--------- Variables Locales à modifier pour définir les paramètres des tests effectués ---------*/
	short t_av_test_ms=4000, duree_test_ms=4000;
	short t_ass_mot_Xus=1000, t_ass_traj_X100us=1000;
	short v_mot=1000, v_trans=200, v_angul=200;
	double theta_test= M_PI/2;
	short cibl_x_test=500, cibl_y_test=0;
	unsigned char direction = Y_pos;
	signed char sens = 1;
	/*----- Fin des variables à modifier pour définir les paramètres des tests effectués -----*/
	
	tempo_TIM5_x_1ms(t_av_test_ms); //avant de faire un test, on laisse un temps pour que tout le monde soit en place 
	
	switch (Test_voulu){
		case Test_mot_BO:
				test_mot_BO(duree_test_ms, v_mot);
				break;
		case Test_mot_BF:
				test_mot_BF(t_ass_mot_Xus, duree_test_ms, v_mot);
				break;
		case Test_cmd_dir:
				test_cmd_dir(t_ass_mot_Xus, theta_test, v_trans, v_angul);
				break;
		case Test_traj:
				test_traj(t_ass_mot_Xus, t_ass_traj_X100us, theta_test, cibl_x_test, cibl_y_test);
				break;
		case Deter_dx:
				deter_dx(t_ass_mot_Xus, direction, duree_test_ms, v_mot);
				break;
		case Deter_dTheta:
				deter_dTheta(t_ass_mot_Xus, sens, duree_test_ms, v_mot);
				break;
		default: break;
	}	
}	
	

// Test moteurs en boucle ouverte et retour codeurs
void test_mot_BO(short duree_test_ms, short v_mot)
{		
		short i;
	
		setMotorSpeedBO(MOTOR1,v_mot); // Arret du moteur
    setMotorSpeedBO(MOTOR2,v_mot); // Arret du moteur
    setMotorSpeedBO(MOTOR3,v_mot); // Arret du moteur
    
		for(i=0;i<40;i++){
			printf("TIM2= %d\n\r", mesureVitesseMoteur1);
			printf("TIM3= %d\n\r", mesureVitesseMoteur2);
			printf("TIM4= %d\n\r", mesureVitesseMoteur3);
			tempo_TIM5_x_1ms(duree_test_ms/40);
    }
}


// Test moteurs asservis
void test_mot_BF(short t_ass_mot_Xus, short duree_test_ms, short v_mot)
{	
		init_Timer9_IT_Xus(t_ass_mot_Xus); // Période de X us pour l'asservissement
    //motorsEnable();
		
    setMotorSpeedBF(MOTOR1,v_mot);
		setMotorSpeedBF(MOTOR2,v_mot);
    setMotorSpeedBF(MOTOR3,v_mot);
		tempo_TIM5_x_1ms(duree_test_ms/2);
    setMotorSpeedBF(MOTOR1,-v_mot);
		setMotorSpeedBF(MOTOR2,-v_mot);
    setMotorSpeedBF(MOTOR3,-v_mot);
		tempo_TIM5_x_1ms(duree_test_ms/2);
		
		TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
    setMotorSpeedBO(MOTOR1,0); // Arret du moteur
    setMotorSpeedBO(MOTOR2,0); // Arret du moteur
    setMotorSpeedBO(MOTOR3,0); // Arret du moteur
}


// Test de commande de direction du robot
void test_cmd_dir(short t_ass_mot_Xus, double theta_test, short v_trans, short v_angul)
{
		short i;
	
		init_Timer9_IT_Xus(t_ass_mot_Xus); // Période de 5 ms pour l'asservissement
		translationRobot(theta_test, v_trans);
		//translationPlusRotation( M_PI/2, 60, 60); //double angle, short vitesseTranslation, short vitesseAngulaire
		//tempo_TIM5_x_1ms(4000);
		for(i=0;i<40;i++){
			printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);
			tempo_TIM5_x_1ms(200);
    }
		
		TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
    setMotorSpeedBO(MOTOR1,0); // Arret du moteur
    setMotorSpeedBO(MOTOR2,0); // Arret du moteur
    setMotorSpeedBO(MOTOR3,0); // Arret du moteur
}	


//Test de trajectoire 
void test_traj(short t_ass_mot_Xus,short t_ass_traj_X100us, double Theta_test, short cibl_x_test, short cibl_y_test)
{
	unsigned char i;
	init_Timer9_IT_Xus(t_ass_mot_Xus); // Période de 5 ms pour l'asservissement
	init_Timer10_IT_X100us(t_ass_traj_X100us); //pour la trajectoire	
    
	//allerAuPoint(cibl_x_test, cibl_y_test, theta_test);
		allerAuPoint(0, 500, M_PI);
    allerAuPoint(500, 500, -M_PI/2);
    allerAuPoint(500, 0, 0);
    allerAuPoint(0, 0, 0);
		
    for(i=0;i<12;i++){
			printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);
			tempo_TIM5_x_1ms(1000);
    }
}


//Pour l'instant on fait seulement avancer le robot en avant
void deter_dx(short t_ass_mot_Xus, char direction, short duree_test_ms, short v_mot){
	signed char coef_m1 = 0, coef_m2 = -1, coef_m3 = 1;
	init_Timer9_IT_Xus(t_ass_mot_Xus); // Période de 1 ms pour l'asservissement des moteurs
	
	//Gestion suivant les differentes directions
	switch (direction){
		case Y_pos:
				coef_m1 = 0;
				coef_m2 = -1;
				coef_m3 = 1;
				break;
		case X_pos_Y_neg:
				coef_m1 = -1;
				coef_m2 = 1;
				coef_m3 = 0;
				break;
		case XY_neg:
				coef_m1 = 1;
				coef_m2 = 0;
				coef_m3 = -1;
				break;
		case Y_neg:
				coef_m1 = 0;
				coef_m2 = 1;
				coef_m3 = -1;
				break;
		case X_neg_Y_pos:
				coef_m1 = 1;
				coef_m2 = -1;
				coef_m3 = 0;
				break;
		case XY_pos:
				coef_m1 = -1;
				coef_m2 = 0;
				coef_m3 = 1;
				break;
		default:
				coef_m1 = 0;
				coef_m2 = 0;
				coef_m3 = 0;
				break;
	}
		setMotorSpeedBF(MOTOR1, coef_m1 * v_mot); 	// 1000 -> on se met bas dans la zone supposée linéaire des moteurs (4500 étant la consigne max)
		setMotorSpeedBF(MOTOR2, coef_m2 * v_mot); 	// 0 ->  Arret du moteur
		setMotorSpeedBF(MOTOR3, coef_m3 * v_mot);		//Là on veut aller tout droit sur y pour trouver le dx	
	
	tempo_TIM5_x_1ms(duree_test_ms); //On va Xmsec dans la même direction
	
	// Arret du robot après la fin du test
	TIM9->CR1 &=~1;     // Arret du Timer9 qui génère l'IT périodique pour l'asservissement de vitesse des moteurs
	setMotorSpeedBO(MOTOR1,0);
	setMotorSpeedBO(MOTOR2,0);
	setMotorSpeedBO(MOTOR3,0);
	
	// Affichage des résultats du test
	tempo_TIM5_x_1ms(8000); //On attend 10sec pour envoyer les résultats
	printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);
}


void deter_dTheta(short t_ass_mot_Xus, signed char sens, short duree_test_ms, short v_mot){
	
	init_Timer9_IT_Xus(t_ass_mot_Xus); // Période de 1 ms pour l'asservissement des moteurs
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
	tempo_TIM5_x_1ms(8000); //On attend 10sec pour envoyer les résultats
	printf("%lf , %lf , %lf °\n\r", X_a, Y_a, THETA_a*180/M_PI);
	
}

