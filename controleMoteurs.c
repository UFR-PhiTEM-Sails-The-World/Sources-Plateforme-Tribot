#include <stm32f4xx_hal.h>
#include "controleMoteurs.h"
#include <math.h>

// Routine d'interruption périodique (5ms) pour gérer l'asservissement en vitesse des 3 moteurs
//void IT_asservissement(void)
void TIM1_BRK_TIM9_IRQHandler(void)
{
    static long int erreurIntegrale1=0,erreurIntegrale2=0,erreurIntegrale3=0;
    long int erreur1,erreur2,erreur3;
    int mesV1,mesV2,mesV3;
    short commandeVitesseMoteur=0;
    //float KItst;
    double tmp;
    
    if((TIM9->SR & 1) !=0){                                                         // Si le drapeau d'interruption du Timer9 est levé
    
        // ********************** //
        // Asservissement moteur1 //
        // ********************** //
        if (mesureVitesseMoteur1!=0)
            mesV1 = 100000/mesureVitesseMoteur1;                                    // Récupération de la mesure
        else
            mesV1 = 0;
        if( (consigneVitesseMoteurBO1>-25) && (consigneVitesseMoteurBO1 <25))      // Saturation basse à 0 pour éviter l'instabilité dans les faibles vitesses
            setMotorSpeedBO(MOTOR1,0);                                              // On impose une vitesse nulle dans ce cas
        else
        {      
             
            erreur1 = consigneVitesseMoteurBO1 - mesV1;                             // Calcul de l'erreur à l'instant t
            if((erreurIntegrale1+erreur1)<20000 && (erreurIntegrale1+erreur1)>-20000)
                erreurIntegrale1 += erreur1;                                        // Mise à jour de l'erreur intégrale avec saturation
            //KItst = 0.0024*abs(consigneVitesseMoteurBO1)+0.18;                    // Test d'un coefficient Ki variable pour que le temps de réponse soit le même pour les faibles consignes et les plus élevées
            tmp = (KP*erreur1+KI*erreurIntegrale1);
            if (tmp>4500.0)                                                         // Saturation ici pour éviter le débordement dans le passa de l'argument à la fonction setMotorSpeedBO
                commandeVitesseMoteur = 4500;
            else if (tmp<-4500.0)
                commandeVitesseMoteur = -4500;
            else
                commandeVitesseMoteur = (short)tmp;
            setMotorSpeedBO(MOTOR1,commandeVitesseMoteur);                         // Envoi de la commande de vitesse
         }  
         
        // ********************** //
        // Asservissement moteur2 //
        // ********************** //
        
        if (mesureVitesseMoteur2!=0)
            mesV2 = 100000/mesureVitesseMoteur2;                                    // Récupération de la mesure
        else
            mesV2 = 0;
        if( (consigneVitesseMoteurBO2>-25) && (consigneVitesseMoteurBO2 <25))      // Saturation basse à 0 pour éviter l'instabilité dans les faibles vitesses
            setMotorSpeedBO(MOTOR2,0);                                              // On impose une vitesse nulle dans ce cas
        else
        {      
             
            erreur2 = consigneVitesseMoteurBO2 - mesV2;                             // Calcul de l'erreur à l'instant t
            if((erreurIntegrale2+erreur2)<20000 && (erreurIntegrale2+erreur2)>-20000)
                erreurIntegrale2 += erreur2;                                        // Mise à jour de l'erreur intégrale avec saturation
            //KItst = 0.0024*abs(consigneVitesseMoteurBO2)+0.18;                    // Test d'un coefficient Ki variable pour que le temps de réponse soit le même pour les faibles consignes et les plus élevées
            tmp = (KP*erreur2+KI*erreurIntegrale2);
            if (tmp>4500.0)                                                         // Saturation ici pour éviter le débordement dans le passa de l'argument à la fonction setMotorSpeedBO
                commandeVitesseMoteur = 4500;
            else if (tmp<-4500.0)
                commandeVitesseMoteur = -4500;
            else
                commandeVitesseMoteur = (short)tmp;
            setMotorSpeedBO(MOTOR2,commandeVitesseMoteur);                         // Envoi de la commande de vitesse
         }  
          
        // ********************** //
        // Asservissement moteur3 //
        // ********************** //
        if (mesureVitesseMoteur3!=0)
                mesV3 = 100000/mesureVitesseMoteur3;                                // Récupération de la mesure
        else
                mesV3 = 0; 
        if( (consigneVitesseMoteurBO3>-25) && (consigneVitesseMoteurBO3 <25))      // Saturation basse à 0 pour éviter l'instabilité dans les faibles vitesses
            setMotorSpeedBO(MOTOR3,0);                                              // On impose une vitesse nulle dans ce cas
        else
        {          
            erreur3 = consigneVitesseMoteurBO3 - mesV3;                             // Calcul de l'erreur à l'instant t
            if((erreurIntegrale3+erreur3)<20000 && (erreurIntegrale3+erreur3)>-20000)
                erreurIntegrale3 += erreur3;                                        // Mise à jour de l'erreur intégrale avec saturation
            //KItst = 0.0024*abs(consigneVitesseMoteurBO3)+0.18;                    // Test d'un coefficient Ki variable pour que le temps de réponse soit le même pour les faibles consignes et les plus élevées
            tmp = (KP*erreur3+KI*erreurIntegrale3);
            if (tmp>4500.0)                                                         // Saturation ici pour éviter le débordement dans le passa de l'argument à la fonction setMotorSpeedBO
                commandeVitesseMoteur = 4500;
            else if (tmp<-4500.0)
                commandeVitesseMoteur = -4500;
            else
                commandeVitesseMoteur = (short)tmp;
            setMotorSpeedBO(MOTOR3,commandeVitesseMoteur);                         // Envoi de la commande de vitesse
         }   
         
        // Sauvegarde de valeurs dans un table pour restituer un log à la fin de l'application (et ne pas ralentir les fonction d'IT par des printf
       /* if(indexDATA<200){
            //DATA[2*indexDATA] = mesureVitesseMoteur3;//mesV3;
            //DATA[indexDATA] = mesV1;
            Data_d[3*indexDATA] = X_a;
            Data_d[3*indexDATA+1] = Y_a;
            Data_d[3*indexDATA+2] = THETA_a;          
            //Data_d[3*indexDATA] = KP*erreur1;
            //Data_d[3*indexDATA+1] = KI*erreurIntegrale1;
            //Data_d[3*indexDATA+2] = (double)commandeVitesseMoteur1;
            
            //DATA[4*indexDATA] = mesV1;//mesV1;
            //DATA[4*indexDATA+1] = erreur1;
            //DATA[4*indexDATA+2] = commandeVitesseMoteur1;
            //DATA[4*indexDATA+3] = TIM1->CCR1;
            
            //DATA[3*indexDATA] = mesV1;
            //DATA[3*indexDATA+1] = mesV2;
            //DATA[3*indexDATA+2] = mesV3;
            indexDATA++;
        }*/
        TIM9->SR&=~1;                   // Mise à zÈro du drapeau d'interruption    
    }
}

/*









Juste pour dire que je modifie actuellement juste la routine d'IT pour le timer 2













*/
// Routine d'interuption du Timer 2 permettant de gérer la mesure de vitesse pour le moteur 1 et la mise à jour de l'odométrie
//void IT_codeurTim2(void){
void TIM2_IRQHandler(void){
    static unsigned short n_overflow1=0;		//Pour compter le nombre de fois qu'on a eu un overflow entre 2 IT d'imput capture
		static signed char sens_mot1;			//Pour conserver l'information du sens de rotation du mot1
		
		if((TIM2->SR & 1) !=0){                     // Déclenchement de l'IT à cause de "Update event" (le CPT à débordé -> Rotation trop lente)
        n_overflow1 ++;
				TIM2->SR &=~1;                          // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }else if ( (TIM2->SR & (1<<1)) !=0){        // Déclenchement de l'IT à cause de "Input capture" un front est apparu sur le codeur, une nouvelle valeur de période est dispo
        //pc.printf("PeriodeTIM2= %d\n\r", TIM2->CNT);		
				if(((GPIOA->IDR&(1<<15))>>15) != ((GPIOB->IDR&(1<<3)))>>3)
					sens_mot1 = sens1;
				else sens_mot1 = sens2;
				/*					/!\					 */
				/*	Risque de débordement de mesureVitesseMoteur1 avec ce calcul */
				mesureVitesseMoteur1 = sens_mot1* (TIM2->CNT + n_overflow1 * (TIM2->ARR));
				/*	Check si on ne peut pas baisser la valeur de ARR si possible */
				/*	Sinon limiter la valeur max de n_overflow										 */
				
				// Mise à jour de l'odométrie
				X_a -= dx*sin(THETA_a+alpha1)* sens_mot1;
        Y_a += dx*cos(THETA_a+alpha1)* sens_mot1;
        THETA_a += dTheta* sens_mot1;
        
				if( THETA_a>M_PI) THETA_a -= (2*M_PI);  // La valeur de THETA_a doit rester dans ]-Pi;Pi]
        else if( THETA_a<=-M_PI) THETA_a += (2*M_PI);
        
				n_overflow1 = 0;	//On a eu une mesure, mais pas d'overflow, donc la série d'overflow repart à 0
        TIM2->CNT = 0;                          // Réinitialisation du compteur pour la prochaine capture
        TIM2->SR &= ~(1<<1);                    // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }
		
}

/*











Voila c'est la fin des modifs que j'apporte



















*/





// Routine d'interuption du Timer 3 permettant de gérer la mesure de vitesse pour le moteur 2 et la mise à jour de l'odométrie
//void IT_codeurTim3(void){
void TIM3_IRQHandler(void){
    static int mauvaiseMesure2=0;              // Mémorisation de l'info de mauvaise mesure (débordement du timer)
    if((TIM3->SR & 1) !=0){                     // Déclenchement de l'IT à cause de "Update event" (le CPT à débordé -> mauvaise mesure)
        mauvaiseMesure2=1;                      // On mémorise que la prochaine mesure doit être jetée
        mesureVitesseMoteur2 = 0;               // Au dela d'une période de 10ms, on considère que la vitesse est nulle
        TIM3->SR &=~1;                          // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }else if ( (TIM3->SR & (1<<1)) !=0){        // Déclenchement de l'IT à cause de "Input capture" un front est apparu sur le codeur, une nouvelle valeur de période est dispo
        if( ((GPIOB->IDR&(1<<4))>>4)!=((GPIOB->IDR&(1<<5)))>>5 ){ // test du sens de rotation : si PB4==PB5 ->sens1, sinon sens2
            if (mauvaiseMesure2==1)             // Si la précédente valeur était mauvaise...   
                mauvaiseMesure2=0;              // On réinitialise la variable
            else mesureVitesseMoteur2 = TIM3->CNT;  // La mesure est bonne
            // Mise à jour de l'odométrie, que la mesure soit bonn ou non
            X_a -= dx*sin(THETA_a+alpha2);
            Y_a += dx*cos(THETA_a+alpha2);
            THETA_a +=dTheta;
        }
        else{
            if (mauvaiseMesure2==1)              // Si la précédente valeur était mauvaise...   
                mauvaiseMesure2=0;               // On réinitialise la variable
            else mesureVitesseMoteur2 = -TIM3->CNT; // La mesure est bonne
            // Mise à jour de l'odométrie, que la mesure soit bonn ou non
            X_a += dx*sin(THETA_a+alpha2);
            Y_a -= dx*cos(THETA_a+alpha2);
            THETA_a -=dTheta;   
        }
        if( THETA_a>M_PI) THETA_a -= (2*M_PI);   // La valeur de THETA_a doit rester dans ]-Pi;Pi]
        else if( THETA_a<=-M_PI) THETA_a += (2*M_PI);
                        
        TIM3->CNT = 0;                          // Réinitialisation du compteur pour la prochaine capture
        TIM3->SR &= ~(1<<1);                    // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }
}

// Routine d'interuption du Timer 4 permettant de gérer la mesure de vitesse pour le moteur 3 et la mise à jour de l'odométrie
//void IT_codeurTim4(void){
void TIM4_IRQHandler(void){
    static int mauvaiseMesure3=0;              // Mémorisation de l'info de mauvaise mesure (débordement du timer)
    if((TIM4->SR & 1) !=0){                     // Déclenchement de l'IT à cause de "Update event" (le CPT à débordé -> mauvaise mesure)
        mauvaiseMesure3=1;                      // On mémorise que la prochaine mesure doit être jetée
        mesureVitesseMoteur3 = 0;               // Au dela d'une période de 10ms, on considère que la vitesse est nulle
        TIM4->SR &=~1;                          // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }else if ( (TIM4->SR & (1<<1)) !=0){        // Déclenchement de l'IT à cause de "Input capture" un front est apparu sur le codeur, une nouvelle valeur de période est dispo
        if( ((GPIOB->IDR&(1<<7))>>7)!=((GPIOB->IDR&(1<<6)))>>6 ){ // test du sens de rotation : si PB6==PB7 ->sens1, sinon sens2
            if (mauvaiseMesure3==1)             // Si la précédente valeur était mauvaise...   
                mauvaiseMesure3=0;              // On réinitialise la variable
             else mesureVitesseMoteur3 = -TIM4->CNT;// La mesure est bonne
        // Mise à jour de l'odométrie, que la mesure soit bonn ou non
            X_a += dx*sin(THETA_a+alpha3);
            Y_a -= dx*cos(THETA_a+alpha3);
            THETA_a -=dTheta; 
        }
        else{
            if (mauvaiseMesure3==1)             // Si la précédente valeur était mauvaise...   
                mauvaiseMesure3=0;              // On réinitialise la variable
             else mesureVitesseMoteur3 = TIM4->CNT;// La mesure est bonne
        // Mise à jour de l'odométrie, que la mesure soit bonn ou non
            X_a -= dx*sin(THETA_a+alpha3);
            Y_a += dx*cos(THETA_a+alpha3);
            THETA_a +=dTheta;  
        }
        if( THETA_a>M_PI) THETA_a -= (2*M_PI);   // La valeur de THETA_a doit rester dans ]-Pi;Pi]
        else if( THETA_a<=-M_PI) THETA_a += (2*M_PI);
        
        TIM4->CNT = 0;                          // Réinitialisation du compteur pour la prochaine capture
        TIM4->SR &= ~(1<<1);                    // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }
}
/*
// Routine d'interuption du Timer 5 permettant de gérer la mesure de vitesse pour le moteur 4 
void IT_codeurTim5(void){
    static int mauvaiseMesure4=0;              // Mémorisation de l'info de mauvaise mesure (débordement du timer)
    if((TIM5->SR & 1) !=0){                     // Déclenchement de l'IT à cause de "Update event" (le CPT à débordé -> mauvaise mesure)
        mauvaiseMesure4=1;                      // On mémorise que la prochaine mesure doit être jetée
        mesureVitesseMoteur4 = 0;               // Au dela d'une période de 10ms, on considère que la vitesse est nulle
        TIM5->SR &=~1;                          // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }else if ( (TIM5->SR & (1<<1)) !=0){        // Déclenchement de l'IT à cause de "Input capture" un front est apparu sur le codeur, une nouvelle valeur de période est dispo
        if( ((GPIOA->IDR&(1<<0))>>0)!=((GPIOA->IDR&(1<<1)))>>1 ){ // test du sens de rotation : si PA0==PA1 ->sens1, sinon sens2
            if (mauvaiseMesure4==1)             // Si la précédente valeur était mauvaise...   
                mauvaiseMesure4=0; 
            else mesureVitesseMoteur4 = TIM5->CNT; // La mesure est bonne
        }
        else{
            if (mauvaiseMesure4==1)             // Si la précédente valeur était mauvaise...   
                mauvaiseMesure4=0; 
            else mesureVitesseMoteur4 = -TIM5->CNT;// La mesure est bonne
        }        
        TIM5->CNT = 0;                          // Réinitialisation du compteur pour la prochaine capture
        TIM5->SR &= ~(1<<1);                    // Mise à zéro du drapeau d'interruption (Validation de l'IT)
    }
}
*/

void motorsBreak(void){
    TIM1->BDTR |= ((1<<15));// Break : désactivation de toutes les sorties OCx   
}
void motorsEnable(void){
    TIM1->BDTR &= ~((1<<15));// Break disable : activation de toutes les sorties OCx   
}

// Impose une valeur de vitesse en boucle ouverte pour un moteur donné (directement dans le registre de comparaison) Speed dans [-4500;4500]
void setMotorSpeedBO(unsigned char motorNum, short speed){
    unsigned short absSpeed, sens=0;
    // Récupération de la valeur absolue et du sens
    if (speed<0){ 
        absSpeed=-speed;
        sens = 0;
    } else {
        absSpeed=speed;
        sens = 1;
    }
    if (absSpeed>4500) absSpeed = 4500;     // Saturation au max en cas de commande trop élevée
    switch(motorNum){
        case MOTOR1 :
            TIM1->CCR3 = absSpeed;          // rapport cyclique du PWM pour le moteur 1
            GPIOA->BSRR = (1<<(7+16*sens)); // Mise à "sens" du signal de direction
            break;
        case MOTOR2 : 
            TIM1->CCR1 = absSpeed;          // rapport cyclique du PWM pour le moteur 2
            GPIOA->BSRR = (1<<(11+16*sens)); // Mise à "sens" du signal de direction
            break;    
        case MOTOR3 : 
            TIM1->CCR2 = absSpeed;          // rapport cyclique du PWM pour le moteur 3
            GPIOA->BSRR = (1<<(12+16*sens)); // Mise à "sens" du signal de direction
            break;
        case MOTOR4 : 
            TIM1->CCR3 = absSpeed;          // rapport cyclique du PWM pour l'éventuel moteur 4
            GPIOC->BSRR = (1<<(3+16*sens)); // Mise à "sens" du signal de direction

            break;
            }
} 
// Impose la vitesse de consigne du moteur n°motorNum en Boucle fermée (asservi)
// La vitesse de consigne doit être inférieure à Vmax (environ 900) en valeur absolue.
void setMotorSpeedBF(unsigned char motorNum, short speed){          
    switch(motorNum){
        case MOTOR1 :
            consigneVitesseMoteurBO1 = speed;
            break;
        case MOTOR2 : 
            consigneVitesseMoteurBO2 = speed;
            break;    
        case MOTOR3 : 
            consigneVitesseMoteurBO3 = speed;
            break;
        case MOTOR4 : 
            break;
            }
 
}
