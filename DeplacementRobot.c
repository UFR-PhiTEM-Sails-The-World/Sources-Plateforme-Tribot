#include <stm32f4xx_hal.h>
#include "controleMoteurs.h"
#include "DeplacementRobot.h"
#include <stdio.h>
#include <math.h>

//extern Serial pc;
//extern DigitalOut myled;

double trajectoire[1500];
short indexTrajectoire=0;
double KPtt=6.0,KPtr=1500.0,KItt=0.02,KItr=10.0 ;
//double KPtt=0.6,KPtr=150.0,KItt=0.002,KItr=1.0 ;


//void IT_trajectoire(void){
void TIM1_UP_TIM10_IRQHandler(void){
    double xc, yc,thetac,dist,d_thetatmp,d_theta;
    double angleCible;
    static double distInt = 0, d_thetaInt = 0;
    if((TIM10->SR & 1) !=0){ // Si le drapeau d'interruption du Timer10 est levé
    //myled = !myled;
        //pc.printf("index : %d \n\r",indexTrajectoire);
        //pc.printf("%lf , %lf , %lf \n\r", trajectoire[3*indexTrajectoire-1],trajectoire[3*indexTrajectoire-2],trajectoire[3*indexTrajectoire-3]);
        xc = trajectoire[3*indexTrajectoire-1];
        yc = trajectoire[3*indexTrajectoire-2];
        thetac = trajectoire[3*indexTrajectoire-3];
        dist = sqrt((xc-X_a)*(xc-X_a)+(yc-Y_a)*(yc-Y_a));
        
        //pc.printf("theta = %lf \n\r",THETA_a);

        //translationPlusRotation(atan((yc-Y_a)/(xc-X_a))-THETA_a,KPtt*dist,KPtr*(thetac-THETA_a));
        d_thetatmp = thetac-THETA_a;
        //angleCible = atan2((yc-Y_a),(xc-X_a))-(d_theta/2);
        //pc.printf("%lf ,",d_theta);
        if( d_thetatmp > M_PI)
            d_theta = d_thetatmp - (2*M_PI); 
        else if ( d_thetatmp < -M_PI) 
            d_theta = d_thetatmp + (2*M_PI); 
        else 
            d_theta = d_thetatmp;
        // Calcul de l'erreur intégrale    
        if (distInt<2000)distInt+=dist;
        if (d_thetaInt<2000)d_thetaInt+=d_theta;

        //pc.printf("%lf\n\r",180*angleCible/M_PI);
        angleCible = atan2((yc-Y_a),(xc-X_a)) - THETA_a - (d_theta/2);
                    //pc.printf("theta_a = %lf, theta_c = %lf, d_theta = %lf,  \n\r",THETA_a,thetac,d_theta);
        printf("%lf\n\r",180*angleCible/M_PI);

        
        translationPlusRotation(angleCible,(short)(KPtt*dist+KItt*distInt),KPtr*d_theta+KItr*d_thetaInt);
        if(indexTrajectoire>1)
            indexTrajectoire--;
        
            
        
        if(indexDATA<50){
            //pc.printf("%d\n\r",indexDATA);
            
            //DATA[2*indexDATA] = mesureVitesseMoteur3;//mesV3;
            //Data_d[indexDATA] = dist;
            Data_d[12*indexDATA] = X_a;
            Data_d[12*indexDATA+1] = Y_a;
            Data_d[12*indexDATA+2] = THETA_a;
            Data_d[12*indexDATA+3] = xc;
            Data_d[12*indexDATA+4] = yc;
            Data_d[12*indexDATA+5] = thetac;
            Data_d[12*indexDATA+6] = dist;
            Data_d[12*indexDATA+7] = d_thetatmp;
            Data_d[12*indexDATA+8] = d_theta; 
            Data_d[12*indexDATA+9] = angleCible; 
            Data_d[12*indexDATA+10] = (short)(KPtt*dist); 
            Data_d[12*indexDATA+11] = KPtr*d_theta; 
                   
           /* Data_d[3*indexDATA] = KP*erreur1;
            Data_d[3*indexDATA+1] = KI*erreurIntegrale1;
            Data_d[3*indexDATA+2] = (double)commandeVitesseMoteur1;
            */
            /*DATA[4*indexDATA] = mesV1;//mesV1;
            DATA[4*indexDATA+1] = erreur1;
            DATA[4*indexDATA+2] = commandeVitesseMoteur1;
            DATA[4*indexDATA+3] = TIM1->CCR1;
            */
            //DATA[3*indexDATA] = mesV1;
            //DATA[3*indexDATA+1] = mesV2;
            //DATA[3*indexDATA+2] = mesV3;
            
            indexDATA++;
        }
        
        TIM10->SR&=~1;                   // Mise à zÈro du drapeau d'interruption    
    }   
}

void rotationRobot(short vitesseAngulaire)// vitesse angulaire entre 0 et Vmax 
{   
    if(vitesseAngulaire<Vmax && vitesseAngulaire>-Vmax)
    {
        setMotorSpeedBF(MOTOR1,vitesseAngulaire);
        setMotorSpeedBF(MOTOR2,vitesseAngulaire);
        setMotorSpeedBF(MOTOR3,vitesseAngulaire);
    }
    else if(vitesseAngulaire>=Vmax)
    {
        setMotorSpeedBF(MOTOR1,Vmax);
        setMotorSpeedBF(MOTOR2,Vmax);
        setMotorSpeedBF(MOTOR3,Vmax);
    }
    else 
    {
        setMotorSpeedBF(MOTOR1,-Vmax);
        setMotorSpeedBF(MOTOR2,-Vmax);
        setMotorSpeedBF(MOTOR3,-Vmax);
    }        
}
void translationRobot(double angle, short vitesseTranslation) //Vitesse de translation entre 0 et Vmax
{
    double V1,V2,V3;
    short vit=vitesseTranslation;
    if(vitesseTranslation>=Vmax)
    {
        vit = Vmax;
    }
    else if(vitesseTranslation<=-Vmax)
    {
        vit = -Vmax;
    }
    
    V1 = (double)vit*sin(angle-alpha1);
    V2 = (double)vit*sin(angle-alpha2);
    V3 = (double)vit*sin(angle-alpha3);
    setMotorSpeedBF(MOTOR1,V1);
    setMotorSpeedBF(MOTOR2,V2);
    setMotorSpeedBF(MOTOR3,V3);
}  
void translationPlusRotation(double angle, short vitesseTranslation,short vitesseAngulaire) //Somme des Vitesses translation entre 0 et Vmax
{
    double V1,V2,V3;
    short vit=abs(vitesseTranslation)+abs(vitesseAngulaire);
        //pc.printf("vit : %d \n\r", vit);

    long vitT = vitesseTranslation, vitR = vitesseAngulaire;
    if(vit>=Vmax)
    {
        vitT = vitesseTranslation*Vmax/vit;
        vitR = vitesseAngulaire*Vmax/vit;       
    }

    
    V1 = (double)vitT*sin(angle-alpha1)+(double)vitR;
    V2 = (double)vitT*sin(angle-alpha2)+(double)vitR;
    V3 = (double)vitT*sin(angle-alpha3)+(double)vitR;
    setMotorSpeedBF(MOTOR1,V1);
    setMotorSpeedBF(MOTOR2,V2);
    setMotorSpeedBF(MOTOR3,V3);
}  
void allerAuPoint(double X_cons, double Y_cons, double THETA_cons)
{
    double dist = sqrt((X_cons-X_a)*(X_cons-X_a)+(Y_cons-Y_a)*(Y_cons-Y_a)+88*88*(THETA_cons-THETA_a)*(THETA_cons-THETA_a) );
    int nBPoint = dist/20;// nombre de points intermédiaires pour une vitesse basse de 100mm/s 
    short i;
    indexTrajectoire =  nBPoint+1;
    for(i=0;i<nBPoint;i++){
        trajectoire[3*(nBPoint-i+1)-1] = X_a+(i+1)*(X_cons-X_a)/nBPoint;
        trajectoire[3*(nBPoint-i+1)-2] = Y_a+(i+1)*(Y_cons-Y_a)/nBPoint;
        trajectoire[3*(nBPoint-i+1)-3] = THETA_a+(i+1)*(THETA_cons-THETA_a)/nBPoint;    
    }
    trajectoire[2] = trajectoire[5]; // dernier point identique au précédent pour assurer l'arret.
    trajectoire[1] = trajectoire[4];
    trajectoire[0] = trajectoire[3];   
    
    printf("dist initiale : %lf \n\r", dist);
    
     
     
        
    
    for(i=0;i<=indexTrajectoire;i++)
    {
         printf("%lf , %lf , %lf \n\r", trajectoire[3*i],trajectoire[3*i+1],trajectoire[3*i+2]);   
    }
    
    TIM10->CR1 |=1;                  // Activation du Timer10 (demmarage du suivi de trajectoire)
    while( dist>10 ){
        //printf("%lf , %lf , %lf °\n\r", X_a,Y_a,THETA_a*180/M_PI);

        dist = sqrt((X_cons-X_a)*(X_cons-X_a)+(Y_cons-Y_a)*(Y_cons-Y_a)+88*88*(THETA_cons-THETA_a)*(THETA_cons-THETA_a) );
        //pc.printf("d= %lf \n\r", dist);
        //wait_ms(10);
        if(ARRET==1)break;
    }
     // Objectif atteint, arret du déplacement
     TIM10->CR1 &=~1;     // Arret du Timer10 qui génère l'IT périodique
    setMotorSpeedBF(MOTOR1,0);
    setMotorSpeedBF(MOTOR2,0);
    setMotorSpeedBF(MOTOR3,0);
     
}
    

   
