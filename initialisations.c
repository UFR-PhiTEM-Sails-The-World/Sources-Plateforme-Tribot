#include "I2C.h"
#include "init_general.h"

extern void IT_codeurTim2(void);
extern void IT_codeurTim3(void);
extern void IT_codeurTim4(void);
extern void IT_codeurTim5(void);
extern void IT_asservissement(void);
/////extern void IT_trajectoire(void);

void init_TIM1_PWMx4(void){
    /**** Horloge ****/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  //GPIOAEN='1' -> activation de l'horloge sur le port A
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // Activation Horloge sur TIM1 (180MHz)

    /**** GPIO ****/
    // PA8, PA9, PA10 et PA11 en alternate fonction
    GPIOA->MODER |= ( (1<<17)|(1<<19)|(1<<21)|(1<<23) );
    GPIOA->MODER &= ~( (1<<16)|(1<<18)|(1<<20)|(1<<22) );   
    //GPIOA->OSPEEDR |= 0x30000; // Set the PWM outputs to high speed (OC1)
    
    // AF1 pour PA8, PA9,PA10 et PA11 (pour TIM1_CH1, TIM1_CH2, TIM1_CH3 et TIM1_CH4)
    // AFRH8[3:0]=0001, AFRH9[3:0]=0001, AFRH10[3:0]=0001, AFRH11[3:0]=0001 
    GPIOA->AFR[1] |= ( (1<<0)|(1<<4)|(1<<8)|(1<<12) );
    GPIOA->AFR[1] &= ~( (7<<1)|(7<<5)|(7<<9)|(7<<13) );
    

    /**** TIM1 ****/
    TIM1->PSC = 0;      // pas de prédivision, le compteur compte à 84Mhz
    TIM1->ARR = 4500;   // Valeur pour une période de PWM de 20KHz en mode center-aligned
    TIM1->EGR |=1;      // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM1->CR1|=( (1<<7)|(1<<5) );   // Mode de comptage center-aligned, mode auto-reload preload activé   
    TIM1->CR2=0;
    TIM1->SMCR=0;
    TIM1->DIER=0;
		
    //OC1M[2:0] = "110" et OC2M[2:0] = "110" PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive
    TIM1->CCMR1 |= ( (3<<5)|(3<<13) );
    TIM1->CCMR1 &= ~( (1<<4)|(1<<12) );
    // CC1S[1:0]=CC2S[1:0]="00" : CC1 and CC2 channel is configured as output
    TIM1->CCMR1 &= ~( (3<<0)|(3<<8) );
    
    //OC3M[2:0] =OC4M[2:0] = "110" : PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive
    TIM1->CCMR2 |= ( (3<<5)|(3<<13) );
    TIM1->CCMR2 &= ~( (1<<4)|(1<<12) );
    // CC3S[1:0]=CC4S[1:0]="00" : CC3 channel is configured as output
    TIM1->CCMR2 &= ~( (3<<0)|(3<<8) );
    
    TIM1->CCER |= ( (1<<0)|(1<<4)|(1<<8)|(1<<12) ); // sorties OC1, OC2, OC3 et OC4 actives
    
    // Initialisation des rapports cycliques 
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
    
    //Activation des sorties OCx
    TIM1->BDTR |= ((1<<15)); // MOE=1 pour activer les sorties OC
  
    TIM1->CR1|=1;       // Enable TIM1
}
void init_TIM8_PWMx4_Servo(void){
    /**** Horloge ****/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  //GPIOAEN='1' -> activation de l'horloge sur le port A
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;  // Activation Horloge sur TIM8 (180MHz)

    /**** GPIO ****/
    // PC6, PC7, PC8 et PC9 en alternate fonction
    GPIOC->MODER |= ( (1<<13)|(1<<15)|(1<<17)|(1<<19) );
    GPIOC->MODER &= ~( (1<<12)|(1<<14)|(1<<16)|(1<<18) );   
    //GPIOA->OSPEEDR |= 0x30000; // Set the PWM outputs to high speed (OC1)
    
    // AF3 pour PC6, PC7,PC8 et PC9 (pour TIM8_CH1, TIM8_CH2, TIM8_CH3 et TIM8_CH4)
    // AFRH6[3:0]=0011, AFRH7[3:0]=0011, AFRH8[3:0]=0011, AFRH9[3:0]=0011 
    GPIOC->AFR[0] |= ( (3<<24)|(3<<28) );
    GPIOC->AFR[0] &= ~( (3<<26)|(3<<30) );
    GPIOC->AFR[1] |= ( (3<<0)|(3<<4) );
    GPIOC->AFR[1] &= ~( (3<<2)|(3<<6) );
    

    /**** TIM1 ****/
    TIM8->PSC = 179;      // prédivision par 180, le compteur compte à 1Mhz
    TIM8->ARR = 9999;   // Valeur pour une période de PWM de 50Hz en mode center-aligned
    TIM8->EGR |=1;      // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM8->CR1|=( (1<<7)|(1<<5) );   // Mode de comptage center-aligned, mode auto-reload preload activé   
    TIM8->CR2=0;
    TIM8->SMCR=0;
    TIM8->DIER=0;
  
    //OC1M[2:0] = "110" et OC2M[2:0] = "110" PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive
    TIM8->CCMR1 |= ( (3<<5)|(3<<13) );
    TIM8->CCMR1 &= ~( (1<<4)|(1<<12) );
    // CC1S[1:0]=CC2S[1:0]="00" : CC1 and CC2 channel is configured as output
    TIM8->CCMR1 &= ~( (3<<0)|(3<<8) );
    
    //OC3M[2:0] =OC4M[2:0] = "110" : PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive
    TIM8->CCMR2 |= ( (3<<5)|(3<<13) );
    TIM8->CCMR2 &= ~( (1<<4)|(1<<12) );
    // CC3S[1:0]=CC4S[1:0]="00" : CC3 channel is configured as output
    TIM8->CCMR2 &= ~( (3<<0)|(3<<8) );
    
    TIM8->CCER |= ( (1<<0)|(1<<4)|(1<<8)|(1<<12) ); // sorties OC1, OC2, OC3 et OC4 actives
    
    // Initialisation des rapports cycliques 
    TIM8->CCR1 = 750;
    TIM8->CCR2 = 500;
    TIM8->CCR3 = 1000;
    TIM8->CCR4 = 600;
    
    //Activation des sorties OCx
    TIM8->BDTR |= ((1<<15)); // MOE=1 pour activer les sorties OC
  
    TIM8->CR1|=1;       // Enable TIM8
}
void initTimer12PWM(void){
    //CONFIG TIMER POUR PWM : Timer 11 pour la sortie CH1 en alternate function sur PB9
    
    RCC->AHB1ENR |= 1<<1;  //GPIOBEN='1' -> activation de l'horloge sur le port B

    //PB9 en alternate fonction
    GPIOB->MODER|=1<<19;
    GPIOB->MODER&=~(1<<18);
    GPIOB->AFR[1]|=(3<<4); //AF3 pour le Timer11 sur PB9 voir RMp190
    
    // Initialisation RCC 
    RCC->APB2ENR|=0x1<<18; // Activation horloge sur TIM11
    
    TIM11->ARR=4095;
    TIM11->PSC=4; //division par 4 pour avoir une fréquence du PWM de 10kHz environ
    TIM11->EGR |=1;     //mise à 1 bit UG de EGR pour provoq chargt de PSC
    
    TIM11->CCR1=1000;
    TIM11->CCMR1|=3<<5; //mode PWM sur chanel 1
    TIM11->CCER|=1;// sortie OC1 active
    
    TIM11->SR&=~1;//Clear the update flag
    TIM11->CR1|=1;// Enable TIM12
}


void init_ES(void){
    /**** Horloge ****/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIOAEN;  //GPIOAEN=GPIOCEN='1' -> activation de l'horloge sur les ports A et C

    /**** GPIO ****/
    //PC0, PC1, PC2, PC3 en sortie (DIR pour chaque pont en H)
    //PA7, PA11, PA12 en sortie (DIR pour chaque pont en H)
    GPIOA->MODER |= ( (1<<14)|(1<<22)|(1<<24) );
    GPIOA->MODER &= ~( (2<<14)|(2<<22)|(2<<24) );
    GPIOC->MODER &= ~( (1<<26)|(1<<27) );  // PC13 en entrée -> bouton USER
           
}

void init_encoders_Nb_Impulsions(void){
    /**** Horloge ****/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;//GPIOAEN='1' -> activation de l'horloge sur le port A
    RCC->APB1ENR |= 1<<0;               // Activation Horloge sur TIM2 (84MHz)
    /**** GPIO ****/
    //PA0 et PA1 en AF MODER0[1:0] = MODER1[1:0] = "10"
    GPIOA->MODER |= ( (1<<1)|(1<<3) );
    GPIOA->MODER &= ~( (1<<0)|(1<<2) ); 
    GPIOA->PUPDR |= ( (1<<0)|(1<<2) ); // Pull-up sur PA0 et PA1
    GPIOA->PUPDR &= ~( (1<<1)|(1<<3) ); 
    
    // AF1 pour PA0 et PA1 (pour TIM2_CH1, TIM2_CH2)
    // AFRH0[3:0]=0001, AFRH1[3:0]=0001 
    GPIOA->AFR[0] |= ( (1<<0)|(1<<4) );
    GPIOA->AFR[0] &= ~( (7<<1)|(7<<5) );
    
    /**** TIM2 ****/
    TIM2->PSC = 0;      // pas de prédivision, le compteur compte en fonction des entrées
    TIM2->ARR = 65535;  // Valeur max sur 16 bit
    TIM2->EGR |=1;      // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM2->CR1|=( (1<<7) );   // Mode auto-reload preload activé   
    TIM2->CR2=0;
    TIM2->SMCR=(3<<0); //SMS[1:0]="011" mode encoder 3 (comptage sur TI1 et TI2) 
    TIM2->DIER=0;
    TIM2->CCMR1=0;    
    TIM2->CCMR2=0;    
    TIM2->CCER=0;    // Pas d'inversion sur les entrées

    TIM2->CR1|=( (1<<0) );   // Enable TIM2   
}
void init_encoders_Periode(void){
      
      /**** Horloge ****/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN; // GPIOAEN = GPIOBEN = '1'  -> activation de l'horloge sur le port A et B
    RCC->APB1ENR |= ((1<<0)|(1<<1)|(1<<2));    // Activation Horloge sur TIM2, TIM3 et TIM4 (84MHz)
    
    /**** GPIO ****/
    // Codeur1 cablé sur PA15/PB3 -> Timer2 / AF1, PA15->TIM2_CH1 et PB3->TIM2_CH2
    // Codeur2 cablé sur PB4/PB5  -> Timer3 / AF2, PB4 ->TIM3_CH1 et PB5->TIM3_CH2
    // Codeur3 cablé sur PB6/PB7  -> Timer4 / AF2, PB6 ->TIM4_CH1 et PB7->TIM4_CH2
    // Codeur4 cablé sur PA0/PA1  -> Timer5 / AF2, PA0 ->TIM5_CH1 et PA1->TIM5_CH2
    
    //PA15 en AF MODER15[1:0] = "10" et PB3 en entrée MODER3[1:0] = "00"
    GPIOA->MODER |= ( (1<<31) );
    GPIOA->MODER &= ~( (1<<30) ); 
    GPIOB->MODER &= ~( (1<<6)|(1<<7) ); 
    GPIOA->PUPDR |= ( (1<<30) ); // Pull-up sur PA15
    GPIOA->PUPDR &= ~( (1<<31) );  
    GPIOB->PUPDR |= ( (1<<6) ); // Pull-up sur PB3
    GPIOB->PUPDR &= ~( (1<<7) );  
    // AF1 pour PA15 (pour TIM2_CH1)
    // AFRH15[3:0]="0001" 
    GPIOA->AFR[1] |= ( (1<<28) );
    GPIOA->AFR[1] &= ~( (7<<29) );
    
    //PB4 en AF MODER4[1:0] = "10" et PB5 en entrée MODER5[1:0] = "00"
    GPIOB->MODER |= ( (1<<9) );
    GPIOB->MODER &= ~( (1<<8)|(1<<10)|(1<<11) ); 
    GPIOB->PUPDR |= ( (1<<8)|(1<<10) ); // Pull-up sur PB4 et PB5
    GPIOB->PUPDR &= ~( (1<<9)|(1<<11) );  
    // AF2 pour PB4 (pour TIM3_CH1)
    // AFRH4[3:0]="0010" 
    GPIOB->AFR[0] |= ( (1<<17) );
    GPIOB->AFR[0] &= ~( (1<<16)|(3<<18) );
    
    //PB6 en AF MODER6[1:0] = "10" et PB7 en entrée MODER7[1:0] = "00"
    GPIOB->MODER |= ( (1<<13) );
    GPIOB->MODER &= ~( (1<<15)|(1<<14)|(1<<12) ); 
    GPIOB->PUPDR |= ( (1<<12)|(1<<14) ); // Pull-up sur PB6 et PB7
    GPIOB->PUPDR &= ~( (1<<13)|(1<<15) );  
    // AF2 pour PB6 (pour TIM4_CH1)
    // AFRH6[3:0]="0010" 
    GPIOB->AFR[0] |= ( (1<<25) );
    GPIOB->AFR[0] &= ~( (1<<24)|(3<<26) );
    
    //PA0 en AF MODER0[1:0] = "10" et PA1 en entrée MODER1[1:0] = "00"
    GPIOA->MODER |= ( (1<<1) );
    GPIOA->MODER &= ~( (1<<0)|(1<<2)|(1<<3) ); 
    GPIOA->PUPDR |= ( (1<<0)|(1<<2) ); // Pull-up sur PA0 et PA1
    GPIOA->PUPDR &= ~( (1<<1)|(1<<3) );  
    // AF2 pour PA0 (pour TIM2_CH1)
    // AFRH0[3:0]="0001" 
    GPIOA->AFR[0] |= ( (1<<1) );
    GPIOA->AFR[0] &= ~( (1<<0)|(3<<2) );
    
    
    
    /**** TIM2 ****/
    TIM2->PSC = 179;            // Prédivison par 180 donc T_CK_CNT=1us
    TIM2->ARR = 49999;          // Valeur max correspondant à une période de 50ms (au dela de cela on considèrera une vitesse nulle).
    TIM2->EGR |=1;              // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM2->CR1 = ( (1<<7) );     // Mode auto-reload preload activé   
    TIM2->CR2=0;
    TIM2->SMCR=0;  
 
    TIM2->CCMR1=0x0031;         // Activation d'un filtrage de 8 échantillon sur l'entrée pour éviter le déclenchement sur parasites + Mode Input Capture sur a voie 1
    TIM2->CCMR2=0;    
    TIM2->CCER = (1<<1)|(1<<3); // Déclenchement de la capture sur les deux front, Montant ET descendant de la voie 1
    
    TIM2->SR&=~3;               // Abaissement des 2 flags pour démarrer les IT proprement
    TIM2->DIER=(1<<0)|(1<<1);   // Activation des IT sur input capture et update event (valeur max atteinte).
    // Remapping de la routine d'interruption
////    NVIC_SetVector(TIM2_IRQn,(uint32_t)IT_codeurTim2);
    // Activation de l'interruption par le controleur d'IT 
    NVIC_EnableIRQ(TIM2_IRQn);
    
    TIM2->CCER|=( (1<<0) );     // Activation de la capture du TIM2 
    TIM2->CR1|=( (1<<0) );      // Activation finale du TIM2  
    
    /**** TIM3 ****/
    TIM3->PSC = 179;            // Prédivison par 180 donc T_CK_CNT=1us
    TIM3->ARR = 49999;          // Valeur max correspondant à une période de 50ms (au dela de cela on considèrera une vitesse nulle).
    TIM3->EGR |=1;              // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM3->CR1 = ( (1<<7) );     // Mode auto-reload preload activé   
    TIM3->CR2=0;
    TIM3->SMCR=0;  
 
    TIM3->CCMR1=0x0031;         // Activation d'un filtrage de 8 échantillon sur l'entrée pour éviter le déclenchement sur parasites + Mode Input Capture sur a voie 1
    TIM3->CCMR2=0;    
    TIM3->CCER = (1<<1)|(1<<3); // Déclenchement de la capture sur les deux front, Montant ET descendant de la voie 1
    
    TIM3->SR&=~3;               // Abaissement des 2 flags pour démarrer les IT proprement
    TIM3->DIER=(1<<0)|(1<<1);   // Activation des IT sur input capture et update event (valeur max atteinte).
    // Remapping de la routine d'interruption
////    NVIC_SetVector(TIM3_IRQn,(uint32_t)IT_codeurTim3);
    // Activation de l'interruption par le controleur d'IT 
    NVIC_EnableIRQ(TIM3_IRQn);
    
    TIM3->CCER|=( (1<<0) );     // Activation de la capture du TIM3 
    TIM3->CR1|=( (1<<0) );      // Activation finale du TIM3   
    
    /**** TIM4 ****/
    TIM4->PSC = 179;            // Prédivison par 180 donc T_CK_CNT=1us
    TIM4->ARR = 49999;          // Valeur max correspondant à une période de 50ms (au dela de cela on considèrera une vitesse nulle).
    TIM4->EGR |=1;              // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM4->CR1 = ( (1<<7) );     // Mode auto-reload preload activé   
    TIM4->CR2=0;
    TIM4->SMCR=0;  
 
    TIM4->CCMR1=0x0031;         // Activation d'un filtrage de 8 échantillon sur l'entrée pour éviter le déclenchement sur parasites + Mode Input Capture sur a voie 2
    TIM4->CCMR2=0;    
    //TIM4->CCER = (1<<5)|(1<<7); // Déclenchement de la capture sur les deux front, Montant ET descendant de la voie 2
    TIM4->CCER = (1<<1)|(1<<3); // Déclenchement de la capture sur les deux front, Montant ET descendant de la voie 1

    TIM4->SR&=~3;               // Abaissement des 2 flags pour démarrer les IT proprement
    TIM4->DIER=(1<<0)|(1<<1);   // Activation des IT sur input capture et update event (valeur max atteinte).
    // Remapping de la routine d'interruption
////    NVIC_SetVector(TIM4_IRQn,(uint32_t)IT_codeurTim4);
    // Activation de l'interruption par le controleur d'IT 
    NVIC_EnableIRQ(TIM4_IRQn);
    
    TIM4->CCER|=( (1<<0) );     // Activation de la capture du TIM4 
    TIM4->CR1|=( (1<<0) );      // Activation finale du TIM4  
    
        /**** TIM5 ****/
/*
    TIM5->PSC = 179;            // Prédivison par 180 donc T_CK_CNT=1us
    TIM5->ARR = 49999;          // Valeur max correspondant à une période de 50ms (au dela de cela on considèrera une vitesse nulle).
    TIM5->EGR |=1;              // Mise à 1 bit UG de EGR pour provoqer chargement de PSC
    
    TIM5->CR1 = ( (1<<7) );     // Mode auto-reload preload activé   
    TIM5->CR2=0;
    TIM5->SMCR=0;  
 
    TIM5->CCMR1=0x3100;         // Activation d'un filtrage de 8 échantillon sur l'entrée pour éviter le déclenchement sur parasites + Mode Input Capture sur a voie 2
    TIM5->CCMR2=0;    
    TIM5->CCER = (1<<5)|(1<<7); // Déclenchement de la capture sur les deux front, Montant ET descendant de la voie 2
    
    TIM5->SR&=~3;               // Abaissement des 2 flags pour démarrer les IT proprement
    TIM5->DIER=(1<<0)|(1<<1);   // Activation des IT sur input capture et update event (valeur max atteinte).
    // Remapping de la routine d'interruption
    NVIC_SetVector(TIM5_IRQn,(uint32_t)IT_codeurTim5);
    // Activation de l'interruption par le controleur d'IT 
    NVIC_EnableIRQ(TIM5_IRQn);
    
    TIM5->CCER|=( (1<<0) );     // Activation de la capture du TIM5 
    TIM5->CR1|=( (1<<0) );      // Activation finale du TIM5 
     */
}

void init_Timer9_IT_Xus(unsigned int x){
    RCC->APB2ENR |=1<<16;           // Activation de l'horloge sur le Timer9
    TIM9->PSC = 179;                 // Prescaler de 180, donc T_CK_CNT=180/180M = 1 us
    TIM9->ARR = x-1;                // Comptage de 0 à x, donc un cycle de comptage en x us    
    TIM9->EGR |=1;                  // Mise à 1 bit UG de EGR pour provoq chargt de PSC
    
    TIM9->SR&=~1;                   //Mise à zéro du drapeau d'interruption pour démarrer dans de bones conditions
    TIM9->DIER|=1;                  // Activation de l'interruption sur l'evt de mise à jour (overflow)
    // Remapping de la routine d'interruption
//////NVIC_SetVector(TIM1_BRK_TIM9_IRQn,(uint32_t)IT_asservissement); 
    //Enable TIM1_update IRQ ->équivalent à la ligne suivante
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
   // NVIC->ISER[0]=1<<25;      // Activation de l'IT du timer9 (position 25 dans le tableau des IT)
    TIM9->CR1 |=1;                  // Activation du Timer1 (demmarage du comptage)
        
}
 void init_Timer10_IT_X100us(unsigned int x){
    RCC->APB2ENR |=1<<17;           // Activation de l'horloge sur le Timer10
    TIM10->PSC = 17999;             // Prescaler de 18000, donc T_CK_CNT=18000/180M = 100 us
    TIM10->ARR = x-1;                // Comptage de 0 à x, donc un cycle de comptage en x 100us    
    TIM10->EGR |=1;                  // Mise à 1 bit UG de EGR pour provoq chargt de PSC
    
    TIM10->SR&=~1;                   //Mise à zéro du drapeau d'interruption pour démarrer dans de bones conditions
    TIM10->DIER|=1;                  // Activation de l'interruption sur l'evt de mise à jour (overflow)
    // Remapping de la routine d'interruption
/////    NVIC_SetVector(TIM1_UP_TIM10_IRQn,(uint32_t)IT_trajectoire); 
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    //TIM10->CR1 |=1;                  // Activation du Timer1 (demmarage du comptage)
        
}   

void init_Telemetre_US_I2C(void){
    //I2C_Init();
}
void ADC_Init_IR(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOAEN='1' -> activation de l'horloge sur le port A
    GPIOA->MODER |= (3<<6)|(3<<8);  // MODER3[1:0]="11" -> PA3 en mode analogique,MODER4[1:0]="11" -> PA4 en mode analogique
    GPIOA->PUPDR &= ~(3<<6)|(3<<8);  // Pas de pull-up ni pull down sur PA3 et PA4 (incompatible avec l'utilisation de l'UART via STLINK.
    
    RCC->APB2ENR |= 1<<8;  // ADC1EN='1' -> activation de l'hologe sur l'ADC n∞1
    ADC1->CR1 = 0;          // RES[1:0]="00"->RÈsolution 12 bit (15ADCCLK cycle pour une conversion), pas d'IT, pas de watchDog, mode SCAN dÈsactivÈ
    ADC1->CR2 = 3;          // CONT='1' -> Continuous conversion mode, ADON='1' -> enable ADC    
    ADC1->SMPR2 |= 7<<9;   // SMP3[2:0]="111" -> sampling time 480 cycles sur la voie 3
    ADC1->SQR3 = 3;         // SQ1[2:0]="011" -> la premiËre (seule) conversion effectuÈe dans la sÈquence rÈguliËre est la voie 3
   // ADC1->SMPR2 |= 7<<12;   // SMP4[2:0]="111" -> sampling time 480 cycles sur la voie 4
   // ADC1->SQR3 = 4;         // SQ1[2:0]="011" -> la premiËre (seule) conversion effectuÈe dans la sÈquence rÈguliËre est la voie 4
    ADC1->CR2 |= 1<<30; 
}    
