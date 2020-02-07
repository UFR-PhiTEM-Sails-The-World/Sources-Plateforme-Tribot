

void init_TIM1_PWMx4(void);         // Initialisation du Timer1 pour 4 sorties PWM sur PA8, PA9, PA10 et PA11, F=20kHz, ARR=2100, donc 2100 valeurs possibles pour le rapport cyclique
void init_TIM8_PWMx4_Servo(void);
void init_encoders_Nb_Impulsions(void);
void init_ES(void);
void initTimer12PWM(void);
void init_Timer9_IT_Xus(unsigned int x);    // Initialisation du Timer9 pour générer une interruption toutes les x us
void init_encoders_Periode(void);
void init_Timer10_IT_X100us(unsigned int x);
void init_Telemetre_US_I2C(void);
void ADC_Init_IR(void);

