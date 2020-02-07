#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

#define M_PI           3.14159265358979323846
#define alpha1 (-M_PI/2)
#define alpha2 (M_PI/6)
#define alpha3 (5*M_PI/6)

// Gains pour le correcteur PI pour l'asservissement en vitesse de chaque moteur/
#define KP 10.0
#define KI 1.0
//float KP=10,KI=1.5;//0.8;

//Variation d'angle pour une impulsion de codeur en radian
#define dTheta 0.00039351324
//Variation de distance perpendiculairement à l'axe de la roue pour une impulsion de codeur en mm
#define dx 0.08937716158

// Coordonnées absolues d'odométrie
extern double X_a,Y_a;
extern double THETA_a;

//Pour savoir le sens de rotation des moteurs
#define sens1 1
#define sens2 -1

//extern Serial pc;
extern short consigneVitesseMoteurBO1,consigneVitesseMoteurBO2,consigneVitesseMoteurBO3,consigneVitesseMoteurBO4;
extern short mesureVitesseMoteur1,mesureVitesseMoteur2,mesureVitesseMoteur3,mesureVitesseMoteur4;
extern int DATA[2000];
extern short indexDATA;

void motorsBreak(void);             // Désactive les 4 sorties PWM du Timer1 
void motorsEnable(void);            // Active les 4 sorties PWM du Timer1

// Impose une valeur de vitesse en boucle ouverte pour un moteur donné (directement dans le registre de comparaison) Speed dans [-4500;4500]
void setMotorSpeedBO(unsigned char motorNum, short speed);  

// Impose la vitesse de consigne du moteur n°motorNum en Boucle fermée (asservi)
// La vitesse de consigne doit être inférieure à Vmax (environ 900) en valeur absolue.
void setMotorSpeedBF(unsigned char motorNum, short speed);

