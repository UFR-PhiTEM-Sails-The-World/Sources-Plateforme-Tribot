#define M_PI           3.14159265358979323846
#define alpha1 (-M_PI/2)
#define alpha2 (M_PI/6)
#define alpha3 (5*M_PI/6)
#define Vmax 900

// Coordonnées absolues d'odométrie
extern double X_a,Y_a;
extern double THETA_a;
extern double Data_d[3000];
extern int DATA[2000];
extern short indexDATA;
extern int ARRET;

void rotationRobot(short vitesseAngulaire);
void translationRobot(double angle, short vitesseTranslation);
void translationPlusRotation(double angle, short vitesseTranslation,short vitesseAngulaire); //Somme des Vitesses translation entre 0 et Vmax
void allerAuPoint(double X_cons, double Y_cons, double THETA_cons);
