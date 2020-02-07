

// Variables pour la selection d'un des tests
#define Test_mot_BO 1
#define Test_mot_BF 2
#define Test_cmd_dir 3
#define Test_traj 4
#define Deter_dx 5
#define Deter_dTheta 6

// Variables pour avoir les différentes directions pour le calcul du dx
#define Y_pos 1
#define X_pos_Y_neg 2
#define XY_neg 3
#define Y_neg 4
#define X_neg_Y_pos 5
#define XY_pos 6

/*---- Fonctions pour les tests ----*/

//La Fonction principale
void tests_Tribot(char Test_voulu);

//Les différentes sous fonctions de tests appelées
void test_mot_BO(short duree_test_ms, short v_mot);
void test_mot_BF(short t_ass_mot_Xus, short duree_test_ms, short v_mot);
void test_cmd_dir(short t_ass_mot_Xus, double theta_test, short v_trans, short v_angul);
void test_traj(short t_ass_mot_Xus, short t_ass_traj_X100us, double theta_test, short cibl_x_test, short cibl_y_test);
void deter_dx(short t_ass_mot_Xus, char direction, short duree_test_ms, short v_mot);
void deter_dTheta(short t_ass_mot_Xus, signed char sens, short duree_test_ms, short v_mot);



