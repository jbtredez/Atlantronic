
#ifndef MACRO_FONCTION_H 
#define MACRO_FONCTION_H 

#define LIGNE 6
#define COLONNE 8
#define NB_CASE LIGNE*COLONNE

#define INFINI NB_CASE + LIGNE

#define CASE_BLEU 8
#define CASE_ROUGE 1

#define OBSTACLE '@' //tout ce qui suit en dessous
#define ROBOT_ADV 'A'
#define ROBOT_ATL 'R'
#define QUEEN 'Q'
#define KING 'K'
#define PION 'P'
#define RIEN 0

char table[NB_CASE];

//verifie les ptits murets
unsigned char check_path(int case1, int case2);
unsigned char check_path_with_coordinate(int c1, int l1, int c2, int l2);

int coutNonRecursif(int dc, int dl, int ac, int al, int *pathc, int *pathl);
// recrussif !!!! et bug dans certaines configurations
int cout(int dc, int dl, int ac, int al, int *pathc, int *pathl);
unsigned int BisonFute(int dc, int dl, int ac, int al, int oc, int ol, int *tribordc, int *tribordl, int *babordc, int *babordl);
int Bresenham(int c1, int l1, int c2, int l2, int *oc, int *ol);

//
void update_table(int numCase, char objet);
void afficher_table(void);
void init_table(void);

//tools
unsigned char caseToVecteur(int nCase, int *x, int *y);
unsigned char vecteurToCase(int x, int y, int *nCase);

#endif //MACRO_FONCTION_H 
