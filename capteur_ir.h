#ifndef CAPTEUR_IR_H_
#define CAPTEUR_IR_H_

enum state_front_road
{
	CLEAR_ROAD=0,
	OBSTACLE=1
};
// créer le "Thread pour la gestion du capteur_proximity"
void initialiser_capteur_ir(void);
//initialiser le message_bus avant d'initialiser le capteur_prox
void initialiser_message_for_prox_ir(void);

void set_etat_marche(unsigned int valeur);
unsigned int get_etat_marche(void);

#endif /* CAPTEUR_IR_H_ */
