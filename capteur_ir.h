#ifndef CAPTEUR_IR_H_
#define CAPTEUR_IR_H_

enum state_front_road

{
	CLEAR_ROAD=0,
	OBSTACLE=1
};
//Initialise state_front_ at Obstacle and Create the Thread
void initialiser_capteur_ir(void);

//Initialise The msg_bus before using proximity_ir (otherwise --> A panic handler will be generated)
void initialiser_message_for_prox_ir(void);

//Returns one of the state_front road
unsigned int get_etat_marche(void);

#endif /* CAPTEUR_IR_H_ */
