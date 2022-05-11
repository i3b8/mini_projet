
#ifndef CONDUCTEUR_H_
#define CONDUCTEUR_H_

void turn_right(void); // make a quarter turn to the right
void turn_left(void); // make a quarter turn to the left
void motor_stop(void); // stop the motors
void move_forward(void);// move forward by a default speed


// partie des fonctions finales pour utiliser dans le projet
void move_forwd_steps(int steps); // move forward for given steps
void initialiser_conducteur(void);

#endif /* CONDUCTEUR_H_ */
