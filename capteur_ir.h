#ifndef CAPTEUR_IR_H_
#define CAPTEUR_IR_H_
/*
void initialiser_capteur(void);
void set_motor_stopped(uint8_t val);
uint8_t get_motor_stopped(void);
void update_capteur(unsigned int ir_val);  // we take updated values for mesure_face to see if motor should stop or not
*/
void initialiser_capteur_ir(void);
void set_etat_marche(unsigned int valeur);
unsigned int get_etat_marche(void);
unsigned int get_etat_do(void);
#endif /* CAPTEUR_IR_H_ */
