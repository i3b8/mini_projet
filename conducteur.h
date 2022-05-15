#ifndef CONDUCTEUR_H_
#define CONDUCTEUR_H_
enum state_conductor
{
	WAIT_SIGNAL_START=0,
	CONDUCTOR_MOVING=1,
	CONDUCTOR_STOPPED=2,
	CONDUCTOR_COMING_BACK=3
};
/*
 * Initialise Conductor (Thread and State of Conductor)
 */
void initialiser_conducteur(void);

#endif /* CONDUCTEUR_H_ */
