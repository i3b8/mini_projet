#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "sensors/proximity.h"
#include "memory_protection.h"

#include <main.h>

#include "communication.h"
#include "capteur_ir.h"

//*** Parameters Declaration ***
// ### Threshold val_ir_stop: this value can be changed depending on the setup conditions
static unsigned int threshold_val_ir_stop =250 ;
// ### if etat_marche=0 --> we detected obstacle , if 1: no obstacle detected
static int etat_marche;
//*** End Parameters Declaration ***

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


void set_etat_marche(unsigned int valeur)
{
	etat_marche=valeur;
}
unsigned int get_etat_marche(void)
{
	return etat_marche;
}


static THD_WORKING_AREA(capteur_ir_thd_wa,1024);
static THD_FUNCTION(capteur_ir_thd,arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);
	while(1)
	{
		switch(etat_marche)
			{
				case OBSTACLE: // In front of an Obstacle
					if((unsigned int)get_prox(0)<threshold_val_ir_stop)
					{
						etat_marche=CLEAR_ROAD;
						chThdSleepMilliseconds(110);
						break;
					}
					else
					{
						chThdSleepMilliseconds(12);
						break;

					}
				case CLEAR_ROAD: // No front Obstacles detected
					if((unsigned int)get_prox(0)>threshold_val_ir_stop)
					{
						etat_marche=OBSTACLE;
						chThdSleepMilliseconds(110);
						break;
					}
					else
					{
						chThdSleepMilliseconds(12);
						break;
					}
			}
	}
}

void initialiser_capteur_ir(void)
{
	etat_marche=OBSTACLE;
	chThdCreateStatic(capteur_ir_thd_wa,sizeof(capteur_ir_thd_wa),NORMALPRIO,capteur_ir_thd,NULL);
}
void initialiser_message_for_prox_ir(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);
}
