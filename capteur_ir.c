#include "hal.h"
#include "sensors/proximity.h"
#include <main.h>
#include "capteur_ir.h"


// ### Threshold val_ir_stop: this value can be changed depending on the setup conditions
static unsigned int threshold_val_ir_stop =250 ;

// ### Etat marche can be (CLEAR or OBSTACLE)
static int etat_marche;


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

unsigned int get_etat_marche(void)
{
	return etat_marche;
}

static THD_WORKING_AREA(capteur_ir_thd_wa,128);
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
