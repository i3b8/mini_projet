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

//static uint8_t motor_stopped ;  // boolean (only zero or one values will be used )
static unsigned int threshold_val =250 ; // this value can be changed depended on the conditions
//static unsigned int etat_marche =0;
static unsigned int etat_marche =2; // juste au début
static unsigned int etat_do = 0;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
/*
void set_motor_stopped(uint8_t val)
{
	motor_stopped = val ;
}
uint8_t get_motor_stopped(void)
{
	return motor_stopped ;
}
void update_capteur(unsigned int ir_value)
{
	if(ir_value >threshold_val)
	{
		motor_stop();
		motor_stopped=1;
	}
}
void initialiser_capteur(void)
{
	motor_stopped=0;
}
*/
unsigned int get_etat_do(void)
{
	return etat_do;
}
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
	systime_t time;
	while(1)
	{
		//time = chVTGetSystemTime();
		switch(etat_marche)
			{
				case 0:
					// 1ère méthode c'est à dire on est à l'arret
					if((unsigned int)get_prox(0)<threshold_val)
					{
						etat_marche=1;
						chThdSleepMilliseconds(110);
						break;
					}
					else
					{
						chThdSleepMilliseconds(12);
						break;

					}
				case 1:
					//chThdSleepMilliseconds(11); //  on a besoin de 10 Ms pour faire la mise à jour de tous les cpateurs (on aurait pu prendre moins vu qu'on utilise
					//un seul capteur

					//messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
					//proximity_msg_t proximity_values;
					//messagebus_topic_wait(proximity_topic, &proximity_values, sizeof(proximity_values));

					if((unsigned int)get_prox(0)>threshold_val)
					{
						etat_marche=0;//c'est à dire on va s'arreter
						//etat_marche=0;
						 // ces valeurs sont choisies de façon empirique
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
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	chThdCreateStatic(capteur_ir_thd_wa,sizeof(capteur_ir_thd_wa),NORMALPRIO,capteur_ir_thd,NULL);
}
