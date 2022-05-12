// Pour la partie des threads ///
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



#include <math.h>

#include "ch.h"
#include "hal.h"

//aussi rajoutée
#include <main.h>
// ******
#include "motors.h"
#include "capteur_ir.h"
//#include "audio_processing.c"
#include "conducteur.h"

#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.36f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     13   //cm
#define NSTEP_ONE_TURN      1000
#define default_speed_m		6.5 // cm/s

static unsigned int etat;
uint8_t motor_already_ordered ;
static uint8_t instruction_left_right =0;  // 0 pour nothing , 1 pour left , 2 pour right
// **************   INTERNAL FUNCTIONS ******************
// this function converts distance or speed from cm or cm/s to steps or steps/s

int convert_cm_to_steps(float cm)
{
	return cm* (NSTEP_ONE_TURN/WHEEL_PERIMETER ) ;
}
void delay(unsigned int n)
{
	while (n--)
	{
		__asm__ volatile ("nop");
	}
}

// ************ 	END INTERNAL Functions ***************

void turn_left(void)
{
	motor_already_ordered=1;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(-convert_cm_to_steps(default_speed_m));
	right_motor_set_speed(+convert_cm_to_steps(default_speed_m));
	while(left_motor_get_pos()>convert_cm_to_steps(-PERIMETER_EPUCK/4));
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	motor_already_ordered=0;

}
void turn_right(void)

{
	motor_already_ordered=1;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(convert_cm_to_steps(default_speed_m));
	right_motor_set_speed(-convert_cm_to_steps(default_speed_m));
	while(left_motor_get_pos()<convert_cm_to_steps(PERIMETER_EPUCK/4));
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	motor_already_ordered=0;
}

void motor_stop(void)
{
	motor_already_ordered=1;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	motor_already_ordered=0;
}


// *** Parties des fonctiosn finales qui seront utilisées dans le projet ***
void move_forwd_steps(int steps)
{
	motor_already_ordered=1;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(convert_cm_to_steps(default_speed_m)) ;
	right_motor_set_speed(convert_cm_to_steps(default_speed_m));
	while(left_motor_get_pos()<steps);
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	motor_already_ordered=0;

}
void move_forward(void)
{
	motor_already_ordered=1;
	left_motor_set_speed(convert_cm_to_steps(default_speed_m));
	right_motor_set_speed(convert_cm_to_steps(default_speed_m));

}


static THD_WORKING_AREA(conducteur_thd_wa,1024);
static THD_FUNCTION(conducteur_thd,arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);
	//systime_t time;
	while(1)
	{
		//time = chVTGetSystemTime();
		//etat=get_etat_marche()+get_instruction_micro();
		switch(etat)
		{
		/*
			case 0:   // etat initial ou on va faire marcher le moteur
				if(!motor_already_ordered)
				{
					move_forward();
					//etat=1; // on passe maintenant à l'état en marche
					set_etat_marche(1);

				}
				//chThdSleepUntilWindowed(time, time + MS2ST(1000));
				chThdSleepMilliseconds(100);
				break;
			case 1:
				//if(!motor_already_ordered)

					motor_stop();
					//etat=0;
					set_etat_marche(0); // pour dire qu'on s'est arreté
					//chThdSleepUntilWindowed(time, time + MS2ST(100));
					chThdSleepMilliseconds(100);
					break;

				//else break;

		*/
		case 0: // c'est cette partie d'initialisation on attend le signal de départ
			//si signal déclaré on passe à l'état 1
			if(get_instruction_micro()==1)
			{
				etat=1;
				chThdSleepMilliseconds(100);
				break;
			}
			else
			{
				motor_stop();
				chThdSleepMilliseconds(100);
				break;

			}

			//instruction_left_right=2; // pour right
			//chThdSleepMilliseconds(100);
			//break;
		case 1: //le signal de départ est donnée , on vérifie si on l'espace est libre
			/*
			if(!motor_already_ordered)
			{
				move_forward();
			}
			*/
			if(get_etat_marche()) //l'espace est libre
			{
				if(!motor_already_ordered)
				{
					move_forward();
					set_etat_micro(1);
					etat=2;
					chThdSleepMilliseconds(100);
					break;

				}
			}
			else
			{
				etat=3;
				set_etat_micro(2);
				delay(100000);
				chThdSleepMilliseconds(100);
				break;
			}
			//motor_stop();
			//set_etat_micro(2);
			//chThdSleepMilliseconds(100);
			//break;
		case 2: // le moteur a commencé à avancer c'est au capteur de detecter l'obstacle
			if(get_etat_marche())
			{
				chThdSleepMilliseconds(100);
				break;

			}
			else
			{
				motor_stop();
				delay(100000);
				etat=3;
				set_etat_micro(2);
				chThdSleepMilliseconds(100);
				break;

			}
		case 3: //le moteur s'est arreté devant un obstacle <--> c'est au microphone de donner l'ordre
			if(get_instruction_micro()==2)
			{
				if(!motor_already_ordered)
				{
					turn_left();
					move_forward();

					etat=2;
					set_etat_micro(1);
					chThdSleepMilliseconds(100);
					break;

				}
			}
			else if(get_instruction_micro()==3)
			{
				if(!motor_already_ordered)
				{
					turn_right();
					move_forward();
					etat=2;
					set_etat_micro(1);
					chThdSleepMilliseconds(100);
					break;
				}
			}
			chThdSleepMilliseconds(100);
			break;
		}
	}

}

void initialiser_conducteur(void)
{
	etat=0 ;
	set_etat_marche(0);
	set_etat_micro(0);
	motor_already_ordered=0;
	chThdCreateStatic(conducteur_thd_wa,sizeof(conducteur_thd_wa),NORMALPRIO,conducteur_thd,NULL);
}
