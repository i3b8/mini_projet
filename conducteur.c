
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include "ch.h"
#include "hal.h"
#include <main.h>

#include "motors.h"
#include "capteur_ir.h"
#include "audio_processing.h"
#include "conducteur.h"


#define MAX_NB_INSTRUCTION	50
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     12.85f   //cm
#define NSTEP_ONE_TURN      1000
#define DEFAULT_SPEED_M		6.5 // cm/s

static unsigned int etat;
// **************   INTERNAL FUNCTIONS ******************
// this function converts distance or speed from cm or cm/s to steps or steps/s

int convert_cm_to_steps(float cm)
{
	return cm* (NSTEP_ONE_TURN/WHEEL_PERIMETER ) ;
}
// ************ 	END INTERNAL Functions ***************

void turn_left(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(-convert_cm_to_steps(DEFAULT_SPEED_M));
	right_motor_set_speed(+convert_cm_to_steps(DEFAULT_SPEED_M));
	while(left_motor_get_pos()>convert_cm_to_steps(-PERIMETER_EPUCK/4));
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);

}
void turn_right(void)

{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
	right_motor_set_speed(-convert_cm_to_steps(DEFAULT_SPEED_M));
	while(left_motor_get_pos()<convert_cm_to_steps(PERIMETER_EPUCK/4));
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void motor_stop(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


// *** Parties des fonctiosn finales qui seront utilisées dans le projet ***
void move_forwd_steps(int steps)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M)) ;
	right_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
	while(left_motor_get_pos()<steps);
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);

}
void move_forward(void)
{
	left_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
	right_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
}


static THD_WORKING_AREA(conducteur_thd_wa,1024);
static THD_FUNCTION(conducteur_thd,arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);
	int32_t instruction_tab[MAX_NB_INSTRUCTION]={NO_INSTRUCTION};
	int num_instruction=-1; //This will contain the index of the last got instruction
	bool wayback=false;
	while(1)
	{
		switch(etat)
		{
		case WAIT_SIGNAL_START: //Initial Situation : We need an Instruction Start to go to new state
			if(get_instruction_micro()==START_INSTRUCTION) // a Start Signal has been Detected from the Micro
			{
				etat=WAIT_ROAD_CLEAR;
				chThdSleepMilliseconds(100);
				break;
			}
			else
			{
				motor_stop();
				chThdSleepMilliseconds(100);
				break;
			}
		case WAIT_ROAD_CLEAR: //After Receiving the Signal Start, We need to verify by IR_Proximity sensor if road is
			//clear or not before moving
			if(get_etat_marche()==CLEAR_ROAD) // No obstacles detected we move to the state of Moving
			{
				move_forward();
				set_etat_micro(MOVING);
				etat=CONDUCTOR_MOVING;
				chThdSleepMilliseconds(100);
				break;
			}
			else //Here the case that the Robot is initially put in front of an Obstacle,
				//we move to the state"Waiting Instruction to turn"
			{
				etat=CONDUCTOR_STOPPED;
				set_etat_micro(STOPPED);
				chThdSleepMilliseconds(3000);
				break;
			}
		case CONDUCTOR_MOVING:// This situation represent the "Moving Situation": the only thing we
			//need to check is if the road is still clear or not
			if(!wayback)
			{
				if(get_etat_marche()==CLEAR_ROAD )
				{
					chThdSleepMilliseconds(100);
					break;
				}
				else
				{
					num_instruction++; //We move to the next case in Tab-Instructions
					instruction_tab[num_instruction]=right_motor_get_pos(); // We get directly the nbr of steps just before stopping
					motor_stop();
					etat=CONDUCTOR_STOPPED;
					set_etat_micro(STOPPED);
					chThdSleepMilliseconds(3000);
					break;
				}
			}
			else
			{
				if(num_instruction != -1)
				{
					if(instruction_tab[num_instruction]==TURN_RIGHT_INSTRUCTION)
					{
						turn_right();
						num_instruction--;
						break;
					}
					else if(instruction_tab[num_instruction]==TURN_LEFT_INSTRUCTION)
					{
						turn_left();
						num_instruction--;
						break;
					}
					else
					{
						move_forwd_steps(instruction_tab[num_instruction]);
						num_instruction--;
						break;
					}
				}
				else
				{
					motor_stop();
					break;
				}
			}
		case 3: //This situation : We are in front of an Obstacle: We wait for an
			//instruction detected from the Micro (Left Right or Come Back)
			if(get_instruction_micro()==TURN_LEFT_INSTRUCTION)
			{
					turn_left();
					num_instruction++;
					instruction_tab[num_instruction]=TURN_RIGHT_INSTRUCTION; // We invert the movement for the coming back
					move_forward();
					etat=CONDUCTOR_MOVING;
					set_etat_micro(MOVING);
					chThdSleepMilliseconds(100);
					break;
			}
			else if(get_instruction_micro()==TURN_RIGHT_INSTRUCTION)
			{
					turn_right();
					num_instruction++;
					instruction_tab[num_instruction]=TURN_LEFT_INSTRUCTION; //We invert the movement for the coming back
					move_forward();
					etat=CONDUCTOR_MOVING;
					set_etat_micro(MOVING);
					chThdSleepMilliseconds(100);
					break;
			}
			else if(get_instruction_micro()==COME_BACK_INSTRUCTION)
			{
					turn_right();
					turn_right();
					motor_stop();
					etat=CONDUCTOR_MOVING;
					set_etat_micro(MOVING);
					wayback=true;
					chThdSleepMilliseconds(100);
					break;
			}
		}
	}

}

void initialiser_conducteur(void)
{
	etat=WAIT_SIGNAL_START ;
	set_etat_marche(OBSTACLE);
	set_etat_micro(INITIAL);
	chThdCreateStatic(conducteur_thd_wa,sizeof(conducteur_thd_wa),NORMALPRIO,conducteur_thd,NULL);
}
