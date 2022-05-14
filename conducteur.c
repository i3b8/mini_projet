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
#include "audio_processing.h"
#include "conducteur.h"


#define MAX_NB_INSTRUCTION	50
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     13   //cm
#define NSTEP_ONE_TURN      1000
#define default_speed_m		6.5 // cm/s

static unsigned int etat;
uint8_t motor_already_ordered ;
//static uint8_t instruction_left_right =0;  // 0 pour nothing , 1 pour left , 2 pour right
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
	int32_t instruction_tab[MAX_NB_INSTRUCTION]={0};
	int num_instruction=-1;
	static int wayback=0;
	while(1)
	{
		switch(etat)
		{
		case 0: //Initial Situation : We need an Instruction Start to go to new state
			if(get_instruction_micro()==1) // a Start Signal has been Detected from the Micro
			{
				etat=1;
				set_instruction_to_do(0);
				chThdSleepMilliseconds(100);
				break;
			}
			else
			{
				motor_stop();
				set_instruction_to_do(0);
				chThdSleepMilliseconds(100);
				break;

			}
		case 1: //After Receiving the Signal Start, We need to verify by IR_Proximity sensor if road is clear or not before moving
			if(get_etat_marche()) // No obstacles detected we move to the state of Moving
			{
				if(!motor_already_ordered)
				{
					move_forward();
					set_etat_micro(1);
					set_instruction_to_do(0);
					etat=2;
					chThdSleepMilliseconds(100);
					break;

				}
				else
				{
					set_instruction_to_do(0);
					chThdSleepMilliseconds(100);
					break;
				}
			}
			else //Here the case that the Robot is initially put in front of an Obstacle, we move to the state"Waiting Instruction to turn"
			{
				etat=3;
				set_etat_micro(2);
				set_instruction_to_do(0);
				delay(1000000);
				chThdSleepMilliseconds(4000);
				break;
			}
		case 2:// This situation represent the "Moving Situation": the only thing we need to check is if the road is still clear or not

			if(!wayback){

					if(get_etat_marche()==1 )
					{
						chThdSleepMilliseconds(100);
						set_instruction_to_do(0);
						break;
					}
					else
					{
						num_instruction++;
						instruction_tab[num_instruction]=right_motor_get_pos();
						motor_stop();
						delay(1000000);
						etat=3;
						set_instruction_to_do(0);
						set_etat_micro(2);
						chThdSleepMilliseconds(4000);
						break;

					}
			}
			else{
				if(num_instruction != -1){
							if(instruction_tab[num_instruction]==-2){
								turn_right();
								num_instruction--;
								etat=2;

								break;
							}
							else if(instruction_tab[num_instruction]==-3){
								turn_left();
								num_instruction--;
								etat=2;

								break;
							}
							else{
								move_forwd_steps(instruction_tab[num_instruction]);
								num_instruction--;
								etat=2;

								break;
							}
						}else{
							motor_stop();
							break;
						}
			}


		case 3: //This situation : We are in front of an Obstacle: We wait for an instruction detected from the Micro (Left Right or Come Back)
			if(get_instruction_micro()==2)
			{
				if(!motor_already_ordered)
				{
					turn_left();
					num_instruction++;
					instruction_tab[num_instruction]=-2;
					move_forward();
					etat=2;
					set_etat_micro(1);
					set_instruction_to_do(0);
					chThdSleepMilliseconds(100);
					break;

				}
				else
				{
					set_instruction_to_do(0);
					chThdSleepMilliseconds(100);
					break;
				}
			}
			else if(get_instruction_micro()==3)
			{
				if(!motor_already_ordered)
				{
					turn_right();
					num_instruction++;
					instruction_tab[num_instruction]=-3;
					move_forward();
					etat=2;
					set_etat_micro(1);
					set_instruction_to_do(0);
					chThdSleepMilliseconds(100);
					break;
				}
				else
				{
					set_instruction_to_do(0);
					chThdSleepMilliseconds(100);
					break;

				}
			}
			else if(get_instruction_micro()==11)
			{
				if(!motor_already_ordered)
				{

					turn_right();
					turn_right();
					motor_stop();
					etat=2;
					set_etat_micro(1);
					set_instruction_to_do(0);
					wayback=1;
					chThdSleepMilliseconds(100);
					break;
				}
				else
				{
					chThdSleepMilliseconds(100);
					break;

				}
			}

		}
	}

}

void initialiser_conducteur(void)
{
	etat=0 ;
	set_etat_marche(0);
	set_etat_micro(0);
	set_instruction_to_do(0);
	motor_already_ordered=0;
	chThdCreateStatic(conducteur_thd_wa,sizeof(conducteur_thd_wa),NORMALPRIO,conducteur_thd,NULL);
}
