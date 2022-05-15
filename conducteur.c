#include "motors.h"
#include "capteur_ir.h"
#include "audio_processing.h"
#include "conducteur.h"
#include "leds.h"

#define MAX_NB_INSTRUCTION	50
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     12.84f   //cm
#define NSTEP_ONE_TURN      1000
#define DEFAULT_SPEED_M		6.5 // cm/s

unsigned int etat;
// **************   INTERNAL FUNCTIONS ******************
// this function converts distance or speed from cm or cm/s to steps or steps/s

int convert_cm_to_steps(float cm)
{
	return cm* (NSTEP_ONE_TURN/WHEEL_PERIMETER ) ;
}

//Make a Quarter Tour to the Left
void turn_left(void)
{
	set_led(LED6,1);
	set_led(LED7,1);
	set_led(LED8,1);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(-convert_cm_to_steps(DEFAULT_SPEED_M));
	right_motor_set_speed(+convert_cm_to_steps(DEFAULT_SPEED_M));
	while(left_motor_get_pos()>convert_cm_to_steps(-PERIMETER_EPUCK/4));
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	set_led(LED6,0);
	set_led(LED7,0);
	set_led(LED8,0);

}
//Make a Quarter Tour to the right
void turn_right(void)

{
	set_led(LED3,1);
	set_led(LED4,1);
	set_led(LED5,1);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
	right_motor_set_speed(-convert_cm_to_steps(DEFAULT_SPEED_M));
	while(left_motor_get_pos()<convert_cm_to_steps(PERIMETER_EPUCK/4));
	left_motor_set_speed(0) ;
	right_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	set_led(LED3,0);
	set_led(LED4,0);
	set_led(LED5,0);
}
//Stop the Motor
void motor_stop(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
//Move forward for x steps --> This Function is Used in the Come_Back situation
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
// Move forward (**infinity steps**)
void move_forward(void)
{
	left_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
	right_motor_set_speed(convert_cm_to_steps(DEFAULT_SPEED_M));
}

// ************ 	END INTERNAL Functions ***************

//*** Thread of The Conductor *** ///
static THD_WORKING_AREA(conducteur_thd_wa,1024);
static THD_FUNCTION(conducteur_thd,arg)
{
	(void)arg;
	chRegSetThreadName(__FUNCTION__);
	int32_t instruction_tab[MAX_NB_INSTRUCTION]={NO_INSTRUCTION};
	int num_instruction=-1; //This will contain the index of the last got instruction
	while(1)
	{
		switch(etat)
		{
		case WAIT_SIGNAL_START: //Initial Situation : We need an Instruction Start to go to new state
			if(get_instruction_micro()==START_INSTRUCTION) // a Start Signal has been Detected from the Micro
			{
				set_front_led(0);
				move_forward();
				set_etat_micro(MOVING);
				etat=CONDUCTOR_MOVING;

			}
			chThdSleepMilliseconds(100);
			break;
		case CONDUCTOR_MOVING:// This situation represent the "Moving Situation": the only thing we
			//need to check is if the road is still clear or not
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
		case CONDUCTOR_STOPPED: //This situation : We are in front of an Obstacle: We wait for an
			//instruction detected from the Micro (Left Right or Come Back)
			if(get_instruction_micro()==TURN_LEFT_INSTRUCTION)
			{

					turn_left();
					num_instruction++;
					instruction_tab[num_instruction]=TURN_RIGHT_INSTRUCTION; // We invert the movement for the coming back
					move_forward();
					etat=CONDUCTOR_MOVING;
					set_etat_micro(MOVING);
			}
			else if(get_instruction_micro()==TURN_RIGHT_INSTRUCTION)
			{

					turn_right();
					num_instruction++;
					instruction_tab[num_instruction]=TURN_LEFT_INSTRUCTION; //We invert the movement for the coming back
					move_forward();
					etat=CONDUCTOR_MOVING;
					set_etat_micro(MOVING);
			}
			else if(get_instruction_micro()==COME_BACK_INSTRUCTION)
			{
					set_body_led(1);
					turn_right();
					turn_right();
					motor_stop();
					set_body_led(0);
					etat=CONDUCTOR_COMING_BACK;
					set_etat_micro(MOVING);
			}
			chThdSleepMilliseconds(100);
			break;
		case CONDUCTOR_COMING_BACK:
			if(num_instruction != -1)
			{
				if(instruction_tab[num_instruction]==TURN_RIGHT_INSTRUCTION)
				{
					turn_right();
					instruction_tab[num_instruction]=NO_INSTRUCTION;
					num_instruction--;
				}
				else if(instruction_tab[num_instruction]==TURN_LEFT_INSTRUCTION)
				{
					turn_left();
					instruction_tab[num_instruction]=NO_INSTRUCTION;
					num_instruction--;
				}
				else
				{
					move_forwd_steps(instruction_tab[num_instruction]);
					instruction_tab[num_instruction]=NO_INSTRUCTION;
					num_instruction--;
				}
			}
			else
			{
				motor_stop();
				set_front_led(1);
				turn_left();
				turn_left();
				etat=WAIT_SIGNAL_START;
				set_etat_micro(INITIAL);
				chThdSleepMilliseconds(6000);
			}
			break;

		}
	}
}



void initialiser_conducteur(void)
{
	etat=WAIT_SIGNAL_START ;
	set_etat_micro(INITIAL);
	set_front_led(1);
	chThdCreateStatic(conducteur_thd_wa,sizeof(conducteur_thd_wa),NORMALPRIO,conducteur_thd,NULL);
}
