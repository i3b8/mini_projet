#include <arm_math.h>
#include "audio/microphone.h"
#include "fft.h"
#include "audio_processing.h"

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
#define MIN_VALUE_THRESHOLD	15000

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_START	    16	//250Hz
#define FREQ_LEFT		26	//406Hz
#define FREQ_RIGHT		19	//300HZ
#define FREQ_BACK		23	//359Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_START_L	(FREQ_START-1)
#define FREQ_START_H	(FREQ_START+1)
#define FREQ_LEFT_L		(FREQ_LEFT-1)
#define FREQ_LEFT_H		(FREQ_LEFT+1)
#define FREQ_RIGHT_L	(FREQ_RIGHT-1)
#define FREQ_RIGHT_H	(FREQ_RIGHT+1)
#define FREQ_BACK_L		(FREQ_BACK-1)
#define FREQ_BACK_H		(FREQ_BACK+1)

// Other parameters

int etat_cours ; // This is a state micro being either:"Initial, Moving or Stopped"
int instruction_to_do; // This is a state_instruction being etither:"No_instr,Start,Turn Left..."

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 
	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	switch(etat_cours)
	{
		case INITIAL: //Initial Situation --> Wait for the Start Signal : FREQ_FORWARD
		if(max_norm_index >= FREQ_START_L && max_norm_index <= FREQ_START_H )
		{
			instruction_to_do= START_INSTRUCTION;
			break;
		}
		else
		{
			instruction_to_do= NO_INSTRUCTION;
			break;
		}
		case MOVING: //Moving Situation : No signal need to be detected
			instruction_to_do= NO_INSTRUCTION;
			break;
		case STOPPED: // Stop Situation : Need to detect a specific signal (Left-Right or Come_back)
			if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H)
			{
				instruction_to_do=TURN_LEFT_INSTRUCTION;
				break;
			}
			else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H)
			{
				instruction_to_do=TURN_RIGHT_INSTRUCTION; //turn_right
				break;

			}
			else if(max_norm_index >= FREQ_BACK_L && max_norm_index <= FREQ_BACK_H)
			{
				instruction_to_do=COME_BACK_INSTRUCTION;
				break;
			}
			else
			{
				instruction_to_do=NO_INSTRUCTION;
				break;
			}
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :*/
	//int16_t *data;
	//Buffer containing 4 times 160 samples. the samples are sorted by micro
	//so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
	//uint16_t num_samples;
	//Tells how many data we get in total (should always be 640)

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	//static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part

		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		nb_samples++;
		micLeft_cmplx_input[nb_samples] = 0;
		nb_samples++;
		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		nb_samples = 0;
		sound_remote(micLeft_output);
	}
}

void set_etat_micro(unsigned int valeur)
{
	etat_cours=valeur;
}
unsigned int get_instruction_micro(void)
{
	return instruction_to_do;
}
void initialiser_audio_proc(void)
{
	instruction_to_do= NO_INSTRUCTION ;
}

