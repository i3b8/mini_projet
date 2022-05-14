#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT

} BUFFER_NAME_t;
enum state_micro
{
	INITIAL=0,
	MOVING=1,
	STOPPED=2
};
enum state_instruction
{
	NO_INSTRUCTION=0,
	START_INSTRUCTION=1,
	TURN_LEFT_INSTRUCTION=2,
	TURN_RIGHT_INSTRUCTION=3,
	COME_BACK_INSTRUCTION=4
};


void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);
void set_etat_micro(unsigned int valeur);
unsigned int get_instruction_micro(void);
void set_instruction_to_do(unsigned int instruction);
void initialiser_audio_proc(void);

#endif /* AUDIO_PROCESSING_H */
