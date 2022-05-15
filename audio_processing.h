#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

enum state_micro // Signal Acquisition(With Specific Freq) depends on the stat micro
/*
 * If Initial <--> We look for start Instruction
 * If Moving <--> We don't look for any Instruction
 * If Stopped <---> We look for Specific Instrcution
 */
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
 * This will set the state_micro "etat_cours in .c file" in one of the specific states
 */
void set_etat_micro(unsigned int valeur);
/*
 * This returns the value of the  state_micro  "etat_cours in .c file"
 */
unsigned int get_instruction_micro(void);

/*
 * Create The Threads and Initialise the static variables
 */
void initialiser_audio_proc(void);

#endif /* AUDIO_PROCESSING_H */
