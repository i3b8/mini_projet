#include "ch.h"
#include "hal.h"
#include "audio/microphone.h"
#include "sensors/proximity.h"
#include "memory_protection.h"
#include "motors.h"
#include "conducteur.h"
#include "capteur_ir.h"
#include "audio_processing.h"

int main(void)
{


	chSysInit();
    halInit();
    mpu_init();
    //*** Inits Inter Process Communication bus ***
    initialiser_message_for_prox_ir();
    motors_init();
    proximity_start();
    calibrate_ir();
    initialiser_capteur_ir();
    initialiser_audio_proc();
    initialiser_conducteur();
    mic_start(&processAudioData);
    //Initialisation Threads //

    //*** End Init Threads ***
    while(1)
    {
    	chThdSleepMilliseconds(500);
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

