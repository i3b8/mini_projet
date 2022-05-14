#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "audio/microphone.h"
#include "sensors/proximity.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"

#include "usbcfg.h"
#include "communication.h"

#include "conducteur.h"
#include "capteur_ir.h"
#include "fft.h"
#include "audio_processing.h"

//*** Define Part *** //
#define SEND_FROM_MIC

//*** End Define Part *** ///

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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
    mic_start(&processAudioData);
    //Initialisation Threads //
    initialiser_capteur_ir();
    initialiser_audio_proc();
    initialiser_conducteur();
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
