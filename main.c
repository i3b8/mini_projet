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

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void)
{


	chSysInit();
    halInit();
    mpu_init();
    //*** Inits Inter Process Communication bus ***
    initialiser_message_for_prox_ir();
    initialiser_capteur_ir();
    motors_init();
    initialiser_conducteur();

    //*** Init Peripherials ***
    proximity_start();
    mic_start(&processAudioData);
    initialiser_audio_proc();

    //***End Init Other Parameters***
    //***Motor configuration to move forward with speed of 6.5cm/s ***
    //*** End Motor Configuration ***
    //chThdSleepMilliseconds(4000);
    //*** IR Auto Calibration***
    calibrate_ir();
    //*** End IR Auto Calibration***
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
