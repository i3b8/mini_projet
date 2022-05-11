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

//#include <audio_processing.h>
//#include <fft.h>
//#include <communications.h>
//#include <arm_math.h>
//#include <conducteur.h>

//#include "capteur_ir.h"

//*** Define Part *** //
#define SEND_FROM_MIC

//*** End Define Part *** ///
//messagebus_t bus;
//MUTEX_DECL(bus_lock);
//CONDVAR_DECL(bus_condvar);


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
    //messagebus_init(&bus, &bus_lock, &bus_condvar);
    initialiser_capteur_ir();

    motors_init();

    initialiser_conducteur();

    //*** Init Peripherials ***


    proximity_start();
    mic_start(&processAudioData);
    initialiser_audio_proc();

    //***End Init Peripherials***

    //*** Init Other Parameters***
    //initialiser_capteur();
    //***End Init Other Parameters***
   // chThdSleepMilliseconds(2000);
    //***Motor configuration to move forward with speed of 6.5cm/s ***
    //move_forward();
    //*** End Motor Configuration ***

    //*** IR Auto Calibration***
    calibrate_ir();
    //*** End IR Auto Calibration***

    //***Checking the proximity working ***
    //messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    //proximity_msg_t proximity_values;

    while(1)
    {
    	//wait for new measures to be published
    	//messagebus_topic_wait(proximity_topic, &proximity_values, sizeof(proximity_values));
    	//update_capteur(proximity_values.delta[0]);
    	chThdSleepMilliseconds(500);

    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
