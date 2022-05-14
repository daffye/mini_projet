#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
//#include "spi_comm.h"
#include "leds.h"

#include <pi_regulator.h>
#include <process_image.h>
//#include <orientation.h>
#include <mouvement.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


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

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
      messagebus_init(&bus, &bus_lock, &bus_condvar);

    clear_leds();
    set_body_led(0);
    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //spi_comm_start();    //voir eventuellement les reinclude si jamais ca bug avec les periphs
    //starts the camera
    dcmi_start();
	po8030_start();
	po8030_set_awb(0);
	//inits the motors
	motors_init();
	//start proximity sensors
	proximity_start();
	VL53L0X_start();

	//wait until the peripherals have been correctly initialized
	chThdSleepMilliseconds(500);

	//stars the threads for the pi regulator and the processing of the image
	process_image_start();
	pi_regulator_start();
	avoid_start();

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
