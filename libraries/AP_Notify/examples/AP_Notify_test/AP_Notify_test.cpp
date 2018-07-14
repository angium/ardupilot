/*
 *       Example of AC_Notify library .
 *       DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_Notify/AP_BoardLED.h>        // Board LED library

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_Notify notify;

// create board led object
//AP_BoardLED board_led;

void setup()
{
    hal.console->printf("AP_Notify library test\n");
	notify.init(true);
    // initialise the board leds
 //   board_led.init();

    // turn on initialising notification
 //   AP_Notify::flags.initialising = true;
 //   AP_Notify::flags.gps_status = 1;
 //   AP_Notify::flags.armed = 1;
 //   AP_Notify::flags.pre_arm_check = 1;
}

void loop()
{
	hal.console->printf("initialising = %d",AP_Notify::flags.initialising);
	notify.update();
	hal.scheduler->delay(20);
}

AP_HAL_MAIN();
