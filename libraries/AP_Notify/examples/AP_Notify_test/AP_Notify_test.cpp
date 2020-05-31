/*
 *       Example of AC_Notify library .
 *       DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_Notify/AP_BoardLED.h>        // Board LED library
#include <AP_Notify/Buzzer_PWM.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create board led object
AP_BoardLED board_led;
Buzzer_PWM buzzer_pwm;

void setup()
{
    hal.console->printf("AP_Notify library test\n");

    // initialise the board leds
    buzzer_pwm.init();
    hal.console->printf("Finished buzzer_pwm init\n");
    board_led.init();
    hal.console->printf("Finished LED init\n");

    // turn on initialising notification
    AP_Notify::flags.initialising = true;
    AP_Notify::flags.gps_status = 1;
    AP_Notify::flags.armed = 1;
    AP_Notify::flags.pre_arm_check = 1;
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
