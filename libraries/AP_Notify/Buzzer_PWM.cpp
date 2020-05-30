/*
  Buzzer driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Buzzer_PWM.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_Notify.h"

#ifndef HAL_BUZZER_ON
    #define HAL_BUZZER_ON 1
    #define HAL_BUZZER_OFF 0 
#endif

extern const AP_HAL::HAL& hal;


bool Buzzer_PWM::init()
{
    if (pNotify->buzzer_enabled() == false) {
        return false;
    }
#if !defined(HAL_BUZZER_PWM_CHIP) || !defined(HAL_BUZZER_PWM_CHANNEL) || !defined(HAL_BUZZER_PWM_FREQ_HZ)
    return false;
#else
    _chip = HAL_BUZZER_PWM_CHIP;
    _channel = HAL_BUZZER_PWM_CHANNEL;
    _freq_hz = HAL_BUZZER_PWM_FREQ_HZ;
#endif

    _pwm = NULL;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
    _pwm = new Linux::PWM_Sysfs_Pocket(_chip, _channel);
#endif
    if (_pwm == NULL) return false;

    _pwm->set_freq(_freq_hz);
    _pwm->set_duty_cycle(hz_to_nsec(_freq_hz)/2);
    // ensure it's off
    on(false);

    // set initial boot states. This prevents us issuing a arming
    // warning in plane and rover on every boot
    _flags.armed = AP_Notify::flags.armed;
    _flags.failsafe_battery = AP_Notify::flags.failsafe_battery;
    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void Buzzer_PWM::update()
{
    update_pattern_to_play();
    update_playing_pattern();
}

void Buzzer_PWM::update_pattern_to_play()
{
    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        // arming failed buzz
        play_pattern(SINGLE_BUZZ);
        return;
    }

    if (AP_HAL::millis() - _pattern_start_time < _pattern_start_interval_time_ms) {
        // do not interrupt playing patterns / enforce minumum separation
        return;
    }

    // check if armed status has changed
    if (_flags.armed != AP_Notify::flags.armed) {
        _flags.armed = AP_Notify::flags.armed;
        if (_flags.armed) {
            // double buzz when armed
            play_pattern(ARMING_BUZZ);
        }else{
            // single buzz when disarmed
            play_pattern(SINGLE_BUZZ);
        }
        return;
    }

    // check ekf bad
    if (_flags.ekf_bad != AP_Notify::flags.ekf_bad) {
        _flags.ekf_bad = AP_Notify::flags.ekf_bad;
        if (_flags.ekf_bad) {
            // ekf bad warning buzz
            play_pattern(EKF_BAD);
        }
        return;
    }

    // if vehicle lost was enabled, starting beep
    if (AP_Notify::flags.vehicle_lost) {
        play_pattern(DOUBLE_BUZZ);
        return;
    }

    // if battery failsafe constantly single buzz
    if (AP_Notify::flags.failsafe_battery) {
        play_pattern(SINGLE_BUZZ);
        return;
    }
}


void Buzzer_PWM::update_playing_pattern()
{
    if (_pattern == 0UL) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const uint32_t delta = now - _pattern_start_time;
    if (delta >= 3200) {
        // finished playing pattern
        on(false);
        _pattern = 0UL;
        return;
    }
    const uint32_t bit = delta / 100UL; // each bit is 100ms
    on(_pattern & (1U<<(31-bit)));
}

// on - turns the buzzer on or off
void Buzzer_PWM::on(bool turn_on)
{
    // return immediately if nothing to do
    if (_flags.on == turn_on) {
        return;
    }

    // update state
    _flags.on = turn_on;

    // enable or disable tone
    _pwm->enable(_flags.on? HAL_BUZZER_ON : HAL_BUZZER_OFF);
}

/// play_pattern - plays the defined buzzer pattern
void Buzzer_PWM::play_pattern(const uint32_t pattern)
{
    _pattern = pattern;
    _pattern_start_time = AP_HAL::millis();
}

