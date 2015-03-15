/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_H
#define STEPPER_H

#include "libs/Module.h"
#include <stdint.h>

class Block;
class StepperMotor;

class Stepper : public Module
{
public:
    Stepper();
    void on_module_loaded();
    void on_config_reload(void *argument);
    void on_block_begin(void *argument);
    void on_block_end(void *argument);
    void on_gcode_received(void *argument);
    void on_gcode_execute(void *argument);
    void on_play(void *argument);
    void on_pause(void *argument);
    void on_halt(void *argument);
    uint32_t main_interrupt(uint32_t dummy);
    void trapezoid_generator_reset();
    void set_step_events_per_second(float);
    void trapezoid_generator_tick(void);
    uint32_t stepper_motor_finished_move(uint32_t dummy);
    int config_step_timer( int cycles );
    void turn_enable_pins_on();
    void turn_enable_pins_off();

    const Block *get_current_block() const { return current_block; }

    // Get the acceleration-based step speed (steps per second) for a given stepper.
    // Computed based on the main stepper rate so that all steppers finish move at
    // the same time.
    uint32_t get_stepper_rate(uint32_t stepped, uint32_t steps_to_move, uint32_t old_rate);
    
    // Get the ratio between current speed and the nominal speed for this move.
    float get_speed_factor();
    
private:
    Block *current_block;
    StepperMotor *main_stepper;
    uint32_t previous_main_rate;
    uint32_t previous_main_pos;

    struct {
        bool enable_pins_status:1;
        bool paused:1;
        bool halted:1;
    };

};




#endif
