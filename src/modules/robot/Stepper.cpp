/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Stepper.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Planner.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "checksumm.h"
#include "SlowTicker.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Block.h"
#include "StepTicker.h"

#include <vector>
using namespace std;

#include "libs/nuts_bolts.h"
#include "libs/Hook.h"

#include <mri.h>

// The stepper reacts to blocks that have XYZ movement to transform them into actual stepper motor moves

Stepper::Stepper()
{
    this->current_block = NULL;
    this->paused = false;
    this->halted= false;
}

//Called when the module has just been loaded
void Stepper::on_module_loaded()
{
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_HALT);

    // Get onfiguration
    this->on_config_reload(this);

    // Acceleration ticker
    THEKERNEL->step_ticker->register_acceleration_tick_handler([this](){trapezoid_generator_tick(); });

    // Attach to the end_of_move stepper event
    THEKERNEL->robot->alpha_stepper_motor->attach(this, &Stepper::stepper_motor_finished_move );
    THEKERNEL->robot->beta_stepper_motor->attach( this, &Stepper::stepper_motor_finished_move );
    THEKERNEL->robot->gamma_stepper_motor->attach(this, &Stepper::stepper_motor_finished_move );
}

// Get configuration from the config file
void Stepper::on_config_reload(void *argument)
{
    // Steppers start off by default
    this->turn_enable_pins_off();
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Stepper::on_pause(void *argument)
{
    this->paused = true;
    THEKERNEL->robot->alpha_stepper_motor->pause();
    THEKERNEL->robot->beta_stepper_motor->pause();
    THEKERNEL->robot->gamma_stepper_motor->pause();
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Stepper::on_play(void *argument)
{
    // TODO: Re-compute the whole queue for a cold-start
    this->paused = false;
    THEKERNEL->robot->alpha_stepper_motor->unpause();
    THEKERNEL->robot->beta_stepper_motor->unpause();
    THEKERNEL->robot->gamma_stepper_motor->unpause();
}

void Stepper::on_halt(void *argument)
{
    if(argument == nullptr) {
        this->turn_enable_pins_off();
        this->halted= true;
    }else{
        this->halted= false;
    }
}

void Stepper::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    // Attach gcodes to the last block for on_gcode_execute
    if( gcode->has_m && (gcode->m == 84 || gcode->m == 17 || gcode->m == 18 )) {
        THEKERNEL->conveyor->append_gcode(gcode);
    }
}

// React to enable/disable gcodes
void Stepper::on_gcode_execute(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        if( gcode->m == 17 ) {
            this->turn_enable_pins_on();
        }
        if( (gcode->m == 84 || gcode->m == 18) && !gcode->has_letter('E') ) {
            this->turn_enable_pins_off();
        }
    }
}

// Enable steppers
void Stepper::turn_enable_pins_on()
{
    for (StepperMotor *m : THEKERNEL->robot->actuators)
        m->enable(true);
    this->enable_pins_status = true;
}

// Disable steppers
void Stepper::turn_enable_pins_off()
{
    for (StepperMotor *m : THEKERNEL->robot->actuators)
        m->enable(false);
    this->enable_pins_status = false;
}

// A new block is popped from the queue
void Stepper::on_block_begin(void *argument)
{
    Block *block  = static_cast<Block *>(argument);

    // Mark the new block as of interrest to us, handle blocks that have no axis moves properly (like Extrude blocks etc)
    if(block->millimeters > 0.0F && (block->steps[ALPHA_STEPPER] > 0 || block->steps[BETA_STEPPER] > 0 || block->steps[GAMMA_STEPPER] > 0) ) {
        block->take();
    } else {
        THEKERNEL->robot->alpha_stepper_motor->move(0, 0);
        THEKERNEL->robot->beta_stepper_motor->move(0, 0);
        THEKERNEL->robot->gamma_stepper_motor->move(0, 0);
        return;
    }

    // We can't move with the enable pins off
    if( this->enable_pins_status == false ) {
        this->turn_enable_pins_on();
    }
    
    // If the block end speed is more larger than acceleration delta, keep moving
    // between blocks to avoid jerks.
    bool keep_moving = (block->final_rate > block->rate_delta);
    
    // Setup : instruct stepper motors to move
    // Find the stepper with the more steps, it's the one the speed calculations will want to follow
    this->main_stepper= nullptr;
    if( block->steps[ALPHA_STEPPER] > 0 ) {
        THEKERNEL->robot->alpha_stepper_motor->move( block->direction_bits[ALPHA_STEPPER], block->steps[ALPHA_STEPPER]);
        THEKERNEL->robot->alpha_stepper_motor->set_keep_moving(keep_moving);
        this->main_stepper = THEKERNEL->robot->alpha_stepper_motor;
    }
    else
    {
        THEKERNEL->robot->alpha_stepper_motor->move(0, 0);
    }

    if( block->steps[BETA_STEPPER ] > 0 ) {
        THEKERNEL->robot->beta_stepper_motor->move(  block->direction_bits[BETA_STEPPER], block->steps[BETA_STEPPER ]);
        THEKERNEL->robot->beta_stepper_motor->set_keep_moving(keep_moving);
        if(this->main_stepper == nullptr || THEKERNEL->robot->beta_stepper_motor->get_steps_to_move() > this->main_stepper->get_steps_to_move())
            this->main_stepper = THEKERNEL->robot->beta_stepper_motor;
    }
    else
    {
        THEKERNEL->robot->beta_stepper_motor->move(0, 0);
    }

    if( block->steps[GAMMA_STEPPER] > 0 ) {
        THEKERNEL->robot->gamma_stepper_motor->move( block->direction_bits[GAMMA_STEPPER], block->steps[GAMMA_STEPPER]);
        THEKERNEL->robot->gamma_stepper_motor->set_keep_moving(keep_moving);
        if(this->main_stepper == nullptr || THEKERNEL->robot->gamma_stepper_motor->get_steps_to_move() > this->main_stepper->get_steps_to_move())
            this->main_stepper = THEKERNEL->robot->gamma_stepper_motor;
    }
    else
    {
        THEKERNEL->robot->gamma_stepper_motor->move(0, 0);
    }

    this->current_block = block;

    // Setup acceleration for this block
    this->trapezoid_generator_reset();

    // Set the initial speed for this move
    this->trapezoid_generator_tick();

    // synchronize the acceleration timer with the start of the new block so it does not drift and randomly fire during the block
    THEKERNEL->step_ticker->synchronize_acceleration(false);

    // set a flag to synchronize the acceleration timer with the deceleration step, and fire it immediately we get to that step
    if( block->decelerate_after > 0 && block->decelerate_after+1 < this->main_stepper->steps_to_move ) {
        this->main_stepper->signal_step= block->decelerate_after+1; // we make it +1 as deceleration does not start until steps > decelerate_after
    }
}

// Current block is discarded
void Stepper::on_block_end(void *argument)
{
    this->current_block = NULL; //stfu !
}

// When a stepper motor has finished it's assigned movement
uint32_t Stepper::stepper_motor_finished_move(uint32_t dummy)
{
    // We care only if all motors are finished
    if (!(THEKERNEL->robot->alpha_stepper_motor->is_move_finished && THEKERNEL->robot->beta_stepper_motor->is_move_finished && THEKERNEL->robot->gamma_stepper_motor->is_move_finished)) {
        return 0;
    }

    // This block is finished, release it
    if( this->current_block != NULL ) {
        this->current_block->release();
    }
    
    return 0;
}

static uint32_t saturating_sub(uint32_t x, uint32_t y)
{
    if (x > y)
        return x - y;
    else
        return 0;
}

uint32_t Stepper::get_stepper_rate(uint32_t stepped, uint32_t steps_to_move, uint32_t old_rate)
{
    if (this->previous_main_pos >= this->main_stepper->steps_to_move)
    {
        // Main stepper has already finished, hopefully we have only 1 step left.
        return old_rate;
    }
    
    if (stepped > steps_to_move)
    {
        // Already finished our move, keep current rate to avoid jerks.
        return old_rate;
    }
    
    // Compute the time (seconds) the main stepper would take to finish at current rate:
    //   main_time = (main_steps_to_move - previous_main_pos) / previous_main_rate
    // Then compute our speed so that we finish at the same time:
    //   my_rate = (steps_to_move - stepped) / main_time
    // Converting to integer-friendly math:
    //   my_rate = (steps_to_move - stepped) * previous_main_rate / (main_steps_to_move - previous_main_pos)
    uint64_t rate = (uint64_t)(steps_to_move - stepped) * this->previous_main_rate;
    uint32_t divider = this->main_stepper->steps_to_move - this->previous_main_pos;
    rate = (rate + divider / 2) / divider;
    return rate;
}

float Stepper::get_speed_factor()
{
    return ((float)previous_main_rate) / current_block->nominal_rate;
}

// Calculate step rate at position x, when it should be v1 at x1 and v2 at x2.
// V as function of X follows a sqrt() shaped curve, so by squaring v1 and v2,
// we can use linear interpolation.
static int quadratic_interpolate(float x, float x1, float v1, float x2, float v2)
{
    if (x <= x1)
        return v1;
    else if (x >= x2)
        return v2;
    
    float y1 = v1 * v1;
    float y2 = v2 * v2;
    float y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
    return sqrtf(y);
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
void Stepper::trapezoid_generator_tick(void)
{
    // Do not do the accel math for nothing
    if(this->current_block && !this->paused && this->main_stepper->moving )
    {
        // Calculate what the main stepper speed should be (in steps per second).
        // All other motors follow the rate of the main stepper.
        uint32_t main_rate = this->previous_main_rate;
        uint32_t min_rate = current_block->rate_delta / 2;
        uint32_t current_pos = this->main_stepper->stepped;
        
        if (THEKERNEL->conveyor->is_flushing())
        {
            // Abort in progress, slow down and stop.
            if (main_rate > min_rate)
            {
                main_rate = saturating_sub(main_rate, current_block->rate_delta);
            }
            else
            {
                for (auto i : THEKERNEL->robot->actuators) i->move(i->direction, 0); // stop motors
                current_block->release();
                current_block = NULL;
                THEKERNEL->call_event(ON_SPEED_CHANGE, 0); // tell others we stopped
                return;
            }
        }
        else if (current_pos >= this->current_block->steps_event_count)
        {
            // Block is changing now, decelerate until the new move activates.
            main_rate = saturating_sub(main_rate, current_block->rate_delta);
        }
        else if (current_pos < this->current_block->accelerate_until)
        {
            // Beginning of move, accelerate
            uint32_t initial_rate = std::max((uint32_t)current_block->initial_rate, min_rate);
            main_rate = quadratic_interpolate(current_pos, 0, initial_rate,
                                             this->current_block->accelerate_until, current_block->max_rate);
        }
        else if (current_pos >= this->current_block->decelerate_after)
        {
            // End of move, decelerate.
            // Calculate desired speed at the end of the next acceleration interval.
            uint32_t end_pos = current_pos + main_rate / THEKERNEL->acceleration_ticks_per_second;
            uint32_t final_rate = std::max((uint32_t)current_block->final_rate, min_rate);
            main_rate = quadratic_interpolate(end_pos, this->current_block->decelerate_after, current_block->max_rate,
                                              this->current_block->steps_event_count, final_rate);
        }
        else
        {
            // Middle of move, cruise at specified speed
            main_rate = current_block->nominal_rate;
        }
        
        // Never decelerate fully to 0. Because acceleration tick happens separately, we may still
        // have a few steps to make before the move is finished. It should be safe to stop immediately
        // from min_rate to 0 when the move ends.
        if (main_rate < min_rate)
            main_rate = min_rate;
        
        this->previous_main_rate = main_rate;
        this->previous_main_pos = current_pos;
        
        this->main_stepper->set_rate(main_rate);
        
        // Now calculate the rates for all other steppers based on the main stepper
        // Note: this has to be recalculated even if speed didn't change, so that accumulating 
        // rounding errors can be eliminated.
        StepperMotor *alpha = THEKERNEL->robot->alpha_stepper_motor;
        StepperMotor *beta  = THEKERNEL->robot->beta_stepper_motor;
        StepperMotor *gamma = THEKERNEL->robot->gamma_stepper_motor;
        if (alpha->moving && this->main_stepper != alpha)
        {
            alpha->set_rate(get_stepper_rate(alpha->stepped, alpha->steps_to_move, alpha->get_rate()));
        }
        if (beta->moving && this->main_stepper != beta)
        {
            beta->set_rate(get_stepper_rate(beta->stepped, beta->steps_to_move, beta->get_rate()));
        }
        if (gamma->moving && this->main_stepper != gamma)
        {
            gamma->set_rate(get_stepper_rate(gamma->stepped, gamma->steps_to_move, gamma->get_rate()));
        }
        
        // Other modules might want to know the speed changed
        THEKERNEL->call_event(ON_SPEED_CHANGE, this);
    }
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
inline void Stepper::trapezoid_generator_reset()
{
    this->previous_main_rate = this->current_block->initial_rate;
    this->previous_main_pos = 0;
}


