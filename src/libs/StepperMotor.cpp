/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "StepperMotor.h"

#include "Kernel.h"
#include "MRI_Hooks.h"
#include "StepTicker.h"

#include <math.h>

// in steps/sec the default minimum speed (was 20steps/sec hardcoded)
uint32_t StepperMotor::default_minimum_actuator_rate = 0;

// A StepperMotor represents an actual stepper motor. It is used to generate steps that move the actual motor at a given speed

StepperMotor::StepperMotor()
{
    init();
}

StepperMotor::StepperMotor(Pin &step, Pin &dir, Pin &en) : step_pin(step), dir_pin(dir), en_pin(en)
{
    init();
    enable(false);
    set_high_on_debug(en.port_number, en.pin);
}

StepperMotor::~StepperMotor()
{
}

void StepperMotor::init()
{
    // register this motor with the step ticker, and get its index in that array and bit position
    this->index= THEKERNEL->step_ticker->register_motor(this);
    this->moving = false;
    this->paused = false;
    this->stepped = 0;
    this->steps_to_move = 0;
    this->tickcount = 0;
    this->steps_per_second = 0;
    this->is_move_finished = true; // No move initially => same as finished
    
    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;
    
    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
    current_position_steps= 0;
    signal_step= 0;
}


// This is called ( see the .h file, we had to put a part of things there for obscure inline reasons ) when a step has to be generated
// we also here check if the move is finished etc ..
// This is in highest priority interrupt so cannot be pre-empted
void StepperMotor::step()
{
    ITM->PORT[this->index].u8 = 'S';
    
    // output to pins 37t
    this->step_pin.set( 1 );

    // we have moved a step 9t
    this->stepped++;

    // keep track of actuators actual position in steps
    this->current_position_steps += (this->direction ? -1 : 1);

    if (!this->is_move_finished)
    {
        // we may need to callback on a specific step, usually used to synchronize deceleration timer
        if(this->signal_step != 0 && this->stepped == this->signal_step) {
            THEKERNEL->step_ticker->synchronize_acceleration(true);
            this->signal_step= 0;
        }

        // Is this move finished ?
        if( this->stepped >= this->steps_to_move ) {
            // Mark it as finished, then StepTicker will call signal_mode_finished()
            // This is so we don't call that before all the steps have been generated for this tick()
            // The stepper will keep moving at current speed while the new block is being set up.
            // These extra steps will be counted in this->stepped and taken into account in next move.
            this->is_move_finished = true;
            THEKERNEL->step_ticker->a_move_finished= true;
            
            if (!this->keep_moving)
                this->moving = false;
        }
    }
}


// If the move is finished, the StepTicker will call this ( because we asked it to in tick() )
void StepperMotor::signal_move_finished()
{
    ITM->PORT[this->index].u8 = 'F';
    
    // signal it to whatever cares
    // in this call a new block may start, new moves set and new speeds
    this->end_hook->call();

    // We only need to do this if we were not instructed to move
    if( !this->moving ) {
        this->update_exit_tick();
    }
}

// This is just a way not to check for ( !this->moving || this->paused || this->fx_ticks_per_step == 0 ) at every tick()
void StepperMotor::update_exit_tick()
{
    if( !this->moving || this->paused || this->steps_to_move == 0 ) {
        // No more ticks will be recieved and no more events from StepTicker
        THEKERNEL->step_ticker->remove_motor_from_active_list(this);
        this->tickcount = 0;
    } else {
        // we will now get ticks and StepTIcker will send us events
        THEKERNEL->step_ticker->add_motor_to_active_list(this);
    }
}

// Instruct the StepperMotor to move a certain number of steps
StepperMotor* StepperMotor::move( bool direction, unsigned int steps, uint32_t initial_rate)
{
    // Disable irqs to prevent steps while we change values
    __disable_irq();
    
    ITM->PORT[this->index].u8 = 'M';
    
    // Take into account any predicted steps that were taken between previous move end and
    // the call of this function.
    if (this->is_move_finished && this->moving && this->stepped > this->steps_to_move)
    {
        uint32_t extra_steps = this->stepped - this->steps_to_move;
        this->stepped = 0;
    
        if (direction != this->direction)
        {
            // Direction has changed and the predicted steps overshot slightly.
            // Usually just 0-1 steps as speed is near zero at turning points.
            steps += extra_steps;
        }
        else if (steps < extra_steps)
        {
            // Shouldn't happen usually, predicted move overshot the actual move
            direction = !direction;
            steps = extra_steps - steps;
        }
        else
        {
            // This is the normal case, predicted move correctly
            this->stepped = extra_steps;
        }
    }
    else
    {
        this->stepped = 0;
    }
    
    // Initialize for new move
    this->dir_pin.set(direction);
    this->direction = direction;
    this->steps_to_move = steps;
    this->keep_moving = false;
    
    // Set initial rate for new move
    if (steps > stepped) {
        if (initial_rate > 0) set_rate(initial_rate);
        this->moving = true;
        this->is_move_finished = false;
    } else {
        this->moving = false;
        this->is_move_finished = true;
        THEKERNEL->step_ticker->a_move_finished = true;
    }
    
    this->update_exit_tick();
    
    // Movement may start now
    __enable_irq();
    
    return this;
}

// Set the speed at which this stepper moves in steps/sec.
StepperMotor* StepperMotor::set_rate( uint32_t rate )
{
    ITM->PORT[this->index].u8 = 'R';
    
    if(rate < default_minimum_actuator_rate) {
        rate = default_minimum_actuator_rate;
    }

    // How many steps we must output per second
    this->steps_per_second = rate;

    return this;
}

// Pause this stepper motor
void StepperMotor::pause()
{
    this->paused = true;
    this->update_exit_tick();
}

// Unpause this stepper motor
void StepperMotor::unpause()
{
    this->paused = false;
    this->update_exit_tick();
}


void StepperMotor::change_steps_per_mm(float new_steps)
{
    steps_per_mm = new_steps;
    last_milestone_steps = lround(last_milestone_mm * steps_per_mm);
    current_position_steps = last_milestone_steps;
}

void StepperMotor::change_last_milestone(float new_milestone)
{
    last_milestone_mm = new_milestone;
    last_milestone_steps = lround(last_milestone_mm * steps_per_mm);
    current_position_steps = last_milestone_steps;
}

int  StepperMotor::steps_to_target(float target)
{
    int target_steps = lround(target * steps_per_mm);
    return target_steps - last_milestone_steps;
}
