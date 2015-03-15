/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include "libs/Hook.h"
#include "Pin.h"
#include <atomic>
#include <functional>
#include "Kernel.h"
#include "StepTicker.h"

class StepTicker;
class Hook;

class StepperMotor {
    public:
        StepperMotor();
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        void step();
        inline void unstep() { step_pin.set(0); };

        inline void enable(bool state) { en_pin.set(!state); };

        bool is_moving() { return moving; }
        void move_finished();
        StepperMotor* move( bool direction, unsigned int steps, uint32_t initial_rate = 0);
        void signal_move_finished();
        void update_exit_tick();
        void pause();
        void unpause();

        uint32_t get_steps_per_second()  const { return steps_per_second; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        void change_last_milestone(float);
        float get_last_milestone(void) const { return last_milestone_mm; }
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }
        float get_max_rate(void) const { return max_rate; }
        void set_max_rate(float mr) { max_rate= mr; }
        void set_keep_moving(bool keep_moving) { this->keep_moving = keep_moving; }

        int  steps_to_target(float);
        uint32_t get_steps_to_move() const { return steps_to_move; }
        uint32_t get_stepped() const { return stepped; }

        StepperMotor* set_rate( uint32_t rate );
        inline uint32_t get_rate() const { return steps_per_second; }
        
        template<typename T> void attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            Hook* hook = new Hook();
            hook->attach(optr, fptr);
            this->end_hook = hook;
        }

        friend class StepTicker;
        friend class Stepper;
        friend class Planner;
        friend class Robot;

    private:
        void init();

        int index;
        Hook* end_hook;

        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        uint32_t steps_per_second;
        float steps_per_mm;
        float max_rate; // this is not really rate it is in mm/sec, misnamed used in Robot and Extruder
        static uint32_t default_minimum_actuator_rate;

        volatile int32_t current_position_steps;
        int32_t last_milestone_steps;
        float   last_milestone_mm;

        uint32_t steps_to_move;
        uint32_t stepped;
        uint32_t signal_step;

        uint32_t tickcount;
        
        struct {
            bool direction:1;
            volatile bool is_move_finished:1; // Whether the move just finished
            bool paused:1;
            volatile bool moving:1;
            volatile bool keep_moving:1;
        };

        // Called a great many times per second, to step if we have to now
        inline bool tick(uint32_t frequency) {
            tickcount += steps_per_second;
            
            if (tickcount > frequency)
            {
                tickcount -= frequency;
                step();
                return true;
            }
            
            return false;
        };
};

#endif

