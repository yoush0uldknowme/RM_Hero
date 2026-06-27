//
// Created by xhuanc on 2021/11/24.
//

#include "ramp.h"
#include "math.h"

void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

    if(input>0)
    {
        if (ramp_source_type->out > input)
            ramp_source_type->out = input;
        else if  (ramp_source_type->out < -input)
            ramp_source_type->out = -input;

    }
    else
    {
        if (ramp_source_type->out < input)
            ramp_source_type->out = input;
        else if  (ramp_source_type->out > -input)
            ramp_source_type->out = -input;

    }

    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void ramp_speed_init(ramp_function_source_speed_t *ramp_source_type, fp32 increase_rate, fp32 max, fp32 min, fp32 decrease_rate)
{
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
    ramp_source_type->decrease_rate = decrease_rate;
    ramp_source_type->increase_rate = increase_rate;
}

void ramp_speed_calc(ramp_function_source_speed_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;

    if(input > 0)
    {
        if (fabsf(ramp_source_type->input) > ramp_source_type->out)
        {
            ramp_source_type->out += ramp_source_type->increase_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->increase_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
        else if (fabsf(ramp_source_type->input) < ramp_source_type->out)
        {
            ramp_source_type->out -= ramp_source_type->decrease_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->decrease_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
    }
    else if(input < 0)
    {
        if(ramp_source_type->input < ramp_source_type->out) {
            ramp_source_type->out -= ramp_source_type->increase_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->increase_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
        else if(fabsf(ramp_source_type->input) < ramp_source_type->out)
        {
            ramp_source_type->out += ramp_source_type->increase_rate;
            if (fabsf(ramp_source_type->input - ramp_source_type->out) < ramp_source_type->decrease_rate)
            {
                ramp_source_type->out = ramp_source_type->input;
            }
        }
    }
    else
    {
        if(ramp_source_type->out > 0)
        {
            ramp_source_type->out -= ramp_source_type->decrease_rate;
            if(ramp_source_type->out < 0)
            {
                ramp_source_type->out = 0;
            }
        }
        else
        {
            ramp_source_type->out += ramp_source_type->decrease_rate;
            if(ramp_source_type->out > 0)
            {
                ramp_source_type->out = 0;
            }
        }
    }
}

