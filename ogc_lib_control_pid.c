#include "ogc_lib_control_pid.h"

bool ogc_lib_control_pid__init( struct pid_control_set* a,
                                double proportional_factor,
                                double integral_factor,
                                double derivative_factor,
                                double desired_target
                              )
{
    if ( NULL != a )
    {
        a.factor_p = proportional_factor;
        a.factor_i = integral_factor;
        a.factor_d = derivative_factor;
        a.target = desired_target;
        a.accumulated_error = 0;
        a.current_error = 0;
        a.outpout_setting = 0;
    }
    else
    {
        return ( false );
    }
}

bool ogc_lib_control_pid__calc( struct pid_control_set* a,
                                double delta_time
                              )
{
    double current_error;
    double delta_error;
    if ( NULL != a )
    {
        current_error = a.target - a.feedback;
        a.accumulated_error += current_error;
        delta_error = current_error - a.current_error;
        a.current_error = current_error;
        a.output_setting = a.factor_p * ( current_error )
                         + a.factor_i * ( a.accumulated_error )
                         + a.factor_d * ( delta_error / delta_time );
        return ( true );
    }
    else
    {
        return ( false );
    }
}

