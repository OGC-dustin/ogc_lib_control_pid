#ifndef OGC_LIB_CONTROL_PID_H
#define OGC_LIB_CONTROL_PID_H

struct
{
    double factor_p;
    double factor_i;
    double factor_d;
    double target;
    double feedback;
    double accumulated_error;
    double current_error;
    double output_setting;
} pid_control_set;

bool ogc_lib_control_pid__init( struct pid_control_set* a,
                                double proportional_factor,
                                double integral_factor,
                                double derivative_factor,
                                double desired_target
                              );
bool ogc_lib_control_pid__calc( struct pid_control_set* a,
                                double delta_time
                             );

#endif /* OGC_LIB_CONTROL_PID_H */

