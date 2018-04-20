
#pragma once

/////////////////////////////////////////////////////////////////////////////
// These variables are global b/c a ROS callback only takes 1 argument
/////////////////////////////////////////////////////////////////////////////

static double t=0; // time will be updated by listening to the 'plant' ROS topic
static double V=0; //current Lyapunov value
static double V_initial=0;
static int first_callback=1; // 1 signals that the callback has not been run yet. Triggers setup calcs

// Read the size of a plant_msg
lyap_control::plant_msg temp_plant_msg; // Just to read the msg size
const static int num_states = temp_plant_msg.x.size();

// Message variable for the control effort message
lyap_control::controller_msg  u_msg;
// Read the size of a 'controller' message
const static int num_inputs = u_msg.u.size();

ublas_vector x(num_states);
ublas_vector setpoint(num_states);
ublas_vector u(num_inputs);

// Convert user-specified aggressiveness from log to linear value
static double V_dot_target_initial = 0;
