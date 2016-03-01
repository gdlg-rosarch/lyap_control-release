#pragma once

// Using ublas for vector/matrix multiplication
#include <boost/numeric/ublas/vector.hpp>
typedef boost::numeric::ublas::vector <double> ublas_vector;
#include <boost/numeric/ublas/matrix.hpp>

// Using to check for NaN
#include <boost/math/special_functions/fpclassify.hpp>

#include "ros/ros.h"
#include "lyap_control/plant_msg.h"
#include "lyap_control/controller_msg.h"
#include "lyap_control/controller_globals.h"


///////////////////////////////////////////////////////////////////////////////
// User-defined parameters - MAKE YOUR CHANGES HERE
///////////////////////////////////////////////////////////////////////////////

// 'Aggressiveness' of the controller, in dB
static const double V_dot_target_initial_dB= -100.0;

static const double high_saturation_limit [] = {10.0, 10.0};
static const double low_saturation_limit []= {-10.0, -10.0};

// Parameter for V2 step location, gamma
static const double g = 1.0;

// Threshold for switching to the alternative Lyapunov function
static const double switching_threshold = 0.01;

// The state space definition-- the dynamic equations of the model.
// Calculates dx/dt
// model_definition sees u b/c it's a global variable, it can't be an argument
void model_definition(const ublas_vector &x, ublas_vector &dxdt, const double t)
{
  dxdt[0] = 0.1*x[0]+u[0];
  dxdt[1] = 2.*x[1]+u[1];
}

/////////////////////////////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////////////////////////////

// The main callback to calculate u
void chatterCallback(const lyap_control::plant_msg& msg);

void initial_error_check(const lyap_control::plant_msg& msg);

// Calculate dx_dot_du and open_loop_dx_dt
void calculate_dx_dot_du(boost::numeric::ublas::matrix<double> &dx_dot_du, ublas_vector & open_loop_dx_dt);

// Calculate V_initial, V, and V_dot_target
void calculate_V_and_damping(double &V_dot_target);

// Calculate a stabilizing control effort
void calculate_u(ublas_vector &D, ublas_vector &open_loop_dx_dt, const double &V_dot_target, boost::numeric::ublas::matrix<double> &dx_dot_du);
