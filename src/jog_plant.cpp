// This file publishes a target location and the controller attempts to track it.
// x: current location
// setpoint: target location
// u: jog velocity

#include "lyap_control/jog_plant_header.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plant");
  
  ros::NodeHandle plant_node;

  // Declare a new message variable
  lyap_control::plant_msg  msg;

  // Initial conditions -- these were defined in the header file
  // Later: get the actual initial robot position
  msg.x[0] = 0.05;
  msg.t = 0.0;
  msg.setpoint[0] = 0.0;

  // Publish a plant.msg
  ros::Publisher chatter_pub = plant_node.advertise<lyap_control::plant_msg>("state", 1);

  // Subscribe to "control_effort" topic to get a controller_msg.msg
  ros::Subscriber sub = plant_node.subscribe("control_effort", 1, chatterCallback );
  
  double x_dot [num_states] = {0};

  ros::Rate loop_rate(1/delta_t); // Control rate in Hz


  while (ros::ok())
  {
    ROS_INFO("x1: %f setpoint: %f", msg.x[0], msg.setpoint[0]);

    // Later: send the jog command to the robot
    chatter_pub.publish(msg);


    // Update the setpoint.
    // x: current position
    // setpoint: target position
    // u: commanded velocity
    msg.setpoint[0] = msg.setpoint[0]+delta_t/100.0; // Linear motion with time
    x_dot[0] = u[0]; // Commanded velocity from the controller

    // Update the actual position and time.
    // Later: get the actual robot position here
    msg.x[0] = msg.x[0]+x_dot[0]*delta_t;
    msg.t = msg.t+delta_t;

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// Callback when something is published on 'control_effort'
void chatterCallback(const lyap_control::controller_msg& u_msg)
{
  //ROS_INFO("I heard: [%f]", u_msg.u[0]);

  // Define the stabilizing control effort
  u[0] = u_msg.u[0];
}
