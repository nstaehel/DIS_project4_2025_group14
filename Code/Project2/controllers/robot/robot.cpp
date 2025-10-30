/**
 * This controller receives messages from the localization system and prints them.
 */

// Standard C headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Webots C API headers
#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/motor.h>

// Standard C++ headers
#include <iostream>

// Eigen headers (C++)
#include <Eigen/Geometry>

static int TIME_STEP = 32; // milliseconds

/**
 * @brief Initialize the robot and return its id.
 */
int init_robot() {

  wb_robot_init();
  TIME_STEP = (int)wb_robot_get_basic_time_step();
  const char* name = wb_robot_get_name();
  const int robot_id = atoi(name + 6); // Extract the robot id from the name "e-puckX"

  return robot_id;
}

/**
 * @brief Simple Eigen3 demo
 */
void eigen_demo(){
  Eigen::Vector2d v(1.0, 2.0);  // 2D column vector
  Eigen::Matrix2d m;            // 2x2 matrix
  m << 1, 2,                    // Matrix initialization
       3, 4;
  Eigen::Vector2d r = m * v;    // matrix vector multiplication
  std::cout << "Eigen demo: r = m * v = [" << r.transpose() << "]" << std::endl;
}

int main(int argc, char **argv) {

  const int robot_id = init_robot();
  printf("### e-puck%d: starting controller ###\n", robot_id);

  // Initialize the receiver
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // Initialize the motors (set constant forward velocity)
  WbDeviceTag left_wheel = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_wheel = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel,  0.0);
  wb_motor_set_velocity(right_wheel, 0.0);

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {

    // Receive messages from localization system and print them
    while (wb_receiver_get_queue_length(receiver) > 0) {

      // Retrieve the message
      const char *message = (const char *)wb_receiver_get_data(receiver);
      double intensity = wb_receiver_get_signal_strength(receiver);
      printf("e-puck%d received: %s with intensity %.3lf\n", robot_id, message, intensity);

      // Parse the message
      int rid;
      double x, y;
      sscanf(message, "%d,%lf,%lf", &rid, &x, &y);

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // EXAMPLE: estimate the mean and standard deviation of robot's x coordinate iteratively                   //
      static double x_mean = 0.0,
                    x_std  = 0.0;
      static int    n      = 0;
      if(rid == robot_id){ // Only consider messages from itself
        x_mean = (x_mean*n + x) / (n+1);
        double delta = x - x_mean;
        x_std  = sqrt((n*x_std*x_std + delta*delta) / (n+1)); // don't use as is: biased and numerically unstable!
        n++;
        printf("e-puck%d: x_mean = %.3lf, x_std = %.3lf (n=%d)\n", robot_id, x_mean, x_std, n);
      }
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Remove the current message from the queue
      wb_receiver_next_packet(receiver);
    }
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
