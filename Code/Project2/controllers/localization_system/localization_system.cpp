#include <math.h>
#include <cstring>
#include <random>

#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>

#define NUM_ROBOTS 5
#define NOISE_SCALE 0.1 // Noise scale factor, n = N(0, NOISE_SCALE * d), where d is the distance between the emitter and the robot

using namespace webots;

struct Position {
  double x;
  double y;
} typedef Position;

Position get_robot_position(Supervisor *supervisor, int robot_id) {

  Position self_position, position;

  const double* self_translation = supervisor->getSelf()->getPosition();
  self_position.x = self_translation[0];
  self_position.y = self_translation[2];
  
  // Get the robot node
  Node *robot_node = supervisor->getFromDef("e-puck"+std::to_string(robot_id));
  // Get the translation field
  const double *translation = robot_node->getPosition();
  // Set the pose
  position.x = translation[0];
  position.y = translation[1];

  // Get distance from emitter to robot
  float distance = sqrt(pow(position.x - self_position.x, 2) + pow(position.y - self_position.y, 2));

  // Add gaussian noise to the robot position, proportional to the distance
  static std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, NOISE_SCALE * distance);
  position.x += distribution(generator);
  position.y += distribution(generator);

  return position;
}

void send_robot_position(Emitter *emitter, int robot_id, Position position){
  char message[64];
  sprintf(message, "%d,%.3f,%.3f", robot_id, position.x, position.y);
  emitter->send(message, strlen(message)+1);
}

int main(int argc, char **argv) {

  // Get the frequency from the command line arguments
  float frequency = -1.0; // Default value
  if (argc == 2) {
    frequency = atof(argv[1]);
    std::cout << "### Using localization system with frequency " << frequency << " Hz ###" << std::endl;
  }
  else{
    std::cerr << "Error: got " << argc-1 << " arguments, expected 1" << std::endl;
    std::cerr << "Usage: ./localization_system <frequency>" << std::endl;
    return 1;
  }

  // If the frequency is 0, we are not using the localization system
  if(frequency <= 0.0f){
    std::cout << "Not using localization system, exiting" << std::endl;
    return 0;
  }

  // create the Robot instance.
  Supervisor *supervisor = new Supervisor();

  // get the time step of the current world in ms
  int timeStep = (int)supervisor->getBasicTimeStep();

  Emitter *emitter = supervisor->getEmitter("emitter");

  // Number of steps between localization updates
  int localization_steps = (int)ceil(1.0f / (frequency * (float)timeStep / 1000.0f));
  std::cout << "    Sending localization data every " << localization_steps << " steps" << std::endl;

  // Main loop:
  int step_counter = 0; 
  while (supervisor->step(timeStep) != -1) {
    
    // Count the steps
    step_counter++;
    if (step_counter < localization_steps)
      continue;

    // Get and send the robot positions
    for(int r=0; r<NUM_ROBOTS; r++){
      send_robot_position(emitter, r, get_robot_position(supervisor, r));
    }
    step_counter = 0;    
  };

  // Enter here exit cleanup code.

  delete supervisor;
  return 0;
}
