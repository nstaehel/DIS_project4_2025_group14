/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        e-puck_task1b.c
 * author:      
 * description: E-puck file for market-based task allocations task1.b of project4
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>
#include <webots/motor.h>
  
#include <webots/supervisor.h> 

#include "../supervisor/message.h" 
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot


#define DEBUG 1
#define TIME_STEP           64      // Timestep (ms)
#define RX_PERIOD           2    // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define DELTA_T             TIME_STEP/1000   // Timestep (seconds)
#define MAX_SPEED         800     // Maximum speed

#define MAX_ENERGY_TIME 120.0 // 2 minutes 

#define INVALID          -999
#define BREAK            -999 //for physics plugin

#define NUM_ROBOTS 5 

// --- PATH PLANNING CONSTANTS ---
// Vertical Wall is at X=0.125. Gap is below Y=-0.2.
#define VERTICAL_DOOR_X  0.125
#define VERTICAL_DOOR_Y -0.50  // Safe point at the bottom

// Horizontal Wall is at Y=0, X < -0.26. Gap is to the right.
#define HORIZONTAL_DOOR_X -0.10 // Safe point near center
#define HORIZONTAL_DOOR_Y  0.00


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 500   // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
	WAITING_FOR_TASK = 5 // state added for the robot to wait for a task
} robot_state_t;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS           8
#define BIAS_SPEED           400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};

// Row 0 = Robot A, Row 1 = Robot B
// Col 0 = Task A, Col 1 = Task B
int service_times[2][2] = {
    {3000, 5000},  // Rob A: Task A=3s, Task B=5s
    {9000, 1000}   // Rob B: Task A=9s, Task B=1s
};
// timing variables for doing the task
double task_start_time = 0.0; 
int current_task_duration = 0;

// The state variables
int clock;
uint16_t robot_id;          // Unique robot ID
robot_state_t state;                 // State of the robot
double my_pos[3];           // X, Z, Theta of this robot
char target_valid;          // boolean; whether we are supposed to go to the target
double target[99][4];       // x and z coordinates of target position (max 99 targets) // I added the fourth ( x, y, ID and the type is the fourth)
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor
int my_type = 0; // 0 for A, 1 for B type of the robot
double active_time = 0.0; // time for wich the robot has been active
int target_list_length = 0; // number of tasks in the queue

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;


// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}

void limit(int *number, int limit) {
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}


void get_intermediate_target(double my_x, double my_y, double target_x, double target_y, double *out_x, double *out_y) {
    
    // Check if the vertical wall (at X = 0.125) blocks the direct path.
    // If the robot and the target are on opposite sides of this X-coordinate,
    // we must route through the specific gap in the vertical wall.
    int i_am_left     = (my_x < VERTICAL_DOOR_X);
    int target_is_left = (target_x < VERTICAL_DOOR_X);

    if (i_am_left != target_is_left) {
        // We must cross the vertical wall. Go to the bottom gap.
        *out_x = VERTICAL_DOOR_X;
        *out_y = VERTICAL_DOOR_Y;
        return;
    }

    // If both points are on the left side, we need to check the horizontal wall at Y=0.
    // This wall only exists for X coordinates less than -0.26.
    if (i_am_left && target_is_left) {

        int i_am_up     = (my_y > 0);
        int target_is_up = (target_y > 0);

        // If the start and end points are in different vertical zones (one up, one down),
        // and either point is far enough left to hit the wall, divert to the horizontal gap.
        if (i_am_up != target_is_up) {
             if (my_x < -0.26 || target_x < -0.26) {
                 *out_x = HORIZONTAL_DOOR_X;
                 *out_y = HORIZONTAL_DOOR_Y;
                 return;
             }
        }
    }
	// No walls block the path, so proceed directly to the target coordinates.
    *out_x = target_x;
    *out_x = target_x;
    *out_y = target_y;
}

// Computes wheel speed to go towards a goal
void compute_go_to_goal(int *msl, int *msr) 
{
    double nav_x, nav_y;

    get_intermediate_target(my_pos[0], my_pos[1], target[0][0], target[0][1], &nav_x, &nav_y);
    // // Compute vector to goal
    float a = nav_x - my_pos[0];
    float b = nav_y - my_pos[1];
    // Compute wanted position from event position and current location
    float x =  a*cosf(my_pos[2]) - b*sinf(my_pos[2]); // x in robot coordinates
    float y =  a*sinf(my_pos[2]) + b*cosf(my_pos[2]); // y in robot coordinates

    float Ku = 0.2;   // Forward control coefficient
    float Kw = 10.0;  // Rotational control coefficient
    float range = 1; //sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position
    
    // Compute forward control
    float u = Ku*range*cosf(bearing);
    // Compute rotational control
    float w = Kw*range*sinf(bearing);
    
    // Convert to wheel speeds!
    *msl = 50*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    *msr = 50*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

double calculate_bid(double task_x, double task_y, int task_type) {
    
    // If I have tasks in queue, I start from the location of the LAST task.
    // If I am free, I start from my current position.
    double start_x, start_y;
    if (indx > 0) { // 'indx' is updated to target_list_length before calling this
        start_x = target[indx-1][0];
        start_y = target[indx-1][1];
    } else {
        start_x = my_pos[0];
        start_y = my_pos[1];
    }

    // If there is a wall, it returns the Door coordinates.
    // If there is no wall, it returns the Task coordinates directly.
    double waypoint_x, waypoint_y;
    get_intermediate_target(start_x, start_y, task_x, task_y, &waypoint_x, &waypoint_y);

    // We calculate the travel distance
    // Segment 1: Start -> Waypoint (Door)
    double dist_segment_1 = dist(start_x, start_y, waypoint_x, waypoint_y);
    // Segment 2: Waypoint (Door) -> Final Task
    // Note: If no wall, waypoint == task, so this becomes 0.
    double dist_segment_2 = dist(waypoint_x, waypoint_y, task_x, task_y);
    
    double total_distance = dist_segment_1 + dist_segment_2;

    // Travel Time = Distance / Max Speed (0.5 m/s)
    double travel_time = total_distance / 0.5;

    // Service Time = How long does it take to do this specific task type depending on the type of the robot and the task
    // (service_times is in ms, so divide by 1000.0)
    double service_time = service_times[my_type][task_type] / 1000.0;

    // Total cost computing
    return travel_time + service_time;
}

// Check if we received a message and extract information
static void receive_updates() 
{
    message_t msg;
    //int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const message_t *pmsg = wb_receiver_get_data(receiver_tag);
        
        // save a copy, cause wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(message_t));
        wb_receiver_next_packet(receiver_tag);

        // double check this message is for me
        // communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if(msg.robot_id != robot_id) {
            fprintf(stderr, "Invalid message: robot_id %d "  "doesn't match receiver %d\n", msg.robot_id, robot_id);
            //return;
            exit(1);
        }

        //find target list length
        i = 0;
        while(target[i][2] != INVALID){ i++;}
        target_list_length = i;  
        
        if(target_list_length == 0) target_valid = 0;   

        
        // Event state machine
        if(msg.event_state == MSG_EVENT_GPS_ONLY)
        {
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
            continue;
        }
        else if(msg.event_state == MSG_QUIT)
        {
            // Set speed
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            wb_robot_step(TIME_STEP);
            exit(0);
        }
        else if(msg.event_state == MSG_EVENT_DONE)
        {
            // If event is done, delete it from array 
            for(i=0; i<=target_list_length; i++)
            {
                if((int)target[i][2] == msg.event_id) 
                { //look for correct id (in case wrong event was done first)
                    for(; i<=target_list_length; i++)
                    { //push list to the left from event index
                        target[i][0] = target[i+1][0];
                        target[i][1] = target[i+1][1];
                        target[i][2] = target[i+1][2];
                    }
                }
            }
            // adjust target list length
            if(target_list_length-1 == 0) target_valid = 0; //used in general state machine 
            target_list_length = target_list_length-1;  
            printf("--- Robot %d COMPLETED Event %d. Tasks Remaining: %d.\n", 
                   robot_id, msg.event_id, target_list_length);  
        }
        else if(msg.event_state == MSG_EVENT_WON)
        {
            // insert event at index
            for(i=target_list_length; i>=msg.event_index; i--)
            {
                target[i+1][0] = target[i][0];
                target[i+1][1] = target[i][1];
                target[i+1][2] = target[i][2];
            }
            target[msg.event_index][0] = msg.event_x;
            target[msg.event_index][1] = msg.event_y;
            target[msg.event_index][2] = msg.event_id;
			target[msg.event_index][3] = msg.event_type; // we store also the type of the task
            target_valid = 1; //used in general state machine
            target_list_length = target_list_length+1;
            printf(">>> Robot %d WON Event %d! New Queue Size: %d/3. (Inserted at index %d)\n", 
                   robot_id, msg.event_id, target_list_length, msg.event_index);
        }
        // check if new event is being auctioned
        else if(msg.event_state == MSG_EVENT_NEW)
        {         
            if (active_time > MAX_ENERGY_TIME) {
                return; 
            }

            if (target_list_length >= 3) {
                return;
            }

            /*
			// If we are busy or have a target, do not bid. 
			if (state == WAITING_FOR_TASK) {
        	    return; 
    		}
            */
            indx = 0;
            double d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
            if(target_list_length > 0){
                for(i = 0; i < target_list_length; i++){
                    if(i == 0){
                        double dbeforetogoal = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
                        double daftertogoal = dist(target[i][0], target[i][1], msg.event_x, msg.event_y);
                        double dbeforetodafter = dist(my_pos[0], my_pos[1], target[i][0], target[i][1]);
                        d = dbeforetogoal + daftertogoal - dbeforetodafter;
                    }else{
                        double dbeforetogoal = dist(target[i-1][0], target[i-1][1], msg.event_x, msg.event_y);
                        double daftertogoal = dist(target[i][0], target[i][1], msg.event_x, msg.event_y);
                        double dbeforetodafter = dist(target[i-1][0], target[i-1][1], target[i][0], target[i][1]);
                        if((dbeforetogoal + daftertogoal - dbeforetodafter) < d){
                            d = dbeforetogoal + daftertogoal - dbeforetodafter;
                            indx = i;
                        }
                        if(i == target_list_length - 1){
                            if(daftertogoal < d) {
                                d = daftertogoal;
                                indx = i+1;
                            }
                        }
                    }
                }
            }

            double total_cost = calculate_bid(msg.event_x, msg.event_y, msg.event_type);
            /*
            printf("robot %d: bidding on Event %d. Cost: %.2f. Sending on Channel %d\n", 
            robot_id, msg.event_id, total_cost, robot_id+1);
            */
            printf("Robot %d [Queue: %d/3]: Bidding on Event %d. Cost: %.2f.\n", 
                   robot_id, target_list_length, msg.event_id, total_cost);
                
            // Send my bid to the supervisor
            const bid_t my_bid = {robot_id, msg.event_id, total_cost, indx};
            wb_emitter_set_channel(emitter_tag, robot_id+1);
            wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));            
        }
    }
    
    
    // Communication with physics plugin (channel 0)            
    i = 0; k = 1;
    while((int)target[i][2] != INVALID){i++;}
    target_list_length = i; 
    if(target_list_length > 0)
    {        
        // Line from my position to first target
        wb_emitter_set_channel(emitter_tag,0);         
        buff[0] = BREAK; // draw new line
        buff[1] = my_pos[0]; 
        buff[2] = my_pos[1];
        buff[3] = target[0][0];
        buff[4] = target[0][1];
        // Lines between targets
        for(i=5;i<5*target_list_length-1;i=i+5)
        {
            buff[i] = BREAK;
            buff[i+1] = buff[i-2]; 
            buff[i+2] = buff[i-1];
            buff[i+3] = target[k][0]; 
            buff[i+4] = target[k][1];
            k++;  
        }
        // send, reset channel        
        if(target[0][2] == INVALID){ buff[0] = my_pos[0]; buff[1] = my_pos[1];}
        wb_emitter_send(emitter_tag, &buff, (5*target_list_length)*sizeof(float));
        wb_emitter_set_channel(emitter_tag,robot_id+1);                     
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RESET and INIT (combined in function reset()) */
void reset(void) 
{
    wb_robot_init();
    int i;

  //get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
    char s[4] = "ps0";
    for(i=0; i<NB_SENSORS;i++) 
    {
        // the device name is specified in the world file
        ds[i]=wb_robot_get_device(s);      
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i],64);
    } 

    clock = 0;
    indx = 0;
    
    // Init target positions to "INVALID"
    for(i=0;i<99;i++){ 
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID; 
		target[i][3] = 0; // we inizialize the state to A but we will assign it in the line below
    }

	

    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name; 
    robot_name = (char*) wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck%d", &tmp_id)) {robot_id = (uint16_t)tmp_id;} 
    else {fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name); exit(1);}

	if (robot_id < 2) my_type = 0; // Assign the type of the robot, we have 2 robot type A (ID 0 and 1) and then 3 robot type B
	else my_type = 1;

	printf("DEBUG: I am Robot %d. My Type is %d (0=A, 1=B).\n", robot_id, my_type);

    // Am I used in this simulation?
    if (robot_id >= NUM_ROBOTS) {
        fprintf(stderr, "Robot %d is not needed. exiting ...\n", robot_id); 
        wb_robot_cleanup(); 
        exit(0);
    }

    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id+1);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);
    
    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}


void update_state(int _sum_distances)
{

    static int energy_depleted_printed = 0;

	// first we check if we are over the MAX_ENERGY_TIME
	if (active_time > MAX_ENERGY_TIME) {
        if (!energy_depleted_printed) {
            printf("Robot %d has run out of ENERGY (Time: %.2f s). Shutting down.\n", 
                   robot_id, active_time);
            energy_depleted_printed = 1;
        }
        state = STAY; 
        return;
    }

	if (state == WAITING_FOR_TASK) {
		double current_time = wb_robot_get_time();
        if (current_time - task_start_time < (current_task_duration / 1000.0)) {
            return; // Continue waiting
        }
        // Task complete -> Return to normal 
        state = DEFAULT_STATE;
	}

    // Ensure target_valid remains true if tasks exist in the queue
    if (target_list_length > 0) target_valid = 1;
    else target_valid = 0;

	double d = 999; // we just initialize with a big number 
    if (target_valid) d = dist(my_pos[0], my_pos[1], target[0][0], target[0][1]);
	
    if (_sum_distances > STATECHANGE_DIST && state == GO_TO_GOAL)
    {
        state = OBSTACLE_AVOID;
    }
	else if (target_valid && d < 0.1) { // 0.1m range to do the task
        state = WAITING_FOR_TASK; // we go in the state of doing the task
        task_start_time = wb_robot_get_time();
        // we look up duration based on stored type
        current_task_duration = service_times[my_type][(int)target[0][3]];
		// we stop the motor, so the robot stops
		wb_motor_set_velocity(left_motor, 0);
  	    wb_motor_set_velocity(right_motor, 0);
    }
    else if (target_valid)
    {
        state = GO_TO_GOAL;
    }
    else
    {
        state = DEFAULT_STATE;
    }
}

// Odometry
void update_self_motion(int msl, int msr) {
    double theta = my_pos[2];
  
    // Compute deltas of the robot
    double dr = (double)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = (double)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl)/2.0;
    double dtheta = (dr - dl)/AXLE_LENGTH;
  
    // Compute deltas in the environment
    double dx = du * cosf(theta);
    double dy = du * sinf(theta);
  
    // Update position
    my_pos[0] += dx;
    my_pos[1] -= dy;
    my_pos[2] -= dtheta;
    
    // Keep orientation within 0, 2pi
    if (my_pos[2] > 2*M_PI) my_pos[2] -= 2.0*M_PI;
    if (my_pos[2] < 0) my_pos[2] += 2.0*M_PI;

    // Keep track of highest velocity for modelling
    double velocity = du * 1000.0 / (double) TIME_STEP;
    if (state == GO_TO_GOAL && velocity > stat_max_velocity)
        stat_max_velocity = velocity;
}


// Compute wheel speed to avoid obstacles
void compute_avoid_obstacle(int *msl, int *msr, int distances[]) 
{
    int d1=0,d2=0;       // motor speed 1 and 2     
    int sensor_nb;       // FOR-loop counters    

    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {   
       d1 += (distances[sensor_nb]-300) * Interconn[sensor_nb];
       d2 += (distances[sensor_nb]-300) * Interconn[sensor_nb + NB_SENSORS];
    }
    d1 /= 80; d2 /= 80;  // Normalizing speeds

    *msr = d1+BIAS_SPEED; 
    *msl = d2+BIAS_SPEED; 
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

// Checks if our path is blocked and returns the next immediate waypoint


// RUN e-puck
void run(int ms)
{
    float msl_w, msr_w;
    // Motor speed and sensor variables	
    int msl=0,msr=0;                // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    int sum_distances=0;        // sum of all distance sensor inputs, used as threshold for state change.  	

    // Other variables
    int sensor_nb;

    // Add the weighted sensors values
    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {  
        distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
        sum_distances += distances[sensor_nb];
    }

    // Get info from supervisor
    update_state(sum_distances);

    receive_updates();

	// we update the active time of the robot adding the ms so the TIMESTEP
	if (state == GO_TO_GOAL || state == OBSTACLE_AVOID || state == WAITING_FOR_TASK) {
    active_time += (double)ms / 1000.0;
	}

    // Set wheel speeds depending on state
    switch (state) {
        case STAY:
            msl = 0;
            msr = 0;
            break;

        case GO_TO_GOAL:
            compute_go_to_goal(&msl, &msr);
            break;

        case OBSTACLE_AVOID:
            compute_avoid_obstacle(&msl, &msr, distances);
            break;

        case RANDOM_WALK:
            msl = 400;
            msr = 400;
            break;

		case WAITING_FOR_TASK:
        msl = 0;
        msr = 0;
        break;

        default:
            printf("Invalid state: robot_id %d \n", robot_id);
    }
    // Set the speed
    msl_w = msl*MAX_SPEED_WEB/1000;
    msr_w = msr*MAX_SPEED_WEB/1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    update_self_motion(msl, msr);

    // Update clock
    clock += ms;
}

// MAIN
int main(int argc, char **argv) 
{
    reset();
  
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {run(TIME_STEP);}
    wb_robot_cleanup();

    return 0;
}
