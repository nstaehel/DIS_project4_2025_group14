//******************************************************************************
//  Name:   supervisor.c
//  Author: -
//  Date: -
//  Rev: -
//******************************************************************************
#include <assert.h>
#include <bitset>
#include <cstdlib>
#include <cstdio>
#include <cmath>

#include <vector>
#include <memory>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */

using namespace std;

#include <webots/robot.h>
#include <webots/supervisor.h>
#include "Point2d.h"


#define DBG(x) printf x
#define RAND ((float) rand()/RAND_MAX)

#define NUM_ROBOTS_A 2              
#define NUM_ROBOTS_B 3  


#define STEP_SIZE 64            // simulation step size

#define EVENT_RANGE (0.1)      // distance within which a robot must come to do event

#define GPS_INTERVAL (500)


#define TOTAL_EVENTS_ALWAYS 10         // number of active events
#define MAX_RUNTIME (3*60)      // ...total runtime after which simulation stops
#define ACTIVITY_TIME_MAX (2*60) //Time spent doing tasks or moving


WbNodeRef g_event_nodes[MAX_EVENTS];
vector<WbNodeRef> g_event_nodes_free;

double gauss(void) {
  double x1, x2, w;
  do {
      x1 = 2.0 * RAND - 1.0;
      x2 = 2.0 * RAND - 1.0;
      w = x1*x1 + x2*x2;
  } while (w >= 1.0);

  w = sqrt((-2.0 * log(w))/w);
  return(x1*w);
}

double* rand_coord(double radius) {
  double* array (double*)malloc(2 * sizeof(double));
  do{ 
  double rand_x=-0.625 + 1.25*RAND;
  double rand_y=-0.625 + 1.25*RAND;
  } while((rand_x<=-0.25 + radius && (rand_y>0.005+radius || rand_y<-0.005-radius))||((rand_x>0.125-0.005-radius&&rand_x<0.125+0.005+radius)&&(rand_y>=-0.225-radius)));
  array[0]=rand_x;
  array[1]=rand_y;
  return array;
}

double expovariate(double mu) {
  double uniform = RAND;
  while (uniform < 1e-7) uniform = RAND;
  return -log(uniform) * mu;
}

// Event class
class Event {*
  WbNodeRef node_;       //event node ref
  uint16_t assigned_to_; //id of the robot that will handle this event
  uint16_t event_type;

  // Auction data
  uint64_t t_announced_;        //time at which event was announced to robots
  bitset<NUM_ROBOTS> bids_in_;
  uint16_t best_bidder_;        //id of the robot that had the best bid so far
  double best_bid_;             //value of the best bid (lower is better)
  uint64_t t_done_;             //time at which the assigned robot reached the event
  int bidder_index;             //index at which the bidder will put event in tasklist

  // Public functions
  public:
  //Event creation
  Event(uint16_t id) : id_(id), pos_(rand_coord(), rand_coord()),
    assigned_to_(-1), event_type(-1), t_announced_(-1), best_bidder_(-1), best_bid_(0.0), t_done_(-1)
  {
    node_ = g_event_nodes_free.back();  // Place node
    g_event_nodes_free.pop_back();
    
    double event_node_pos[3];           // Place event in arena
    event_node_pos[0] = pos_.x;
    event_node_pos[1] = pos_.y;
    event_node_pos[2] = .01;
    wb_supervisor_field_set_sf_vec3f(
      wb_supervisor_node_get_field(node_,"translation"),
      event_node_pos);
  }

  //SIMPLE WORLD
  Event(uint16_t id, Point2d pos) : id_(id), pos_(pos),
    assigned_to_(-1), t_announced_(-1), best_bidder_(-1), best_bid_(0.0), t_done_(-1)
  {
    node_ = g_event_nodes_free.back();  // Place node
    g_event_nodes_free.pop_back();
    
    double event_node_pos[3];           // Place event in arena
    event_node_pos[0] = pos_.x;
    event_node_pos[1] = pos_.y;
    event_node_pos[2] = .01;
    wb_supervisor_field_set_sf_vec3f(
      wb_supervisor_node_get_field(node_,"translation"),
      event_node_pos);
  }
  //SIMPLE WORLD

  bool is_assigned() const { return assigned_to_ != (uint16_t) -1; }
  bool was_announced() const { return t_announced_ != (uint64_t) -1; }
  bool has_bids() const { return best_bidder_ != (uint16_t) -1; }
  bool is_done() const { return t_done_ != (uint64_t) -1; }

  // Check if event can be assigned
  void updateAuction(uint16_t bidder, double bid, int index) {
    if (bid >= 0.0 && (!has_bids() || bid < best_bid_)) {
      best_bidder_ = bidder;
      best_bid_ = bid;
      bidder_index = index;  
    }
    bids_in_.set(bidder);
    if (bids_in_.all()) assigned_to_ = best_bidder_;
  }

  void restartAuction() {
    assigned_to_ = -1;
    t_announced_ = -1;
    bids_in_.reset();
    best_bidder_ = -1;
    best_bid_ = 0.0;
    t_done_ = -1;
  }

  void markDone(uint64_t clk) {
    t_done_ = clk;
    double event_node_pos[3] = {-5,-5,0.1};
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(node_,"translation"),
                                     event_node_pos);
    g_event_nodes_free.push_back(node_);
  }
};

int main(int argc, char *argv[]) {

  /* initialize Webots */
  wb_robot_init();
  
  return 0;
}

