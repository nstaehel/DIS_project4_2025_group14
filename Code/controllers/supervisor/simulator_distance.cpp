#define RAND ((float) rand()/RAND_MAX)
#include <cmath>
#include <Point2d.h>
#include <supervisor.cc>
#define TOTAL_ITER 100

const Point2d wall_1_left_pt = Point2d(-0.625,0.0);
const Point2d wall_1_right_pt = Point2d(-0.25,0.0);
const Point2d wall_2_upper_pt = Point2d(0.125,0.625);
const Point2d wall_2_lower_pt = Point2d(0.125,-0.225);

double average_distance = 0.0;

for(int i=0; i<TOTAL_ITER; i++){
    double travel_distance;
    Point2d robot_rand = rand_coord();
    Point2d event_rand = rand_coord();
    bool intersects_wall_1 = get_line_intersection(robot_rand.x,robot_rand.y,event_rand.x,event_rand.y,-0.625,0.0,-0.25,0.0);
    bool intersects_wall_2 = get_line_intersection(robot_rand.x,robot_rand.y,event_rand.x,event_rand.y,0.125,0.625,0.125,-0.225);
    if(intersects_wall_1){
        travel_distance+=robot_rand.Distance(wall_1_right_pt);
        if(intersects_wall_2){
            travel_distance+=wall_1_right_pt.Distance(wall_2_lower_pt)+wall_2_lower_pt.Distance(event_rand);
        }else{
            travel_distance+=wall_1_right_pt.Distance(event_rand);
        }
    }else if(intersects_wall_2){
        travel_distance+=robot_rand.Distance(wall_2_lower_pt)+wall_2_lower_pt.Distance(event_rand);
    }else{
        travel_distance+=robot_rand.Distance(event_rand);
    }
    average_distance+=travel_distance;
}
average_distance=average_distance/TOTAL_ITER;
printf("Average distance traveled (assuming only presence of 1 robot and 1 event) is %f", average_distance);