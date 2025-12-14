import random as rand
import math

# Constants
TOTAL_ITER = 10000

# Wall endpoints
wall_1_left_pt = (-0.625, 0.0)
wall_1_right_pt = (-0.25, 0.0)
wall_2_upper_pt = (0.125, 0.625)
wall_2_lower_pt = (0.125, -0.225)

def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def ccw(A, B, C):
    """Check if three points are listed in counter-clockwise order."""
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def segments_intersect(A, B, C, D):
    """Check if line segment AB intersects line segment CD."""
    return (ccw(A, C, D) != ccw(B, C, D)) and (ccw(A, B, C) != ccw(A, B, D))

def get_line_intersection(robot_x, robot_y, event_x, event_y, wall_x1, wall_y1, wall_x2, wall_y2):
    """Check if the segment between robot and event intersects with a wall segment."""
    A = (robot_x, robot_y)
    B = (event_x, event_y)
    C = (wall_x1, wall_y1)
    D = (wall_x2, wall_y2)
    return segments_intersect(A, B, C, D)

def rand_coord():
    """Generate random coordinates with the arena constraints."""
    rand_x = rand.uniform(-0.625, 0.625)
    rand_y = rand.uniform(-0.625, 0.625)
    while((rand_x <= -0.20 and (rand_y < 0.055 or rand_y > -0.055))or((rand_x > 0.07 and rand_x < 0.18)and(rand_y >= -0.275)) ):
        rand_x = rand.uniform(-0.625, 0.625)
        rand_y = rand.uniform(-0.625, 0.625)
    return (rand_x, rand_y)

def main():
    average_distance = 0.0
    
    for i in range(TOTAL_ITER):
        travel_distance = 0.0
        
        # Generate random positions
        robot_rand = rand_coord()
        event_rand = rand_coord()
        
        # Check for intersections with walls
        intersects_wall_1 = get_line_intersection(
            robot_rand[0], robot_rand[1], 
            event_rand[0], event_rand[1],
            wall_1_left_pt[0], wall_1_left_pt[1],
            wall_1_right_pt[0], wall_1_right_pt[1]
        )
        
        intersects_wall_2 = get_line_intersection(
            robot_rand[0], robot_rand[1],
            event_rand[0], event_rand[1],
            wall_2_upper_pt[0], wall_2_upper_pt[1],
            wall_2_lower_pt[0], wall_2_lower_pt[1]
        )
        
        # Calculate travel distance based on wall intersections
        if intersects_wall_1:
            travel_distance += distance(robot_rand, wall_1_right_pt)
            if intersects_wall_2:
                travel_distance += distance(wall_1_right_pt, wall_2_lower_pt)
                travel_distance += distance(wall_2_lower_pt, event_rand)
            else:
                travel_distance += distance(wall_1_right_pt, event_rand)
        elif intersects_wall_2:
            travel_distance += distance(robot_rand, wall_2_lower_pt)
            travel_distance += distance(wall_2_lower_pt, event_rand)
        else:
            travel_distance += distance(robot_rand, event_rand)
        
        average_distance += travel_distance
    
    average_distance /= TOTAL_ITER
    print("Average distance traveled (assuming only presence of 1 robot and 1 event) is {:.6f}".format(average_distance))
    
    return average_distance

if __name__ == "__main__":
    main()
