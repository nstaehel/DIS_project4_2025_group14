import random
import math

def circle_square_intersection_area(r, k, center_x, center_y):
    """Calculate area of intersection between circle of radius r and square of side k centered at (center_x, center_y)"""
    # Square boundaries relative to circle center
    left = max(-k/2.0, center_x - k/2.0)
    right = min(k/2.0, center_x + k/2.0)
    bottom = max(-k/2.0, center_y - k/2.0)
    top = min(k/2.0, center_y + k/2.0)
    
    # If no intersection
    if left >= right or bottom >= top:
        return 0
    
    # Monte Carlo integration
    samples = 10000
    count = 0
    
    for _ in range(samples):
        # Sample point in the square bounding box
        x = random.uniform(left, right)
        y = random.uniform(bottom, top)
        
        # Check if point is in both circle and original square
        if x**2 + y**2 <= r**2:
            count += 1
    
    # Area of the bounding box times proportion of points in intersection
    bounding_area = (right - left) * (top - bottom)
    return (float(count) / samples) * bounding_area

def average_intersection_area(r, k, n_runs=1000):
    """Calculate average intersection area over n runs"""
    total_area = 0
    
    for _ in range(n_runs):
        # Generate random center uniformly in the square [-k/2, k/2]
        center_x = random.uniform(-k/2.0, k/2.0)
        center_y = random.uniform(-k/2.0, k/2.0)
        
        area = circle_square_intersection_area(r, k, center_x, center_y)
        total_area += area
    
    return total_area / float(n_runs)

# Example usage
if __name__ == "__main__":
    # Parameters
    r = 0.5 # Circle radius (0.268 for type 1)
    k = 1.25  # Square side length
    n_runs = 1000
    
    avg_area = average_intersection_area(r, k, n_runs)
    print "Average intersection area over %d runs:" % n_runs
    print "Circle radius: %.2f, Square side: %.2f" % (r, k)
    print "Average area: %.6f" % avg_area
    
    # Theoretical maximums for comparison
    circle_area = math.pi * r**2
    square_area = k**2
    print "\nFor reference:"
    print "Circle area: %.6f" % circle_area
    print "Square area: %.6f" % square_area
    print "Minimum possible (circle completely inside): %.6f" % min(circle_area, square_area)