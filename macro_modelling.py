import numpy as np

MAX_RUNTIME = 3 * 60 
DELTA_T = 1 / 3
DEATH_TIME = 2 * 60
DEATH_STEPS = int(DEATH_TIME / DELTA_T)

for i in range(2):

    if i == 0: # TYPE 1 ROBOTS
        TOTAL_ROBOTS = 2
        P_ALLOC =  1
        tau_work = 43
        tau_travel = round(0.6525/0.5/DELTA_T)
    else: # TYPE 2 ROBOTS
        TOTAL_ROBOTS = 3
        P_ALLOC = 1
        tau_work = 37
        tau_travel = round(0.6525/0.5/DELTA_T)

    N_MOVING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
    N_WAITING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
    N_WORKING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
    N_WAITING[0] = TOTAL_ROBOTS

    for k in range(int(MAX_RUNTIME/DELTA_T)):
        
        active_steps_so_far = np.count_nonzero((N_MOVING[:k] + N_WORKING[:k]) > 0.001)
        
        if active_steps_so_far > DEATH_STEPS:
            break
        
        # 1. Flow: Moving -> Working
        if k - tau_travel < 0:
            term_arrival_work = 0
        else:
            k_delta_t = k - tau_travel
            term_arrival_work = P_ALLOC * N_WAITING[k_delta_t]

        # Update Moving
        N_MOVING[k+1] = N_MOVING[k] + P_ALLOC * N_WAITING[k] - term_arrival_work
        
        # 2. Flow: Working -> Waiting
        if k - tau_work - tau_travel < 0:
            term_finish_work = 0
        else:
            k_delta_work = k - tau_work - tau_travel
            term_finish_work = P_ALLOC * N_WAITING[k_delta_work]

        # Update Working
        N_WORKING[k+1] = N_WORKING[k] + term_arrival_work - term_finish_work
        
        # 3. Update Waiting (Conservation of Mass)
        N_WAITING[k+1] = TOTAL_ROBOTS - N_WORKING[k+1] - N_MOVING[k+1]

    N_ACTIVE = N_MOVING + N_WORKING
    
    if i == 0:
        average_type1 = np.mean(N_ACTIVE)
    else:
        average_type2 = np.mean(N_ACTIVE)

print("Average for type 1: ", average_type1)
print("Average for type 2: ", average_type2)
print("Overall average:    ", (average_type1+average_type2))
