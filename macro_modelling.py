import numpy as np

MAX_RUNTIME = 3*60 
DELTA_T = 1/3

for i in range(2):
    if i==0: #type 1 robots
        P_ALLOC = 0.5
        N_MOVING = [0]
        N_WAITING = [2]
        N_WORKING = [0]
        tau_work = 43
        tau_travel = round(0.6525/0.5/DELTA_T)
        for k in range(MAX_RUNTIME/DELTA_T):
            if k - tau_travel < 0:
                k_delta_t = 0
            else:
                k_delta_t = k - tau_travel
            N_MOVING.append(N_MOVING[k] + P_ALLOC * N_WAITING[k] - P_ALLOC * N_WAITING[k_delta_t])
            if k - tau_work - tau_travel < 0:
                k_delta_work = 0
            else:
                k_delta_work = k - tau_work - tau_travel
            N_WORKING.append(N_WORKING[k] + P_ALLOC * N_WAITING[k_delta_t] - P_ALLOC * N_WAITING[k_delta_work])
            N_WAITING.append(2 - N_WORKING[k] - N_MOVING[k]) 
    else:
        P_ALLOC = 0.5
        N_MOVING = [0]
        N_WAITING = [3]
        N_WORKING = [0]
        tau_work = 37
        tau_travel = round(0.6525/0.5/DELTA_T)
        for k in range(MAX_RUNTIME/DELTA_T):
            if k - tau_travel < 0:
                k_delta_t = 0
            else:
                k_delta_t = k - tau_travel
            N_MOVING.append(N_MOVING[k] + P_ALLOC * N_WAITING[k] - P_ALLOC * N_WAITING[k_delta_t])
            if k - tau_work - tau_travel < 0:
                k_delta_work = 0
            else:
                k_delta_work = k - tau_work - tau_travel
            N_WORKING.append(N_WORKING[k] + P_ALLOC * N_WAITING[k_delta_t] - P_ALLOC * N_WAITING[k_delta_work])
            N_WAITING.append(2 - N_WORKING[k] - N_MOVING[k]) 