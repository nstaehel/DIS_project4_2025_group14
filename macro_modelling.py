import numpy as np

MAX_RUNTIME = 3*60 
DELTA_T = 1/3


for i in range(2):
    if i==0: #type 1 robots
        P_ALLOC = 0.5
        N_MOVING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
        N_WAITING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
        N_WORKING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
        N_WAITING[0]=2
        tau_work = 43
        tau_travel = round(0.6525/0.5/DELTA_T)
        for k in range(int(MAX_RUNTIME/DELTA_T)):
            if k - tau_travel < 0:
                k_delta_t = 0
            else:
                k_delta_t = k - tau_travel
            N_MOVING[k+1] = N_MOVING[k] + P_ALLOC * N_WAITING[k] - P_ALLOC * N_WAITING[k_delta_t]
            if k - tau_work - tau_travel < 0:
                k_delta_work = 0
            else:
                k_delta_work = k - tau_work - tau_travel
            N_WORKING[k+1] = N_WORKING[k] + P_ALLOC * N_WAITING[k_delta_t] - P_ALLOC * N_WAITING[k_delta_work]
            N_WAITING[k+1] = 2 - N_WORKING[k] - N_MOVING[k]
        N_ACTIVE = N_MOVING + N_WORKING
        average_type1 = np.mean(N_ACTIVE)
    else:
        P_ALLOC = 0.5
        N_MOVING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
        N_WAITING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
        N_WORKING = np.zeros(int(MAX_RUNTIME/DELTA_T)+2)
        N_WAITING[0]=3
        tau_work = 37
        tau_travel = round(0.6525/0.5/DELTA_T)
        for k in range(int(MAX_RUNTIME/DELTA_T)):
            if k - tau_travel < 0:
                k_delta_t = 0
            else:
                k_delta_t = k - tau_travel
            N_MOVING[k+1] = N_MOVING[k] + P_ALLOC * N_WAITING[k] - P_ALLOC * N_WAITING[k_delta_t]
            if k - tau_work - tau_travel < 0:
                k_delta_work = 0
            else:
                k_delta_work = k - tau_work - tau_travel
            N_WORKING[k+1] = N_WORKING[k] + P_ALLOC * N_WAITING[k_delta_t] - P_ALLOC * N_WAITING[k_delta_work]
            N_WAITING[k+1] = 2 - N_WORKING[k] - N_MOVING[k]
        N_ACTIVE = N_MOVING + N_WORKING
        average_type2 = np.mean(N_ACTIVE)
        
print("Average for type 1: ", average_type1," ; Average for type 2: ", average_type2, "; Overall average: ", 0.5*(average_type1+average_type2))
