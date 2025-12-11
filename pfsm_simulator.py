import random
import numpy as np

MAX_RUNTIME = 3*60 
DEATH_TIME = 2*60
DELTA_T = 1/3
P_ALLOCATION_1 = 0.1189/4 #CHANGE WITH VALUE FROM SUB_MICRO MODEL
P_ALLOCATION_2 = 0.344/4
TRAVEL_TIME = 0.625/0.5 #computed from simulator_distance
ACTIVE_TIME_ARRAY = []

for i in range(5):
    if i in [0,1]:
        STATE = 0 #state 0 is wait, state 1 is move, state 2 is working, state 3 is DEAD
        T_work = 13/3
        active_time = 0   
        for k in np.arange(0,MAX_RUNTIME,DELTA_T):
            if active_time >= DEATH_TIME:
                STATE = 3
            else:
                if STATE == 0:
                    r = random.random()
                    if r < P_ALLOCATION_1:
                        STATE = 1
                        TIMER = TRAVEL_TIME / DELTA_T
                elif STATE == 1:
                    TIMER -= 1
                    active_time += DELTA_T
                    if TIMER <= 0:
                        STATE = 2
                        TIMER = T_work / DELTA_T
                elif STATE == 2:
                    active_time += DELTA_T
                    TIMER -= 1
                    if TIMER <= 0:
                        STATE = 0
        ACTIVE_TIME_ARRAY.append(active_time)
    else:
        STATE = 0 #state 0 is wait, state 1 is move, state 2 is working, state 3 is DEAD
        T_work = 11/3
        active_time = 0   
        for k in np.arange(0,MAX_RUNTIME,DELTA_T):
            if active_time >= DEATH_TIME:
                STATE = 3
            else:
                if STATE == 0:
                    r = random.random()
                    if r < P_ALLOCATION_2:
                        STATE = 1
                        TIMER = TRAVEL_TIME / DELTA_T
                elif STATE == 1:
                    TIMER -= 1
                    active_time += DELTA_T
                    if TIMER <= 0:
                        STATE = 2
                        TIMER = T_work / DELTA_T
                elif STATE == 2:
                    active_time += DELTA_T
                    TIMER -= 1
                    if TIMER <= 0:
                        STATE = 0
        ACTIVE_TIME_ARRAY.append(active_time)


print("Average active time (s) :", np.mean(np.array(ACTIVE_TIME_ARRAY)))
