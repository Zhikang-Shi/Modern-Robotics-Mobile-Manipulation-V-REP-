import numpy as np



def NextState(config,joints_speed,wheels_speed,delta_t,max_limit):

    wheels_speed = np.clip(wheels_speed, -max_limit, max_limit)
    joints_speed = np.clip(joints_speed,-max_limit, max_limit)


    arm_angle = config[3:8] + joints_speed * delta_t
    wheels_angle = config[8:12] + wheels_speed * delta_t

    delta_theta = wheels_speed * delta_t 


    r = 0.0475
    l = 0.2350
    w = 0.1500
    F = (r/4) * np.array([
        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
        [1, 1, 1, 1],
        [-1, 1, -1,1]
    ])
    V_b = F @ delta_theta

    w_bz = V_b[0]
    if w_bz==0:
        delta_qb = np.array([0,V_b[1],V_b[2]])
    else:
        delta_qb = np.array(
            [w_bz, (V_b[1]*np.sin(w_bz) + V_b[2]*(np.cos(w_bz) - 1))/w_bz, (V_b[2]*np.sin(w_bz) + V_b[1]*(1-np.cos(w_bz)))/w_bz]
        )
    delta_q = np.array([
        [1, 0,0],
        [0,np.cos(config[0]), -np.sin(config[0])],
        [0, np.sin(config[0]), np.cos(config[0])]
    ]) @ delta_qb

    new_q = config[:3] + delta_q

    
    new_config = np.hstack((new_q, arm_angle, wheels_angle))

    return new_config


# config = np.array([0,1,0,0,0,0,0,0,0,0,0,0])
# wheels_speed = np.array([-10,10,-10,10])
# joints_speed = np.array([1,2,3,4,5])
# delta_t = 0.01


# # print(NextState(config,joints_speed,wheels_speed,delta_t,5))


# configs = []
# configs.append(config)
# T = 1.0

# for i in range(int(T/delta_t)):
#     state = configs[i]
#     new_state = NextState(state,joints_speed,wheels_speed,delta_t,5)
#     configs.append(new_state)
    


# np.savetxt("configs.csv", configs, delimiter=",", fmt="%s")


