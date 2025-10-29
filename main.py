import numpy as np
import matplotlib.pyplot as plt

from modern_robotics import FKinBody,IKinBody
from FeedbackControl import FeedbackControl
from NextState import NextState
from TrajectoryGenerator import TrajectoryGenerator




Kp = 2.25
Ki = 1.75 # 越大误差越光滑
delta_t = 0.01
max_limit = 1000
jointlimit = 0

configs = []


# 初始
config = np.array([0,0,0,
                   0,0,0,0,0,
                   0,0,0,0,
                   0])

configs.append(config)






# 参考轨迹初始位姿
T_se_initial = np.array([
    [0,0,1,0],
    [0,1,0,0],
    [-1,0,0,0.5],
    [0,0,0,1]
])



T_sc_initial= np.array([
    [1,0,0,1],
    [0,1,0,0],
    [0,0,1,0.025],
    [0,0,0,1]
])


T_sc_final= np.array([
    [0,1,0,0],
    [-1,0,0,-1],
    [0,0,1,0.025],
    [0,0,0,1]
])

# 绕{c}的y轴旋转 theta
theta = np.pi/2 + np.pi/4

T_ce_grasp= np.array([
    [np.cos(theta), 0, np.sin(theta), 0],
    [0, 1, 0, 0],
    [-np.sin(theta), 0, np.cos(theta), 0],
    [0, 0, 0, 1]
])


T_ce_standoff = np.array([
    [np.cos(theta), 0, np.sin(theta), 0],
    [0, 1, 0, 0],
    [-np.sin(theta), 0, np.cos(theta), 0.05],
    [0, 0, 0, 1]
])





trajectory = TrajectoryGenerator(T_se_initial, T_sc_initial,T_sc_final, T_ce_grasp, T_ce_standoff, k=1)


X_err_history = []
time_history = []
step_idx = 0


for i in range(len(trajectory)-1):

    X_d = np.array([
        [trajectory[i][0], trajectory[i][1], trajectory[i][2],trajectory[i][9]],
        [trajectory[i][3], trajectory[i][4], trajectory[i][5], trajectory[i][10]],
        [trajectory[i][6], trajectory[i][7], trajectory[i][8], trajectory[i][11]],
        [0, 0, 0, 1]
    ])
    X_d_next = np.array([
        [trajectory[i+1][0], trajectory[i+1][1], trajectory[i+1][2],trajectory[i+1][9]],
        [trajectory[i+1][3], trajectory[i+1][4], trajectory[i+1][5], trajectory[i+1][10]],
        [trajectory[i+1][6], trajectory[i+1][7], trajectory[i+1][8], trajectory[i+1][11]],
        [0, 0, 0, 1]
    ])

    speeds, V_e, X_err = FeedbackControl(configs[i],X_d,X_d_next,Kp,Ki,delta_t, jointlimit)
    joints_speed = speeds[4:]
    wheels_speed = speeds[:4]

    new_config = NextState(configs[i][:12],joints_speed,wheels_speed,delta_t,max_limit)
    new_config = np.append(new_config,trajectory[i][-1])
    configs.append(new_config)
    
    X_err_history.append(X_err.flatten())  # X_err 为长度6的向量
    time_history.append(step_idx * delta_t)
    step_idx += 1


                                                             
try:
    X_err_arr = np.array(X_err_history)  # shape (N,6)
    if X_err_arr.size > 0:
        labels = ['ω_x','ω_y','ω_z','v_x','v_y','v_z']  # modern_robotics.se3ToVec 顺序
        fig, axs = plt.subplots(6,1, figsize=(8,10), sharex=True)
        for j in range(6):
            axs[j].plot(time_history, X_err_arr[:,j], linewidth=1)
            axs[j].set_ylabel(labels[j])
            axs[j].grid(True)
        axs[-1].set_xlabel('time (s)')
        plt.suptitle('X_err components vs time')
        plt.tight_layout(rect=[0,0,1,0.96])
        plt.savefig('X_err.png', dpi=150)
        # plt.show()
except Exception as e:
    print('绘图 X_err 失败:', e)



np.savetxt("CoppeliaSim.csv",configs, delimiter=",", fmt="%s")

# np.savetxt("trajectory.csv",trajectory, delimiter=",", fmt="%s")