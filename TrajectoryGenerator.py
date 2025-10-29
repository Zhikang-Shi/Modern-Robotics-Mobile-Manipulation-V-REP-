import numpy as np
from modern_robotics.core import ScrewTrajectory

def matrix_to_vector(T_se):
    return np.array([
        T_se[0][0], T_se[0][1], T_se[0][2], T_se[1][0], T_se[1][1], T_se[1][2], T_se[2][0], T_se[2][1], T_se[2][2], T_se[0][3], T_se[1][3], T_se[2][3]
    ])



def TrajectoryGenerator(T_se_initial, T_sc_initial,T_sc_final, T_ce_grasp, T_ce_standoff, k=1):
    trajectory = []

    # 1.将夹具从其初始配置移动到块上方几厘米的“隔离”配置的轨迹。T_ce_standoff
    Tf1 = 5
    T_se_standoff = T_sc_initial @ T_ce_standoff
    T_1 = ScrewTrajectory(T_se_initial,T_se_standoff, Tf1, Tf1/0.01, 3)
    
    for i in range(int(Tf1/0.01)):
        T_vec = matrix_to_vector(T_1[i])
        trajectory.append(np.append(T_vec,0))

    # 2.将夹具向下移动到抓取位置的轨迹
    Tf2 = 2
    T_se_grasp = T_sc_initial @ T_ce_grasp 
    T_2 = ScrewTrajectory(T_se_standoff,T_se_grasp, Tf2, Tf2/0.01, 3)
    for i in range(int(Tf2/0.01)):
        T_vec = matrix_to_vector(T_2[i])
        trajectory.append(np.append(T_vec,0))

    # 3.闭合夹具。
    Tf3 = 1.0
    T_se_last = trajectory[-1]
    T_se_last[12] = 1
    for i in range(int(Tf3/0.01)):
        trajectory.append(T_se_last)


    # 4.将夹具移回“对峙”配置的轨迹。
    Tf4 = 2
    T_4 = ScrewTrajectory(T_se_grasp, T_se_standoff, Tf4, Tf4/0.01, 3)
    for i in range(int(Tf4/0.01)):
        T_vec = matrix_to_vector(T_4[i])
        trajectory.append(np.append(T_vec,1))

    # 5.将夹具移动到最终配置上方的“对峙”配置的轨迹。
    Tf5 = 8
    T_se_standoff_final = T_sc_final @ T_ce_standoff
    T_5 = ScrewTrajectory(T_se_standoff,T_se_standoff_final, Tf5, Tf5/0.01, 3)
    for i in range(int(Tf5/0.01)):
        T_vec = matrix_to_vector(T_5[i])
        trajectory.append(np.append(T_vec,1))


    # 6.将夹具移动到物体最终配置的轨迹。
    Tf6 = 2
    T_se_grasp_final = T_sc_final @ T_ce_grasp 
    T_6 = ScrewTrajectory(T_se_standoff_final,T_se_grasp_final, Tf6, Tf6/0.01, 3)
    for i in range(int(Tf6/0.01)):
        T_vec = matrix_to_vector(T_6[i])
        trajectory.append(np.append(T_vec,1))


    # 7.夹持器的打开。
    Tf7 = 1.0
    T_se_last_lose = trajectory[-1]
    T_se_last_lose[12] = 0
    for i in range(int(Tf7/0.01)):
        trajectory.append(T_se_last_lose)

    # 8.将夹具移回“对峙”配置的轨迹。
    Tf8 = 2
    T_se_grasp_final = T_sc_final @ T_ce_grasp 
    T_8 = ScrewTrajectory(T_se_grasp_final,T_se_standoff_final, Tf8, Tf8/0.01, 3)
    for i in range(int(Tf8/0.01)):
        T_vec = matrix_to_vector(T_8[i])
        trajectory.append(np.append(T_vec,0))

    return trajectory





# T_se_initial = np.array([
#     [0,0,1,0],
#     [0,1,0,0],
#     [-1,0,0,0.5],
#     [0,0,0,1]
# ])


# T_sc_initial= np.array([
#     [1,0,0,1],
#     [0,1,0,0],
#     [0,0,1,0.025],
#     [0,0,0,1]
# ])


# T_sc_final= np.array([
#     [0,1,0,0],
#     [-1,0,0,-1],
#     [0,0,1,0.025],
#     [0,0,0,1]
# ])

# # 绕{c}的y轴旋转 theta
# theta = np.pi/2 + np.pi/4

# T_ce_grasp= np.array([
#     [np.cos(theta), 0, np.sin(theta), 0],
#     [0, 1, 0, 0],
#     [-np.sin(theta), 0, np.cos(theta), 0],
#     [0, 0, 0, 1]
# ])


# T_ce_standoff = np.array([
#     [np.cos(theta), 0, np.sin(theta), 0],
#     [0, 1, 0, 0],
#     [-np.sin(theta), 0, np.cos(theta), 0.05],
#     [0, 0, 0, 1]
# ])



# print(T_sc_initial @ T_ce_standoff)


# trajectory = TrajectoryGenerator(T_se_initial, T_sc_initial,T_sc_final, T_ce_grasp, T_ce_standoff, k=1)

# np.savetxt("trajectory.csv",trajectory, delimiter=",", fmt="%s")




