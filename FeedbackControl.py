import numpy as np
from modern_robotics import MatrixLog6,se3ToVec,Adjoint,TransInv,JacobianBody,FKinBody



def testjointLimits(config,J_arm):
    if config[5] > -0.2:
        J_arm[:,2] = np.array([0,0,0,0,0,0])
    if config[6] > -0.2:
        J_arm[:,3] = np.array([0,0,0,0,0,0])


def FeedbackControl(config, X_d, X_d_next, Kp, Ki, delta_t, joint_limit):

    T_sb = np.array([
        [np.cos(config[0]), -np.sin(config[0]), 0, config[1]],
        [np.sin(config[0]), np.cos(config[0]), 0, config[2]],
        [0, 0, 1, 0.0963],
        [0, 0, 0, 1]
    ])
    T_b0 = np.array([
        [1, 0, 0, 0.1662],
        [0, 1, 0, 0],
        [0, 0, 1, 0.0026],
        [0, 0, 0, 1]
    ])

    M_0e = np.array([
        [1, 0, 0, 0.033],
        [0, 1, 0, 0],
        [0, 0, 1, 0.6546],
        [0, 0, 0, 1]
    ])
    Blist = np.array([
        [0, 0, 1, 0, 0.033, 0],
        [0, -1, 0, -0.5076, 0, 0],
        [0, -1, 0, -0.3526, 0, 0],
        [0, -1, 0, -0.2176, 0, 0],
        [0, 0, 1, 0, 0, 0]
    ]).T

    thetalist = np.array(
        [config[3],config[4],config[5],config[6],config[7]]
    )

    T_0e = FKinBody(M_0e, Blist, thetalist)

    T_se = T_sb @ T_b0 @ T_0e

    X = T_se

    # print(X)

    V_d_hat = (1/delta_t) * MatrixLog6( TransInv(X_d) @ X_d_next)
    V_d = se3ToVec(V_d_hat)

    X_err =  se3ToVec(MatrixLog6(TransInv(X) @ X_d))

    # 静态变量用于存储累积误差
    if not hasattr(FeedbackControl, "integral_error"):
        FeedbackControl.integral_error = np.zeros(6)
    # 更新积分项
    FeedbackControl.integral_error += X_err * delta_t


    V_e = Adjoint(TransInv(X) @ X_d) @ V_d + Kp * X_err + Ki * FeedbackControl.integral_error

    J_b = JacobianBody(Blist, thetalist)

    r = 0.0475
    l = 0.2350
    w = 0.1500
    F = r/4 * np.array([
        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
        [1, 1, 1, 1],
        [-1, 1, -1,1]
    ])

    F6 = np.zeros((6,4))

    F6[2:5, : ] = F
    
    J_base =  Adjoint(TransInv(T_0e) @ TransInv(T_b0)) @ F6

    #  testjointLimits(config,J_b)

    # print(J_b)

    J_e = np.hstack((J_base, J_b))

    
    speeds = np.linalg.pinv(J_e,1e-2) @ V_e


    return speeds, V_e, X_err


# config = np.array([0,0,0,0,0,0.2,-1.6,0])

# X_d = np.array([
#     [0, 0, 1, 0.5],
#     [0, 1, 0, 0],
#     [-1, 0, 0, 0.5],
#     [0, 0, 0, 1]
# ])

# X_d_next = np.array([
#     [0, 0, 1, 0.6],
#     [0, 1, 0, 0],
#     [-1, 0, 0, 0.3],
#     [0, 0, 0, 1]
# ])



# Kp = 1
# Ki = 0
# delta_t = 0.01


# print(FeedbackControl(config,X_d,X_d_next,Kp,Ki,delta_t))



