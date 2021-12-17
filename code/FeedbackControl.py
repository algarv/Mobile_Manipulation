from modern_robotics import *
import numpy as np 
from numpy import sin, cos, pi
import csv

'''
FUNCTION CALL:
FeedbackControl(Tse, Tse_ref, Tse_ref_next, Kp, Ki, x_err_acc, dt)

Function test call commented below
'''

def FeedbackControl(Tse, Tse_ref, Tse_ref_next, Kp, Ki, x_err_acc, dt):
    '''
    Input:
        Tse: The current actual end-effector configuration X
        Tse_ref: The current end-effector reference configuration Xd
        Tse_ref_next: The end-effector reference configuration at the next timestep in the reference trajectory
        Kp, Ki: The PI gain matrices
        x_err_acc: Accumulated error
        dt: timestep
    Output:
        Ve: The commanded end-effector twist expressed in the end-effector frame {e}
    '''
    x_err_se3 = MatrixLog6(TransInv(Tse) @ Tse_ref)
    x_err = se3ToVec(x_err_se3)
    x_err_int = x_err_acc + x_err * dt

    
    Vd_se3 = (MatrixLog6(TransInv(Tse_ref) @ Tse_ref_next))/dt
    Vd = se3ToVec(Vd_se3)

    Ve = Adjoint(TransInv(Tse) @ Tse_ref) @ Vd + (Kp @ x_err) + (Ki @ x_err_int)

    return Ve, x_err_int, x_err

##Function test call##

# Tse = np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])
# Tse_ref = np.array([[0,0,1,0.5], [0,1,0,0], [-1,0,0,0.5], [0,0,0,1]])
# Tse_ref_next = np.array([[0,0,1,0.6], [0,1,0,0], [-1,0,0,0.3], [0,0,0,1]])

# Kp = 0 * np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
# Ki = 0 * np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
# x_err_acc = ([0,0,0,0,0,0])
# dt = .01

# print(FeedbackControl(Tse, Tse_ref, Tse_ref_next, Kp, Ki, x_err_acc, dt))