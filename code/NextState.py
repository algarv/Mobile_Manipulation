from modern_robotics import *
import numpy as np 
from numpy import sin, cos, pi
import csv

'''
FUNCTION CALL:
NextState(Tsr, V_joints, dt, speed_limit)

Function test call commented below
'''

def NextState(Tsr, V_joints, dt, speed_limit):
    '''
    Inputs:

        Tsr: A 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
        V_joints: A 9-vector of controls indicating the arm joint speeds \dot{\theta} (5 variables) and the wheel speeds u (4 variables).
        dt: timestep
        V_max: A positive real value indicating the maximum angular speed of the arm joints and the wheels. For example, if this value is 12.3, the angular speed of the wheels and arm joints is limited to the range [-12.3 radians/s, 12.3 radians/s]. Any speed in the 9-vector of controls that is outside this range will be set to the nearest boundary of the range. If you don't want speed limits, just use a very large number. If you prefer, your function can accept separate speed limits for the wheels and arm joints.
        dt: time step
        speed_limt: maximum rotational velocities of the wheel and arm joints

    Outputs: 

        Tsr_new = A 12-vector representing the configuration of the robot time Δt later
    '''

    l = 0.47/2
    w = 0.3/2
    r = 0.0475

    Tsr_new = Tsr.copy()
    for i in range(3,8):
        Tsr_new[i] = Tsr[i] + V_joints[i-3] * dt #new_joint_angles = (old arm joint angles) + (joint speeds) * Δt
    
    for i in range(8,12):
        Tsr_new[i] = Tsr[i] + V_joints[i-3] * dt #new_wheel_angles = (old wheel angles) + (wheel speeds) * Δt

    dTheta = []
    for i in range(8,12):
        '''        
        If the difference between the new wheel joint angle and the old joint angle divided by the time step is
        above the speed limit, set the new joint angle to the maximum delta value
        '''
        if (Tsr_new[i]-Tsr[i])/dt > speed_limit:
            dTheta.append(speed_limit*dt)
        elif (Tsr_new[i]-Tsr[i])/dt < -speed_limit:
            dTheta.append(-speed_limit*dt)
        else:
            dTheta.append(Tsr_new[i]-Tsr[i])
    
    F = (r/4) * np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
    Vb = F @ dTheta

    for i in range(0,3):
        Tsr_new[i] = Tsr[i] + Vb[i]

    return Tsr_new

##Function test call##

# Tsr = [0,0,0,0,0,0,0,0,0,0,0,0]
# chassis = [Tsr + [0]]
# for i in range(100):
#     Tsr_old = Tsr.copy()
#     Tsr = NextState(Tsr_old,[0,0,0,0,0,-10,10,10,-10],.01)
#     chassis.append(Tsr + [0])

# np.savetxt("/home/algarverick/ME_449/FinalProject/NextState_Test",chassis,delimiter=",")
