from modern_robotics import *
import numpy as np 
from numpy import sin, cos, pi
import csv

'''
FUNCTION CALL:
TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k)

Note: Update final line of the function with a valid file path for the output csv file
'''

#Initial position of the end effector in the world frame (estimated, will change depending on the robot's initial configuration)
Tse_init = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, .3],
                    [0, 0, 0, 1]])

#Initial and final block positions in the world frame (given)
Tsc_init = np.array([[1, 0, 0, 1],
                    [0, 1, 0, 0],
                    [0, 0, 1, .025],
                    [0, 0, 0, 1]])

Tsc_final = np.array([[0, 1, 0, 0],
                    [-1, 0, 0, -1],
                    [0, 0, 1, .025],
                    [0, 0, 0, 1]])

#Position of the end effector relative to the cube for grasping. The cube will be grasped at a 45 degree angle.
Tce_grasp = np.array([[cos(-5*pi/4), 0, sin(-5*pi/4), 0],
                        [0,         1,      0, 0],
                        [-sin(-5*pi/4),  0, cos(-5*pi/4),.01],
                        [0, 0, 0, 1]])

Tce_standoff = np.array([[cos(-5*pi/4), 0, sin(-5*pi/4), 0],
                        [0,         1,      0, 0],
                        [-sin(-5*pi/4),  0, cos(-5*pi/4),.1],
                        [0, 0, 0, 1]])

k = 1 

def flatten(T,gripper_state):
    '''
    Inputs: 
        T: trajectory from the ScrewTrajectory function
        gripper_state: 0 (gripper open) or 1 (gripper closed)
    Outputs:
        A list with each rotation in the transform, followed by each translation in the transform, and finally the set gripper state
    '''
    return [T[0][0],T[0][1],T[0][2],T[1][0],T[1][1],T[1][2],T[2][0],T[2][1],T[2][2],T[0][3],T[1][3],T[2][3],gripper_state]

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
    '''
    Inputs:
        Tse_init: Initial configuration of the end-effector in the reference trajectory
        Tsc_init: Initial configuration of the cube
        Tsc_final: Desired configuration of the cube
        Tce_grasp: Configuration of the end-effector relative to the cube when it is grasping the cube
        Tce_standoff: Stand-off configuration of the end-effector above the cube before and after the grasp
        k: Frequency (number of trajectory reference configurations per .01 seconds)
    Outputs: 
        traj: A list of N T_se matrices, representing the end-effector configurations along the reference trajectory
    '''
    Tf = 500 * .01 # Time for each motion
    N = (Tf * k)/.01 # Number of steps for each motion
    method = 3 # Time scaling for the trajectory generation (cubic or quintic)

    traj = [flatten(Tse_init,0)]

    #Segment 1: Move to initial grasp stand-off
        
    Xstart = Tse_init
    Xend = np.matmul(Tsc_init,Tce_standoff)
    gripper_state = 0
    T_list = ScrewTrajectory(Xstart,Xend,Tf,N,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row)
    
    
    #Segment 2: Move to initial grasp

    Xstart = np.matmul(Tsc_init,Tce_standoff)
    Xend = np.matmul(Tsc_init,Tce_grasp)
    gripper_state = 0
    T_list = ScrewTrajectory(Xstart,Xend,Tf/2,N/2,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row)

    #Segment 3: Close gripper

    Xstart = np.matmul(Tsc_init,Tce_grasp)
    Xend = np.matmul(Tsc_init,Tce_grasp)
    gripper_state = 1
    T_list = ScrewTrajectory(Xstart,Xend,.75,5,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row)

    #Segment 4: Return to initial grasp stand-off

    Xstart = np.matmul(Tsc_init,Tce_grasp)
    Xend = np.matmul(Tsc_init,Tce_standoff)
    gripper_state = 1
    T_list = ScrewTrajectory(Xstart,Xend,Tf/2,N/2,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row) 

    #Segment 5: Move to final stand-off

    Xstart = np.matmul(Tsc_init,Tce_standoff)
    Xend = np.matmul(Tsc_final,Tce_standoff)
    gripper_state = 1
    T_list = ScrewTrajectory(Xstart,Xend,Tf,N,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row) 
    

    #Segment 6: Move to final grasp

    Xstart = np.matmul(Tsc_final,Tce_standoff)
    Xend = np.matmul(Tsc_final,Tce_grasp)
    gripper_state = 1
    T_list = ScrewTrajectory(Xstart,Xend,Tf/2,N/2,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row) 


    #Segment 7: Open gripper

    Xstart = np.matmul(Tsc_final,Tce_grasp)
    Xend = np.matmul(Tsc_final,Tce_grasp)
    gripper_state = 0
    T_list = ScrewTrajectory(Xstart,Xend,.75,5,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row)

    #Segment 8: Move to final stand-off

    Xstart = np.matmul(Tsc_final,Tce_grasp)
    Xend = np.matmul(Tsc_final,Tce_standoff)
    gripper_state = 0
    T_list = ScrewTrajectory(Xstart,Xend,Tf/2,N/2,method)

    for T in T_list:
        row = flatten(T,gripper_state)
        traj.append(row) 

    #np.savetxt("/home/algarverick/ME_449/FinalProject/traj",traj,delimiter=",")
    #print('Trajectory saved to CSV file')
    return traj
