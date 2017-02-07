#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    #### YOUR CODE HERE ####
    # load puma and place it next to pr2
    env.Load('robots/puma.robot.xml')
    puma = env.GetRobots()[1]
    puma.SetTransform([[1., 0., 0., -3.4], 
        [ 0.   , 1. ,   0. ,  -.3 ],
        [ 0.,  0.,   1.,0.05],
        [ 0.,0., 0. , 1.  ]])
    
    # move left arm of pr2 to touch puma
    with env:
        jointnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 
        'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 
        'l_wrist_roll_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.5,-1.4,0.0,
         -.5, 0.0,1.0,
         0.0]);
        # robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    print 'Is PR2 touching Puma with its left gripper?', env.CheckCollision(robot, puma)
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

