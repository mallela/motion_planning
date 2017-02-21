#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import math
import Queue
import astar
#### END OF YOUR IMPORTS ####

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

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####
        # n_stepSize = .05
        n_stepSize = .1
        # n_connectivity = 4
        n_connectivity = 8
        # n_stepSize_deg=0
        n_stepSize_deg=-pi/4
        # c_heuristic = 'manhattan'
        c_heuristic = 'eucledian'
        handles=[]
        handles.append(env.plot3(points=array(tuple((goalconfig[0],goalconfig[1],0.05))),
                           pointsize=.04,
                           colors=array((0,1,0)),drawstyle=1))  
        trajectory= astar.astar(env, goalconfig, robot, n_stepSize, n_connectivity, c_heuristic, n_stepSize_deg)
        # robot.SetActiveDOFValues((1,2,.05))
        # print env.CheckCollision(robot)
        for pt in trajectory:
            handles.append(env.plot3(points=array(tuple((pt[0],pt[1],.05))),
                   pointsize=.04,
                   colors=array((0,0,0)),drawstyle=1)) 

        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. 
        ###### The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.



        #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)

        #### Draw the X and Y components of the configurations explored by A*
        if trajectory:

            traj = RaveCreateTrajectory(env, '')
            config = robot.GetActiveConfigurationSpecification('linear')
            config.AddDeltaTimeGroup()

            traj.Init(config)

            finalPath = list()

            for i, point in enumerate(trajectory):
               node = trajectory[i]

               finalPath.append([node[0], node[1], node[2], i*0.003])

            for i, p in enumerate(finalPath):
               traj.Insert(i, p, config, True)

            robot.GetController().SetPath(traj)
        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. 
        #### You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);

        #### END OF YOUR CODE ###
        print robot.GetActiveDOFValues()
    waitrobot(robot)

    raw_input("Press enter to exit...")

