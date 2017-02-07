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
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    print 'Is PR2 touching Puma?', env.CheckCollision(robot, puma)

    # basemanip = interfaces.BaseManipulation(robot,plannername=None)
    # Tgoal = array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
    # Tgoal = numpy.array([[  4.17106133e-02 , -1.99037457e-16 , -9.99129734e-01 , -3.4],
    #         [ -2.71121496e-17 ,  1.00000000e+00 , -2.00342673e-16 , -.7], 
    #         [  9.99129734e-01 ,  3.54449706e-17  , 4.17106133e-02  , 1.0], 
    #         [  0.00000000e+00  , 0.00000000e+00  , 0.00000000e+00   ,1.00000000e+00]])
    # # res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)

    # # print robot.GetDOF()
    # # RaveSetDebugLevel(DebugLevel.Debug)
    # manipprob = interfaces.BaseManipulation(robot)
    # print (robot.GetActiveManipulator().GetJoint()), 40*'_'
    # exit()
    # # link_touch=  env.GetKinBody('PR2').GetActiveManipulator()#('l_gripper_l_finger_tip_link')
    # # robot.SetActiveManipulator('leftarm_camera') # set the manipulator to leftarm + torso
    # manipprob.MoveManipulator(goal=[1.5,1.5,1.5,1.3,1.6]) # call motion planner with goal joint angles
    # waitrobot(robot)
    # robot.WaitForController(0) # wait
    # print link_touch
    # exit()
    # ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    # if not ikmodel.load():
    #     ikmodel.autogenerate()
        
    # with robot: # lock environment and save robot state
    #     # robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
    #     Tee = manip.GetEndEffectorTransform() # get end effector
    #     ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    #     sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
    # # manipprob = interfaces.BaseManipulation(robot)   

    # res = manipprob.MoveToHandPosition(matrices=[T],seedik=10) # call motion planner with goal joint angles
    # robot.WaitForController(0) # wait


    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

