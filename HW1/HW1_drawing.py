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
    handles = []
    pts=[]
    for itr in range(0,360,10):
        if len(pts)<35:
            theta= itr*3.14/180
            pts.append((4.5*cos(theta),4.5*sin(theta),0))
    pts=tuple(pts)
    table_list1 = ['Table5','Table6']
    table_list2 = ['Table1','Table2','Table3','Table4']
    y=.62
    x=.32
    for table in table_list1:
        T = env.GetKinBody(table).GetTransform()
        t = ((T[0,3]-x,T[1,3]-y,T[2,3]),(T[0,3]-x,T[1,3]+y,T[2,3]), (T[0,3]+x,T[1,3]+y,T[2,3]), (T[0,3]+x,T[1,3]-y,T[2,3]), (T[0,3]-x,T[1,3]-y,T[2,3]))
        handles.append(env.drawlinestrip(points=array(t), linewidth=5.0, colors=array((1,0,0))))

    for table in table_list2:
        T = env.GetKinBody(table).GetTransform()
        t = ((T[0,3]-y,T[1,3]-x,T[2,3]),(T[0,3]-y,T[1,3]+x,T[2,3]), (T[0,3]+y,T[1,3]+x,T[2,3]), (T[0,3]+y,T[1,3]-x,T[2,3]), (T[0,3]-y,T[1,3]-x,T[2,3]))
        handles.append(env.drawlinestrip(points=array(t), linewidth=5.0, colors=array((1,0,0))))
        
    handles.append(env.plot3(points=array(pts),
                               pointsize=.04,
                               colors=array((0,0,1)),drawstyle=1))
    # handles.append(env.plot3(points=array((T[1,3],T[0,3],T[2,3])), pointsize=15.0, colors=array((0,1,0))))
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
