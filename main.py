#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('build/rrt_connect')
    rrt = RaveCreateModule(env,'rrt_module')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
  
    #set start config
    # jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']

    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    # startconfig = [-0.15,0.075,-1.008,0,0,-0.11,0]
    startconfig = [-0.15,0.075,-1.008,-0.11,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    lmodel=databases.linkstatistics.LinkStatisticsModel(robot)
    if not lmodel.load():
        lmodel.autogenerate()
    # lmodel.setRobotResolutions(0.01)
    lmodel.setRobotWeights()

    with env:
        goalconfig = [0.449,-0.201,-0.151,-0.11,0,-0.11,0]

        # print dir(robot)
        print 'calling rrt now'
        # biases = []
        # for i in range(1, 97, 5):
        #     biases.append(i)
        # for bias in biases:
        # rrt.SendCommand('setbias ' + str(bias))    
        # command = 'birrt ' + str(goalconfig)
        # for i in range(10):
        # command = 'birrt ' + str(goalconfig)
        command = 'rrtconnect ' + str(goalconfig)
        rrt.SendCommand(command)
        
        print 'rrt ended'
        ### END CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

