#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
import timeit
import math
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from dsr_msgs.msg import Tracker
from std_msgs.msg import Header
from math import pi
from time import sleep
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

tracker = Tracker()


# Some of Contstants
DISTANCE_AWAY_FROM_TARGET = 0.2

EPSILON = 0.0000001
DEG2RAD = 3.141592 / 180.0

ROLL = 180.0 * DEG2RAD
PITCH = 0.0 * DEG2RAD
YAW = 0.0 * DEG2RAD

# Set the tracker topic name
TRACKER_TOPIC = "cxy"
AR_MARKER_TOPIC = 'ar_pose_marker'

# initial joint angle in [rad]
INIT_JOINT_ANGLE = [0.0, 0.0, -90.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 90.0*DEG2RAD]

# end-effector poses for searching the target [m], [rad]
SEARCH_POSE = [ 
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
              ]


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('m1013_demo', anonymous=True)
    self.counter = 1
    self.start_flag = "false"
    self.offset_x = 0.0
    self.offset_y = 0.0

    self.waypoints= []
    self.pointx = []
    self.pointy = []
    self.phase = 1
    self.object_cnt = 0
    self.track_flag = False
    self.default_pose_flag = True
    self.cx = 400.0
    self.cy = 400.0
    self.points=[]

    rospy.Subscriber('R_007/ur_pnp', String, self.ur_PickPlace, queue_size=1)

    self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
    self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    self.robot = moveit_commander.RobotCommander(robot_description="dsr/robot_description", ns="dsr")

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    self.scene = moveit_commander.PlanningSceneInterface(ns="dsr")

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
    ## Change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the M1013:
    group_name = "arm"
    self.group = moveit_commander.MoveGroupCommander(group_name, robot_description="dsr/robot_description", ns="dsr")

    # Set the reference frame for pose targets, Set the M1013 arm reference frame accordingly
    # reference_frame = "/base_link"
    # self.gruop.set_pose_reference_frame(reference_frame)

    self.default_joint_states = self.group.get_current_joint_values()

    Q1 = [0.0, 0.0, -90.0*DEG2RAD, 0.0, -90.0*DEG2RAD, 90.0*DEG2RAD]  # change later based on the position of table

    self.default_joint_states = Q1


    # Allow replanning to increase the odds of a solution
    self.group.allow_replanning(True) 
    self.group.set_goal_position_tolerance(0.01)
    self.group.set_goal_orientation_tolerance(0.1)
    self.group.set_planning_time(0.1)
    self.group.set_max_acceleration_scaling_factor(.5)
    self.group.set_max_velocity_scaling_factor(.65)

    # We can also print the name of the end-effector link for this group:
    self.end_effector_link = self.group.get_end_effector_link()
   # print "============ End effector: %s" % self.end_effector_link

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
   # print "============ Robot Groups:", self.robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
   # print "============ Printing robot state"
   # print self.robot.get_current_state()


   # print "============ Moveit_Commander Setup Initialized !!!"
    
    self.go_to_joint_state()
   # print "Robot Pose Initialized!"


##############################################################################################################

  def ur_PickPlace(self,msg):
        self.start_flag = msg.data 

  def go_to_joint_state(self):
    joint_goal = self.group.get_current_joint_values()

    joint_goal[0] = INIT_JOINT_ANGLE[0]
    joint_goal[1] = INIT_JOINT_ANGLE[1]
    joint_goal[2] = INIT_JOINT_ANGLE[2]
    joint_goal[3] = INIT_JOINT_ANGLE[3]
    joint_goal[4] = INIT_JOINT_ANGLE[4]
    joint_goal[5] = INIT_JOINT_ANGLE[5]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


##############################################################################################################

#=============================== Tracking objects===============================

  def tracking_callback(self, msg):
      if (self.start_flag == "1"):

        self.track_flag = msg.flag1
        self.cx = msg.y
        self.cy = msg.x
        self.cz = msg.z 
        self.error_x = msg.error_y  
        self.error_y = msg.error_x 
        print self.track_flag
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1
            self.start_flag =  "-1" 
       # if (self.track_flag and -0.6 < self.waypoints[0].position.y and self.waypoints[0].position.y < 0.6):    
        if (self.track_flag):      
            self.execute()
            self.default_pose_flag = False
        else:
            if not self.default_pose_flag:
                self.track_flag = False
                self.execute()
                self.default_pose_flag = True

            if (self.counter==1 or self.counter == 6):
               self.group.set_joint_value_target(self.default_joint_states)
               # Set the internal state to the current state
               self.group.set_start_state_to_current_state()
               plan = self.group.plan()
               self.group.execute(plan)
               self.counter += 1
               self.start_flag =  "-1" 

#=============================== Try to searching the object=================================   Whether just need to rotate 6th axis is ok or not
            elif self.counter==2:
              while True:
                 start_pose = self.group.get_current_pose(self.end_effector_link).pose
                 wpose = deepcopy(start_pose)
                 waypoints1 = []
                 waypoints1.append(deepcopy(wpose))
                 wpose.position.x = -0.219
                 waypoints1.append(deepcopy(wpose))
                 self.group.set_start_state_to_current_state()
                 plan, fraction = self.group.compute_cartesian_path(waypoints1, 0.01, 0.0, True)
                 if 1-fraction < 0.2:
                    break
              self.group.execute(plan)
              sleep(0.5) # 0.5
              self.counter += 1

#===================================================================================== 
            elif self.counter==3:
              while True:
                 start_pose = self.group.get_current_pose(self.end_effector_link).pose
                 wpose = deepcopy(start_pose)
                 waypoints0 = []
                 waypoints0.append(deepcopy(wpose))
                 wpose.position.y -= 0.3
                 waypoints0.append(deepcopy(wpose))
                 self.group.set_start_state_to_current_state()
                 plan, fraction = self.group.compute_cartesian_path(waypoints0, 0.01, 0.0, True)
                 if 1-fraction < 0.2:
                    break
              self.group.execute(plan)
              sleep(0.5) #0.5
              self.counter += 1

#=======================================================================================
            elif self.counter==4:
              start_pose = self.group.get_current_pose(self.end_effector_link).pose
              wpose = deepcopy(start_pose)
              waypoints2 = []
              waypoints2.append(deepcopy(wpose))
              wpose.position.x = -0.409 
              wpose.position.y = -0.095
              waypoints2.append(deepcopy(wpose))
              self.group.set_start_state_to_current_state()
              plan, fraction = self.group.compute_cartesian_path(waypoints2, 0.01, 0.0, True)
              if 1-fraction < 0.2:
                  self.group.execute(plan)
                  self.counter += 1

              joint_goal = self.group.get_current_joint_values()  # Confirm the start position
              joint_goal[5] = 0.0
              self.group.go(joint_goal, wait=True)
              self.group.stop()
              sleep(0.5) # 0.5
#===================================================================================== 
            elif self.counter==5:
              start_pose = self.group.get_current_pose(self.end_effector_link).pose
              wpose = deepcopy(start_pose)
              waypoints3 = []
              waypoints3.append(deepcopy(wpose))
              wpose.position.x = -0.384
              wpose.position.y = 0.130
              waypoints3.append(deepcopy(wpose))
              self.group.set_start_state_to_current_state()
              plan, fraction = self.group.compute_cartesian_path(waypoints3, 0.01, 0.0, True)
              if 1-fraction < 0.2:
                  self.group.execute(plan)
                  self.counter += 1

              joint_goal = self.group.get_current_joint_values()  # Confirm the start position
              joint_goal[5] = pi/10
              self.group.go(joint_goal, wait=True)
              self.group.stop()
              sleep(0.5)  #0.5

#================== Executing after get the position of object ==================

  def execute(self):
        if self.track_flag:
            
            # Get the current pose so we can add it as a waypoint
            start_pose = self.group.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

#==================================== previous ======================================

            if len(self.pointx)>8:  
                if len(self.pointx)==9:
                    y_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.y += 2 * y_speed
                    wpose.position.z = 0.05

                else:
                    if len(self.pointx)==11:
                        tracker.flag2 = 1
                        self.cxy_pub.publish(tracker)

                    if len(self.pointx)<12:
                        y_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.y += (y_speed-self.error_y*0.015/105)

                    else:
                        if tracker.flag2:
                            self.track_flag=False
                        transition_pose = deepcopy(start_pose)
                        transition_pose.position.z = 0.4000

                        self.waypoints.append(deepcopy(transition_pose))

                        self.group.set_start_state_to_current_state()
                        plan, fraction = self.group.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.group.execute(plan)

                        self.group.set_max_acceleration_scaling_factor(.15)
                        self.group.set_max_velocity_scaling_factor(.25)

                        self.group.set_joint_value_target(self.transition_pose)
                        self.group.set_start_state_to_current_state()
                        plan = self.group.plan()
                        self.group.execute(plan)

                        self.group.set_joint_value_target(self.end_joint_states)
                        self.group.set_start_state_to_current_state()
                        plan = self.group.plan()
                        self.group.execute(plan)

                        if -0.1+0.02*self.object_cnt<0.2:
                            self.object_cnt += 1

                        self.waypoints = []
                        start_pose = self.group.get_current_pose(self.end_effector_link).pose
                        transition_pose = deepcopy(start_pose)
                        transition_pose.position.y -= 0.1
                        transition_pose.position.z = -0.1 + self.object_cnt*0.025
                        self.waypoints.append(deepcopy(transition_pose))

                        self.group.set_start_state_to_current_state()
                        plan, fraction = self.group.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.group.execute(plan)

                        self.phase = 2
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)

            else:
                wpose.position.x += self.error_x*0.03/105     # 0.03
                wpose.position.y += self.error_y*0.05/105
                wpose.position.z += 0.005  
                self.waypoints.append(deepcopy(wpose))
#==================================================================

            plan, fraction = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)

            self.phase = 1
            # If we have a complete plan, execute the trajectory  
            if 1-fraction < 0.2 and not (abs(self.error_y*0.025/105)<0.01 and abs(self.error_x*0.015/105)<0.01):
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.group.execute(plan)
                rospy.loginfo("Path execution complete.")
                sleep(0.5)  #1
    #        If caught the target object
            elif (abs(self.error_y*0.025/105)<0.01 and abs(self.error_x*0.015/105)<0.01):
                self.phase = 2
            else:
                rospy.loginfo("Path planning failed")

            if self.phase ==2:
                self.track_flag=False
                start_pose = self.group.get_current_pose(self.end_effector_link).pose
                wpose = deepcopy(start_pose)
                self.waypoints = []
                self.waypoints.append(deepcopy(wpose))
 #=======================  If error, change to """ """================================= 
                if self.counter == 4:
                   self.offset_x = -0.002
                   self.offset_y = 0.00#3 
                elif self.counter == 5:
                   self.offset_x = 0.025  #3
                   self.offset_y = -0.01 #5
                elif self.counter == 6:
                   self.offset_x = -0.005  # 0.01
                   self.offset_y = -0.005  # 0.01
                else:
                   self.offset_x = 0.005  # 0.005
                   self.offset_y = 0.005

                wpose.position.x += (0.013 + self.offset_x)   # 0.015   
                wpose.position.y -= (0.020 + self.offset_y)  # 0.025
                wpose.position.z -= (self.cz - 0.0)
                self.waypoints.append(deepcopy(wpose)) 

                self.group.set_start_state_to_current_state()
                plan, fraction = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
                self.group.execute(plan)

                self.counter=1

                self.group.set_joint_value_target(self.default_joint_states)
                # Set the internal state to the current state
                self.group.set_start_state_to_current_state()
                plan = self.group.plan()
                self.group.execute(plan)
                sleep(0.5) # 1
 #===========================  Come back initial pposition ==============================
        else:

            #self.default_pose_execute()  # check to runt this function
            if self.counter==1 or self.counter >4:
               self.group.set_joint_value_target(self.default_joint_states)
               # Set the internal state to the current state
               self.group.set_start_state_to_current_state()
               plan = self.group.plan()
               self.group.execute(plan)
               sleep(0.5) # 1
               self.start_flag = "-1" 
               self.counter += 1




mp=MoveGroupPythonInteface()

rospy.spin()
