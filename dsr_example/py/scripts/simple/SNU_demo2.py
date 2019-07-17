#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
import roslib 
import actionlib
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
from m1013_vision_system.msg import Tracker
from std_msgs.msg import Header
from math import pi
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep

tracker = Tracker()
TARGET_POSE = 'cxy'
PLANNING_GROUP_NAME = 'arm'


client = None


class m1013_mp:
    def __init__(self):
        rospy.init_node("m1013_demo", anonymous=True, disable_signals=True)
        self.counter = 1
        self.start_flag = "false"
        self.offset_x = 0.0
        self.offset_y = 0.0

        try:
                self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
                print "Waiting for server..."
                self.client.wait_for_server()
                print "Connected to server"
                print "Please make sure that your robot can move freely between these poses before proceeding!"

                # self.default_pose_execute()
                self.init_pose_execute()

                rospy.Subscriber('R_007/ur_pnp', String, self.ur_PickPlace, queue_size=1)

                self.cxy_sub = rospy.Subscriber('/' +TARGET_POSE, Tracker, self.tracking_cb, queue_size=1)
                # self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
#================================

    def ur_PickPlace(self,msg):
        self.start_flag = msg.data 

    def go_init_pose(self):
        movej(p0, vel = 100, acc = 100)


    def default_pose_execute(self):
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

        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the m1013_arm
        self.arm = moveit_commander.MoveGroupCommander(PLANNING_GROUP_NAME)

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the m1013_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.65)
        self.shoulder_lift_vel_name = '/robot_description_planning/joint_limits/shoul der_lift_joint/max_velocity'
        self.shoulder_pan_vel_name = '/robot_description_planning/joint_limits/shoulder_pan_joint/max_velocity'
        self.elbow_vel_name = '/robot_description_planning/joint_limits/elbow_joint/max_velocity'

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []

        # Default pose in Cartesian frame
        wpose = deepcopy(start_pose)

        self.waypoints.append(deepcopy(wpose))

        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()

        Q1 = [0, -1.45783222, -1.437422222, -1.8142222, 1.57, 0]  # change later based on the position of table

        self.default_joint_states = Q1

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        sleep(0.4)  #0.8
        self.arm.execute(plan)

#===========================================================================================================================

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

#=============================== Tracking objects===============================

    def tracking_cb(self, msg):
      if (self.start_flag == "1"):
        self.track_flag = msg.flag1
        self.cx = msg.y
        self.cy = msg.x
        self.cz = msg.z 
        self.error_x = msg.error_y  
        self.error_y = msg.error_x  
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
               self.arm.set_joint_value_target(self.default_joint_states)
               # Set the internal state to the current state
               self.arm.set_start_state_to_current_state()
               plan = self.arm.plan()
               self.arm.execute(plan)
               self.counter += 1
               self.start_flag =  "-1" 

#=============================== Try to searching the object=================================   Whether just need to rotate 6th axis is ok or not
            elif self.counter==2:
              while True:
                 start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                 wpose = deepcopy(start_pose)
                 waypoints1 = []
                 waypoints1.append(deepcopy(wpose))
                 wpose.position.x = -0.219
                 waypoints1.append(deepcopy(wpose))
                 self.arm.set_start_state_to_current_state()
                 plan, fraction = self.arm.compute_cartesian_path(waypoints1, 0.01, 0.0, True)
                 if 1-fraction < 0.2:
                    break
              self.arm.execute(plan)
              sleep(0.5) # 0.5
              self.counter += 1

#===================================================================================== 
            elif self.counter==3:
              while True:
                 start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                 wpose = deepcopy(start_pose)
                 waypoints0 = []
                 waypoints0.append(deepcopy(wpose))
                 wpose.position.y -= 0.3
                 waypoints0.append(deepcopy(wpose))
                 self.arm.set_start_state_to_current_state()
                 plan, fraction = self.arm.compute_cartesian_path(waypoints0, 0.01, 0.0, True)
                 if 1-fraction < 0.2:
                    break
              self.arm.execute(plan)
              sleep(0.5) #0.5
              self.counter += 1

#=======================================================================================
            elif self.counter==4:
              start_pose = self.arm.get_current_pose(self.end_effector_link).pose
              wpose = deepcopy(start_pose)
              waypoints2 = []
              waypoints2.append(deepcopy(wpose))
              wpose.position.x = -0.409 
              wpose.position.y = -0.095
              waypoints2.append(deepcopy(wpose))
              self.arm.set_start_state_to_current_state()
              plan, fraction = self.arm.compute_cartesian_path(waypoints2, 0.01, 0.0, True)
              if 1-fraction < 0.2:
                  self.arm.execute(plan)
                  self.counter += 1

              joint_goal = self.arm.get_current_joint_values()  # Confirm the start position
              joint_goal[5] = 0.0
              self.arm.go(joint_goal, wait=True)
              self.arm.stop()
              sleep(0.5) # 0.5
#===================================================================================== 
            elif self.counter==5:
              start_pose = self.arm.get_current_pose(self.end_effector_link).pose
              wpose = deepcopy(start_pose)
              waypoints3 = []
              waypoints3.append(deepcopy(wpose))
              wpose.position.x = -0.384
              wpose.position.y = 0.130
              waypoints3.append(deepcopy(wpose))
              self.arm.set_start_state_to_current_state()
              plan, fraction = self.arm.compute_cartesian_path(waypoints3, 0.01, 0.0, True)
              if 1-fraction < 0.2:
                  self.arm.execute(plan)
                  self.counter += 1

              joint_goal = self.arm.get_current_joint_values()  # Confirm the start position
              joint_goal[5] = pi/10
              self.arm.go(joint_goal, wait=True)
              self.arm.stop()
              sleep(0.5)  #0.5

#================== Executing after get the position of object ==================

    def execute(self):
        if self.track_flag:
            
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

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

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        self.arm.set_max_acceleration_scaling_factor(.15)
                        self.arm.set_max_velocity_scaling_factor(.25)

                        self.arm.set_joint_value_target(self.transition_pose)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan)

                        self.arm.set_joint_value_target(self.end_joint_states)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan)

                        if -0.1+0.02*self.object_cnt<0.2:
                            self.object_cnt += 1

                        self.waypoints = []
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        transition_pose = deepcopy(start_pose)
                        transition_pose.position.y -= 0.1
                        transition_pose.position.z = -0.1 + self.object_cnt*0.025
                        self.waypoints.append(deepcopy(transition_pose))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        self.phase = 2
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)

            else:
                wpose.position.x += self.error_x*0.03/105     # 0.03
                wpose.position.y += self.error_y*0.05/105
                wpose.position.z += 0.005  
                self.waypoints.append(deepcopy(wpose))
#==================================================================

            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)

            self.phase = 1
            # If we have a complete plan, execute the trajectory  
            if 1-fraction < 0.2 and not (abs(self.error_y*0.025/105)<0.005 and abs(self.error_x*0.015/105)<0.005):
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
                sleep(0.5)  #1
    #        If caught the target object
            elif (abs(self.error_y*0.025/105)<0.005 and abs(self.error_x*0.015/105)<0.005):
                self.phase = 2
            else:
                rospy.loginfo("Path planning failed")

            if self.phase ==2:
                self.track_flag=False
                start_pose = self.arm.get_current_pose(self.end_effector_link).pose
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

                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
                self.arm.execute(plan)

                self.counter=1
 #===========================  Come back initial pposition ==============================
        else:

            #self.default_pose_execute()  # check to runt this function
            if self.counter==1 or self.counter >4:
               self.arm.set_joint_value_target(self.default_joint_states)
               # Set the internal state to the current state
               self.arm.set_start_state_to_current_state()
               plan = self.arm.plan()
               self.arm.execute(plan)
               sleep(0.5) # 1
               self.start_flag = "-1" 
               self.counter += 1

mp=m1013_mp()

rospy.spin()
