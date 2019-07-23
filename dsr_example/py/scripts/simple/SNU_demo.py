#!/usr/bin/env python
## SNU Demo: M1013 + Vision Tracker

import sys
import copy
import rospy
from copy import deepcopy
from time import sleep
import threading, time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from ur5_vision_system.msg import Tracker
from ar_track_alvar_msgs.msg import AlvarMarker
from ar_track_alvar_msgs.msg import AlvarMarkers



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
INIT_JOINT_ANGLE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
    ## Change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the M1013:
    group_name = "arm"
    self.group = moveit_commander.MoveGroupCommander(group_name)

    # Set the reference frame for pose targets, Set the M1013 arm reference frame accordingly
    # reference_frame = "/base_link"
    # self.gruop.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    self.group.allow_replanning(True) 
    self.group.set_goal_position_tolerance(0.01)
    self.group.set_goal_orientation_tolerance(0.1)
    self.group.set_planning_time(0.1)
    self.group.set_max_acceleration_scaling_factor(.5)
    self.group.set_max_velocity_scaling_factor(.65)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish trajectories for RViz to visualize:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Subscribing the target pose from AR_Tracker
    self.target_pose = geometry_msgs.msg.Pose()
    
    # t1 = threading.Thread(target=self.thread_subscriber_tracker)
    # t1.daemon = True 
    # t1.start()

    t1 = threading.Thread(target=self.thread_subscriber_arMarks)
    t1.daemon = True 
    t1.start()

    

    # Getting Basic Information: the name of the reference frame for this robot:
    self.planning_frame = self.group.get_planning_frame()
    print "============ Reference frame: %s" % self.planning_frame

    # We can also print the name of the end-effector link for this group:
    self.end_effector_link = self.group.get_end_effector_link()
    print "============ End effector: %s" % self.end_effector_link

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    print "============ Robot Groups:", self.robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print self.robot.get_current_state()


    print "============ Moveit_Commander Setup Initialized !!!"
    
    self.go_to_joint_state()
    print "Robot Pose Initialized!"


##############################################################################################################

  def thread_subscriber_tracker(self):
      rospy.Subscriber('/'+TRACKER_TOPIC, Tracker, self.tracker_cb, queue_size=10)
      rospy.spin()

  def tracker_cb(self, msg):
      self.target_pose.position.x = msg.y
      self.target_pose.position.y = msg.x
      self.target_pose.position.z = msg.z

  def thread_subscriber_arMarks(self):
    rospy.Subscriber('/'+AR_MARKER_TOPIC, AlvarMarkers, self.arMarker_cb, queue_size=10)
    rospy.spin()

  def arMarker_cb(self, msg):
    try:
      self.target_pose = msg.markers[0].pose.pose
    except IndexError:
      return
    print "%s" % len(msg.markers)
      
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


  def go_to_pose_goal(self):
    ## We can plan a motion for this group to a desired pose for the end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    
    qt = quaternion_from_euler(YAW, PITCH, ROLL)
    pose_goal.orientation.w = qt[0]
    pose_goal.orientation.x = qt[1]
    pose_goal.orientation.y = qt[2]
    pose_goal.orientation.z = qt[3]
    pose_goal.position.x = self.target_pose.position.x
    pose_goal.position.y = self.target_pose.position.y
    pose_goal.position.z = self.target_pose.position.z + DISTANCE_AWAY_FROM_TARGET

    self.group.set_pose_target(pose_goal)
    plan = self.group.plan()
    sleep(0.4)
    self.group.execute(plan)
    
    # #############
    # waypoints = []
    # # start_pose = self.group.get_current_pose(self.end_effector_link).pose
    # # wpose = deepcopy(start_pose)
    # waypoints.append(deepcopy(self.target_pose))

    # fraction = 0.0
    # maxtries = 100
    # attempts = 0

    # # Set the internal state to the current state
    # self.group.set_start_state_to_current_state()

    # # Plan the Cartesian path connecting the waypoints
    # while (1 - fraction) > 0.2 and attempts < maxtries:
    #     (plan, fraction) = self.group.compute_cartesian_path (waypoints, 0.05, 0.0, True)

    #     # Increment the number of attempts
    #     attempts += 1

    #     # Print out a progress message
    #     if attempts % 10 == 0:
    #         rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

    # # If we have a complete plan, execute the trajectory
    # if (1 - fraction) <= 0.2:
    #     attemps = 0
    #     rospy.loginfo("Path computed successfully. Moving the arm.")
    #     self.group.execute(plan)
    #     rospy.loginfo("Path execution complete.")
    # else:
    #     rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
    # ############



#
    # ## Now, we call the planner to compute the plan and execute it.
    # plan = self.group.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    # self.group.stop()
    # # It is always good to clear your targets after planning with poses.
    # # Note: there is no equivalent function for clear_joint_value_targets()
    # self.group.clear_pose_targets()

    # # For testing:
    # # Note that since this section of code will not be included in the tutorials
    # # we use the class variable rather than the copied state variable
    # current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    ## You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through:
    waypoints = []

    start_pose = self.group.get_current_pose(self.end_effector_link).pose
    wpose = deepcopy(start_pose)
    waypoints.append(deepcopy(self.target_pose))

    (plan, fraction) = self.group.compute_cartesian_path(
                                                      waypoints,  # waypoints to follow
                                                      0.01,       # eef_step
                                                      0.0,        # jump_threshold
                                                      True)
    return plan, fraction


  def display_trajectory(self, plan):
    ## Displaying a Trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    ## Use execute if you would like the robot to follow the plan that has already been computed:
    self.group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

##############################################################################################################

def main():
  try:   
    move_group_snu = MoveGroupPythonInteface()

    while not rospy.is_shutdown():
      print "============ Press `Enter` to execute a movement using a pose goal ..."
      raw_input()
      move_group_snu.go_to_pose_goal()

    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = move_group_snu.plan_cartesian_path()

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # move_group_snu.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # move_group_snu.execute_plan(cartesian_plan)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()