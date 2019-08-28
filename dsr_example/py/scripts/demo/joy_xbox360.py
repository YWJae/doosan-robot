#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)   

import rospy
import os
import threading, time
import sys
from sensor_msgs.msg import Joy
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import pass : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
m_joyAnalogFlag = False
m_TxyCompareFlag = False
m_RxyCompareFlag = False
m_joyButtonFlag = False
m_joyJogFlag = False
m_joyJogVel = False
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

ROBOT_SPEED_LINEAR  = 100.0 # in [mm/s]
ROBOT_SPEED_ANGULAR = 10.0 # in [deg/s]
EPSILON = 0.001

BOTTON_A           = 0
BOTTON_B           = 1
BOTTON_X           = 2
BOTTON_Y           = 3
BOTTON_UPPER_LEFT  = 4
BOTTON_UPPER_RIGHT = 5
BOTTON_BACK        = 6
BOTTON_START       = 7
BOTTON_CENTER      = 8
BOTTON_JOY_LEFT    = 9
BOTTON_JOY_RIGHT   = 10

AXIS_LEFT_H        = 0
AXIS_LEFT_V        = 1
AXIS_UPPER_LEFT    = 2
AXIS_RIGHT_H       = 3
AXIS_RIGHT_V       = 4
AXIS_UPPER_RIGHT   = 5
AXIS_DIR_H         = 6
AXIS_DIR_V         = 7


def shutdown():
    print "shutdown time!"
    print "shutdown time!"
    print "shutdown time!"

    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        #print("  current_posx      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))

        #print("  io_control_box    : %d" % (msg.io_control_box))
        ##print("  io_modbus         : %d" % (msg.io_modbus))
        ##print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)    -

r = CDsrRobot(ROBOT_ID, ROBOT_MODEL)

def joy_cb(msg):
    global m_joyAnalogFlag  # Global flag
    global m_TxyCompareFlag
    global m_RxyCompareFlag 
    global m_joyButtonFlag
    global m_joyJogFlag     # Jog flag
    global m_joyJogVel

    targetPos = [0, 0, 90, 0, 90, 0]
    hommingPos = [0, 0, 0, 0, 0, 0]
##  
    print(msg.axes)
    print(msg.buttons)
    if msg.buttons[BOTTON_START] == 1 and msg.buttons[BOTTON_BACK] == 1:
        r.movej(targetPos, 50, 50)
    elif msg.buttons[BOTTON_CENTER] == 1:
        r.movej(hommingPos, 50, 50)
    
    # Analog 신호 하나라도 들어오면 m_joyAnalogFlag -> set 됨 (global flag)
    if msg.axes[AXIS_LEFT_V] != 0 or msg.axes[AXIS_LEFT_H] != 0 or msg.axes[AXIS_RIGHT_V] != 0 or msg.axes[AXIS_RIGHT_H] != 0 or msg.axes[AXIS_UPPER_LEFT] < 1.0 or msg.axes[AXIS_UPPER_RIGHT] < 1.0 or msg.axes[AXIS_DIR_V] != 0 or msg.axes[AXIS_DIR_H] != 0:
        m_joyAnalogFlag = True
    else:
        m_joyAnalogFlag = False

    # 왼쪽 joystick -> m_TxyCompareFlag 로 구분
    if msg.axes[AXIS_LEFT_V] != 0 or msg.axes[AXIS_LEFT_H] or 0:
        if abs(msg.axes[AXIS_LEFT_V]) > abs(msg.axes[AXIS_LEFT_H]):
            m_TxyCompareFlag = False
        else:
            m_TxyCompareFlag = True
        
    # 오른쪽 joystick -> m_RxyCompareFlag 로 구분
    if msg.axes[AXIS_RIGHT_V] != 0 or msg.axes[AXIS_RIGHT_H] or 0:
        if abs(msg.axes[AXIS_RIGHT_V]) > abs(msg.axes[AXIS_RIGHT_H]):
            m_RxyCompareFlag = False
        else:
            m_RxyCompareFlag = True

    # 왼쪽 방향 버튼 -> m_ZCompareFlag 로 구분
    if msg.axes[AXIS_DIR_V] != 0 or msg.axes[AXIS_DIR_H] != 0:
        m_ZCompareFlag = True
    else:
        m_ZCompareFlag = False

    ## 버튼 -> m_joyButtonFlag 로 구분 (버튼을 위한 flag -> 수정 필요)
    #if msg.axes[AXIS_DIR_H] != 0 or msg.axes[AXIS_DIR_V] != 0:
    #    m_joyButtonFlag = True
    #else:
    #    m_joyButtonFlag = False



    ### BOTTON 제어
    #if m_joyJogFlag == -1 and not m_joyAnalogFlag and m_joyButtonFlag:
    #    # BOTTON_LEFT_UP/DOWN -> Z-direction Translation
    #    if msg.axes[AXIS_DIR_V] == 1:
    #        m_joyJogFlag = JOG_AXIS_TASK_Z
    #        m_joyJogVel = ROBOT_SPEED_LINEAR
    #    if msg.axes[AXIS_DIR_V] == -1:
    #        m_joyJogFlag = JOG_AXIS_TASK_Z
    #        m_joyJogVel = -ROBOT_SPEED_LINEAR
    #
    #    # BOTTON_LEFT_LEFT/RIGHT -> Z-direction Rotation
    #    if msg.axes[AXIS_DIR_H] == 1:
    #        m_joyJogFlag = JOG_AXIS_TASK_RZ
    #        m_joyJogVel  = -ROBOT_SPEED_LINEAR
    #    if msg.axes[AXIS_DIR_H] == -1:
    #        m_joyJogFlag = JOG_AXIS_TASK_RZ
    #        m_joyJogVel  = ROBOT_SPEED_LINEAR
    #
    #    r.jog(m_joyJogFlag, MOVE_REFERENECE_TOOL, m_joyJogVel)


    ### ANALOG 제어
    print "B: m_joyJogFlag: %s"%m_joyJogFlag
    print "B: m_joyButtonFlag: %s"%m_joyButtonFlag
    print "B: m_joyAnalogFlag: %s"%m_joyAnalogFlag
    if m_joyJogFlag == -1 and not m_joyButtonFlag and m_joyAnalogFlag: # m_joyAnalogFlag에 대한 움직임
        print "A: m_joyJogFlag: %s"%m_joyJogFlag
        print "A: m_joyButtonFlag: %s"%m_joyButtonFlag
        print "A: m_joyAnalogFlag: %s"%m_joyAnalogFlag
        ## AXIS_UPPER_LEFT/RIGHT -> Z-direction Translation (-> 나중에 Gripper Open/Close로 바꾸기)
        #if msg.axes[AXIS_UPPER_LEFT] < 1.0:
        #    m_joyJogFlag = JOG_AXIS_TASK_Z
        #    m_joyJogVel = ROBOT_SPEED_LINEAR
        #if msg.axes[AXIS_UPPER_RIGHT] < 1.0:
        #    m_joyJogFlag = JOG_AXIS_TASK_Z
        #    m_joyJogVel = -ROBOT_SPEED_LINEAR
        
        # AXIS_DIR_LEFT/RIGHT -> Z-direction Rotation
        if m_ZCompareFlag:
            if msg.axes[AXIS_DIR_H] == 1:
                m_joyJogFlag = JOG_AXIS_TASK_RZ
                m_joyJogVel = -ROBOT_SPEED_LINEAR
            if msg.axes[AXIS_DIR_H] == -1:
                m_joyJogFlag = JOG_AXIS_TASK_RZ
                m_joyJogVel = ROBOT_SPEED_LINEAR

            # AXIS_DIR_LEFT/RIGHT -> Z-direction Translation
            if msg.axes[AXIS_DIR_V] == 1:
                m_joyJogFlag = JOG_AXIS_TASK_Z
                m_joyJogVel = -ROBOT_SPEED_LINEAR
            if msg.axes[AXIS_DIR_V] == -1:
                m_joyJogFlag = JOG_AXIS_TASK_Z
                m_joyJogVel = ROBOT_SPEED_LINEAR


        # AXIS_LEFT_UP/DOWN -> X-direction Translation
        if msg.axes[AXIS_LEFT_V] > 0 and m_TxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = -ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_LEFT_V] < 0 and m_TxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_X
            m_joyJogVel = ROBOT_SPEED_LINEAR

        # AXIS_LEFT_LEFT/RIGHT -> Y-direction Translation
        if msg.axes[AXIS_LEFT_H] > 0 and m_TxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_LEFT_H] < 0 and m_TxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_Y
            m_joyJogVel = -ROBOT_SPEED_LINEAR

        # AXIS_RIGHT_UP/DOWN -> Y-direction Rotation
        if msg.axes[AXIS_RIGHT_V] > 0 and m_RxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_RY
            m_joyJogVel = ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_RIGHT_V] < 0 and m_RxyCompareFlag == 0:
            m_joyJogFlag = JOG_AXIS_TASK_RY
            m_joyJogVel = -ROBOT_SPEED_LINEAR

        # AXIS_RIGHT_LEFT/RIGHT -> X-direction Rotation
        if msg.axes[AXIS_RIGHT_H] > 0 and m_RxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_RX
            m_joyJogVel = ROBOT_SPEED_LINEAR
        if msg.axes[AXIS_RIGHT_H] < 0 and m_RxyCompareFlag == 1:
            m_joyJogFlag = JOG_AXIS_TASK_RX
            m_joyJogVel = -ROBOT_SPEED_LINEAR

        r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, m_joyJogVel)

    else:
        if not m_joyAnalogFlag and not m_joyButtonFlag:
            rospy.loginfo("jog stop")
            r.jog(m_joyJogFlag, MOVE_REFERENCE_TOOL, 0)
            m_joyJogFlag = -1


if __name__ == "__main__":
    rospy.init_node('joy_xbox360_py')
    rospy.on_shutdown(shutdown)

    #t1 = threading.Thread(target=thread_subscriber)
    #t1.daemon = True 
    #t1.start()

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)
    sub_joy  = rospy.Subscriber("joy", Joy, joy_cb)
    while not rospy.is_shutdown():
        pass

    print 'good bye!'
