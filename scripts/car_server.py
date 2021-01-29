#! /usr/bin/env python
# -*- coding: utf-8 -*-

from car_control import Car
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan

from dynamic_reconfigure.server import Server
from diff_car_controller.cfg import PIDParamsConfig
from diff_car_controller.srv import *

from enum import Enum

import tf
import math
from diff_car_controller.srv import *


import threading
import time
try:
    import _thread
except:
    import thread as _thread

mutex = threading.Lock()

max_vel = 1.2

kp_gain = 5000
ki_gain = 0
kd_gain = 0



def odom_puber(odom_info,puber):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        msg.header.seq = msg.header.seq + 1
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = odom_info['x']
        msg.pose.pose.position.y = odom_info['y']
        msg.pose.pose.position.z = 0
        odom_qua = tf.transformations.quaternion_from_euler(0, 0, odom_info['theta'])
        msg.pose.pose.orientation.x = odom_qua[0]
        msg.pose.pose.orientation.y = odom_qua[1]
        msg.pose.pose.orientation.z = odom_qua[2]
        msg.pose.pose.orientation.w = odom_qua[3]
        msg.twist.twist.linear.x = odom_info['vx']
        msg.twist.twist.linear.y = odom_info['vy']
        msg.twist.twist.angular.z = odom_info['w']
        puber.publish(msg)
        if diff_car.publish_tf:
            br.sendTransform((odom_info['x'],odom_info['y'],0),odom_qua,rospy.Time.now(),'base_footprint','odom')
        rate.sleep()

def vel_callback(msg,arg):
    global max_vel
    diff_car = arg[0]
    mode = arg[1]
    if diff_car.modeTopic != diff_car.prevmode:
        print "vel: %s mode: %s" % (mode,diff_car.modeTopic)
        diff_car.prevmode = diff_car.modeTopic
    # Processing speed, depending on the release speed of vel topic.
    if diff_car.isRunMode and diff_car.modeTopic == mode:
        if(msg.linear.x <= max_vel):
            v = msg.linear.x
            w = msg.angular.z
        else:
            v = 0.0
            w = 0.0
        diff_car.set_car_vel(v,w)
    elif diff_car.isRunMode and diff_car.modeTopic == 'manual_nostop' and mode == 'manual':
        if(msg.linear.x <= max_vel):
            v = msg.linear.x
            w = msg.angular.z
        else:
            v = 0.0
            w = 0.0
        diff_car.set_car_vel(v,w)
    # print("cmd to drive",v)
    # print("max_vel",max_vel)

def scan_callback(msg, arg):
    diff_car = arg[0]
    if not diff_car.isRunMode or diff_car.modeTopic == 'manual_nostop':
        return
    printvalue = ' '
    i = 0
    fall_flag = False
    for point in msg.ranges:
        if i > 5 and i <= 180:
            trad = math.radians(45 - 0 - 0.25 * (i-0))
            tdist = 0.20 / math.cos(trad)
            if point > 0.11 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif  i <= 290:
            trad = math.radians(72.5 - 72.5 + 0.25 * (i-181))
            tdist = 0.20 / math.cos(trad)
            if point > 0.11 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif i <= 540:
            trad = math.radians(135 - 72.5 - 0.25 * (i-291))
            tdist = 0.11 / math.cos(trad)
            if point > 0.11 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif  i <= 790:
            trad = math.radians(207.5 - 207.5 + 0.25 * (i-541))
            tdist = 0.11 / math.cos(trad)
            if point > 0.11 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif  i <= 900:
            trad = math.radians(235 - 207.5 - 0.25 * (i-791))
            tdist = 0.20 / math.cos(trad)
            if point > 0.11 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        elif i <= 1075:
            trad = math.radians(270 - 270 + 0.25 * (i-901))
            tdist = 0.20 / math.cos(trad)
            if point > 0.11 and point < tdist:
                fall_flag = True
                printvalue = str(i) + ' ' + str(point) + ' tdist: ' + str(tdist)
                break
        i = i + 1
    if fall_flag:
        diff_car.set_car_vel(0,0)
        diff_car.modeTopic = 'config'
        diff_car.config_mode()
        print 'CONFIG MODE! point: ' + printvalue

def mode_callback(msg):
    print(msg.data)
    # diff_car = arg[0]
    if msg.data == 'cmd':
        diff_car.modeTopic = 'cmd'
        if not diff_car.isRunMode:
            diff_car.run_mode()
    elif msg.data == 'manual':
        diff_car.modeTopic = 'manual'
        diff_car.torque_mode()
        print("diff_car go into the Torque mode")
        # if not diff_car.isRunMode:
        #     diff_car.run_mode()
    elif msg.data == 'manual_nostop':
        diff_car.modeTopic = 'manual_nostop'
        # if not diff_car.isRunMode:
        #     diff_car.run_mode()
    else:
        diff_car.modeTopic = 'config'
        
        diff_car.config_mode()

def maxvel_callback(msg):
    global max_vel
    max_vel = msg.data
    if(max_vel == 0.0):
        diff_car.set_car_vel(0.00,0.00)
    print("set max_vel : %f " % (max_vel))

#########################################################################################
def cbPIDParam(config, level):
    rospy.loginfo("[ZLAC706 Drive] PID Calibration Parameter reconfigure to")
    rospy.loginfo("Kp : %d", config.kp)
    rospy.loginfo("Ki : %d", config.ki)
    rospy.loginfo("Kd : %d", config.kd)

    global kp_gain, ki_gain, kd_gain

    kp_gain = config.kp
    ki_gain = config.ki
    kd_gain = config.kd

    diff_car.set_pid(kp_gain,ki_gain, kd_gain)

    return config

def srvCarmode_callback(req):
    CarMode = Enum('CarMode', 'speed_mode torque_mode')
    if req.mode == CarMode.speed_mode.value :
        diff_car.run_mode()
    elif req.mode == CarMode.torque_mode.value :
        diff_car.torque_mode()
    
    print("current car_mode : %d" % (req.mode))

    return SetModeResponse(req.mode) 
#########################################################################################


if __name__ == '__main__':
    # Initialize node
    rospy.init_node("car_server",anonymous=True)

    # max_vel = 1.2 

    # car params
    if not rospy.has_param("~wheel_diameters"):
        rospy.set_param("~wheel_diameter", 0.17)
    if not rospy.has_param("~wheel_distance"):
        rospy.set_param("~wheel_distance", 0.52)
    if not rospy.has_param("~publish_tf"):
        rospy.set_param("~publish_tf", False)
    if not rospy.has_param("~wheel_distance"):
        rospy.set_param("~wheel_distance", 0.52)

    wheel_diameter = rospy.get_param("~wheel_diameter")
    wheel_distance = rospy.get_param("~wheel_distance")
    publish_tf = rospy.get_param("~publish_tf")

    diff_car = Car(wheel_diameter,wheel_distance, publish_tf)
    diff_car.run_mode()
    

    # Create an odom release thread
    odom_publisher = rospy.Publisher('/odom',Odometry,queue_size=50)

    # Create a subscriber for cmd_vel
    rospy.Subscriber('/manual_vel',Twist,vel_callback,(diff_car,'manual',))
    rospy.Subscriber('/cmd_vel',Twist,vel_callback,(diff_car,'cmd',))

    # rospy.Subscriber('/car_mode',String,mode_callback,(diff_car,))
    rospy.Subscriber('/car_mode',String,mode_callback)
    rospy.Subscriber("/scan_filtered", LaserScan, scan_callback,(diff_car,))
    
    ################################################################

    rospy.Service('/car_mode', SetMode, srvCarmode_callback)
    rospy.Subscriber('/max_vel', Float32, maxvel_callback)
    ###############################################################

    try:
        odom_thread = _thread.start_new(odom_puber, (diff_car.odom, odom_publisher))
    except :
        rospy.loginfo("thread creation failed! program exit!")
        exit(0)
    finally:
        rospy.loginfo("odom thread created！ begin to pub odom info and transform.")
        # print("  ")
    

    #########################################################################
    global kp_gain, kp_gain, kd_gain
    kp_gain = rospy.get_param("~zlac706_PID/Kp", 9000)
    ki_gain = rospy.get_param("~zlac706_PID/Ki", 1000)
    kd_gain = rospy.get_param("~zlac706_PID/Kd", 0)

    # is_calibration_mode = rospy.get_param("~zlac706_PID/is_detection_calibration_mode", False)
    # if is_calibration_mode == True:
    srv_tune_pid = Server(PIDParamsConfig, cbPIDParam)
    ##########################################################################
        
    # Determine if car is in run_mode or config_mode
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if diff_car.isRunMode:
            #mutex.acquire()
            diff_car.update_status()
            #mutex.release()
            #rospy.spinOnce()
        else:
            diff_car.update_status()
        rate.sleep()

    rospy.spin()
