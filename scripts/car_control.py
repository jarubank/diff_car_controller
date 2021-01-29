#! /usr/bin/env python
# -*- coding: utf-8 -*-

from zlac706_new import SpeedMotor
from math import *
import time

import threading
import time
try:
    import _thread
except:
    import thread as _thread

mutex = threading.Lock()

def motorThread (motor_obj):
    while True:
        motor_obj.send_motor()
        time.sleep(0.07)

def motorRaedThread (motor1, motor2, car):
    while True:
        motor1.read_motor()
        motor2.read_motor()
        # motor3.read_motor()
        # motor4.read_motor()
        car.set_odom()
        time.sleep(0.1)

class Car(object):
    def __init__(self,wheel_diameter,wheel_distance,publish_tf):

        self.steering_angle_ = 0.0
        self.linear_velocity_x_ = 0.0
        self.linear_velocity_y_ = 0.0
        self.angular_velocity_z_ = 0.0
        #ros::Time last_vel_time_ = 0.0
        self.vel_dt_ = 0.0
        self.x_pos_ = 0.0
        self.y_pos_ = 0.0
        self.heading_ = 0.0

        self.diameter = wheel_diameter
        self.distance = wheel_distance
        self.publish_tf = publish_tf
        self.odom = {'x':0,'y':0,'theta':0,'vx':0,'vy':0,'w':0}
        self.isRunMode = False
        self.modeTopic = 'cmd'
        self.prevmode = 'cmd'
        self.isSending = False
        self.current_time = time.time()
        self.last_time = self.current_time
        self.motor = []
        self.motor.append(SpeedMotor('/dev/zla06_right'))
        self.motor.append(SpeedMotor('/dev/zla06_left'))
        # self.motor.append(SpeedMotor('/dev/ttyUSB2'))
        # self.motor.append(SpeedMotor('/dev/ttyUSB0'))
        self.enable()

        try:
            motor1_thread = _thread.start_new(motorThread, (self.motor[0],))
            motor2_thread = _thread.start_new(motorThread, (self.motor[1],))
            # motor3_thread = _thread.start_new(motorThread, (self.motor[2],))
            # motor4_thread = _thread.start_new(motorThread, (self.motor[3],))
            motor_read_thread = _thread.start_new(motorRaedThread, (self.motor[0],self.motor[1],self))
        except:
            print("thread creation failed! program exit!")
            exit(0)
        finally:
            print("motor thread ready!")

    def enable(self):
        self.motor[0].motor_start()
        self.motor[1].motor_start()

        return True

    def disable(self):
        self.motor[0].motor_stop()
        self.motor[1].motor_stop()

        return True

    # According to the speed of the car, calculate the speed of the wheel
    def set_car_vel(self,v,w):
        w_r, w_l = self.cal_wheel_vel(v,w)
#        if w is not 0 and v == 0:
#            self.motor[0].set_speed = w_r * 0.0
#            self.motor[1].set_speed = w_l * 0.0
#            self.motor[2].set_speed = w_r * 1.2
#            self.motor[3].set_speed = w_l * 1.2
#        else:
        if True:
            self.motor[0].set_speed = w_r
            self.motor[1].set_speed = w_l

        return True

###########################################################################################
    def set_pid(self, kp, ki, kd):
        self.motor[0].is_config_pid = True
        self.motor[0].kp_gain_ = kp
        self.motor[0].ki_gain_ = ki
        self.motor[0].kd_gain_ = kd

        self.motor[1].is_config_pid = True
        self.motor[1].kp_gain_ = kp
        self.motor[1].ki_gain_ = ki
        self.motor[1].kd_gain_ = kd
##########################################################################################

    # Calculate wheel speed
    def cal_wheel_vel(self,v,w):
        # w_l = 2*v/(2*self.diameter) - w*self.distance/(2*self.diameter)
        # w_r = -(2*v/(2*self.diameter) + w*self.distance/(2*self.diameter))
        # print(w_l*10 , w_r*10)
        
        # # w_l = 2*v/(2*self.diameter) + w*self.distance/(2*self.diameter)
        # # w_r = -(2*v/(2*self.diameter) - w*self.distance/(2*self.diameter))


        linear_vel_x_mins = 0.0
        linear_vel_y_mins = 0.0
        angular_vel_z_mins = 0.0
        tangential_vel = 0.0
        x_rpm = 0.0
        y_rpm = 0.0
        tan_rpm = 0.0
        rpm_motor1 = 0.0
        pm_motor2 = 0.0
        
        max_val = 200
        wheel_circumference_ = pi * 0.175
        wheels_x_distance_ = 0.0
        wheels_y_distance_ = 0.52

        # convert m/s to m/min
        linear_vel_x_mins = v * 60
        linear_vel_y_mins = 0.0

        # convert rad/s to rad/min
        angular_vel_z_mins = w * 60

        tangential_vel = angular_vel_z_mins * ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2))

        x_rpm = linear_vel_x_mins / wheel_circumference_
        y_rpm = linear_vel_y_mins / wheel_circumference_
        tan_rpm = tangential_vel / wheel_circumference_

        # calculate for the target motor RPM and direction
        # front-left motor
        w_l = x_rpm - y_rpm - tan_rpm
        w_l = min(max_val, max(-max_val, w_l))

        # front-right motor
        w_r = -(x_rpm + y_rpm + tan_rpm)
        w_r = min(max_val, max(-max_val, w_r))

        return [w_r, w_l]
    
    # Get the speed and speed of the car
    def get_car_status(self):
        # odom_w_r = self.motor[0].rel_speed
        # odom_w_l = self.motor[1].rel_speed
        # # rpm_1 = odom_w_l
        # # rpm_2 = odom_w_r
        # #if odom_w_r < -2437:
        # #    odom_w_r += 2437.5
        # #if odom_w_l < -2437:
        # #    odom_w_l += 2437.5    
        # print("odom check", odom_w_r, odom_w_l)
        # # print("odom check", rpm_1, rpm_2)
        # v = ((odom_w_r-odom_w_l)*(self.diameter/2)/10)
        # w = ((odom_w_r+odom_w_l)*(self.diameter/self.distance)/2)



        odom_w_r = self.motor[0].rel_speed
        odom_w_l = self.motor[1].rel_speed
        rpm1 = -odom_w_r 
        rpm2 = odom_w_l
        # print("rpm_1", rpm1, "rpm_2", rpm2 )
        rpm3 = 0
        rpm4 = 0

        average_rps_x = 0.0
        average_rps_y = 0.0
        average_rps_a = 0.0

                
        max_val = 200
        wheel_circumference_ = pi * 0.175
        wheels_x_distance_ = 0.0
        wheels_y_distance_ = 0.52
        total_wheels_ = 2

        # convert average revolutions per minute to revolutions per second
        average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60 # RPM
        v = average_rps_x * wheel_circumference_; # m/s

        #convert average revolutions per minute in y axis to revolutions per second
        # average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_) / 60; // RPM
        
        # vel.li_y = 0

        # convert average revolutions per minute to revolutions per second
        average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60
        w =  (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); # rad/s
        w = -w

        # print("return")
        # print(v,w)
        # print(" ")

        return [v,w]

    # Set vehicle odom information
    def set_odom(self):
        self.current_time = time.time()

        v,w = self.get_car_status() 
        # print(v, w)

        # steering_angle_ = 0.0
        # linear_velocity_x_ = 0.0
        # linear_velocity_y_ = 0.0
        # angular_velocity_z_ = 0.0
        # #ros::Time last_vel_time_ = 0.0
        # vel_dt_ = 0.0
        # x_pos_ = 0.0
        # y_pos_ = 0.0
        # heading_ = 0.0

        self.linear_velocity_x_ = v
        self.linear_velocity_y_ = 0.0
        self.angular_velocity_z_ = w

        self.vel_dt_ = self.current_time - self.last_time
        self.last_time = self.current_time

        self.delta_heading = self.angular_velocity_z_ * self.vel_dt_;#radians
        self.delta_x = (self.linear_velocity_x_ * cos(self.heading_) - self.linear_velocity_y_ * sin(self.heading_)) * self.vel_dt_ #m
        self.delta_y = (self.linear_velocity_x_ * sin(self.heading_) + self.linear_velocity_y_ * cos(self.heading_)) * self.vel_dt_ #m
        
        #calculate current position of the robot
        self.x_pos_ += self.delta_x
        self.y_pos_ += self.delta_y
        self.heading_ += self.delta_heading

        #print("w", 1 * w)
        # Twist
        self.vx = self.linear_velocity_x_
        self.vy = self.linear_velocity_y_

        # Pose
        self.odom['x']= self.x_pos_
        self.odom['y']= self.y_pos_
        #if self.odom['theta'] >= 3.14 or self.odom['theta'] <= -3.14:
        #    self.odom['theta'] = -self.odom['theta'] + theta_dot*dt
        self.odom['theta'] = self.heading_ 
        self.odom['vx'] = self.vx
        self.odom['vy'] = self.vy
        self.odom['w'] = w
        #print("set odom", self.odom['x'],'y',self.odom['y'],'theta',self.odom['pose_theta'])


    # Enter config mode, close bus
    def config_mode(self):
        self.motor[0].motor_stop()
        self.motor[1].motor_stop()

        self.isRunMode = False
 #########################################################################   
    # Enter torque mode, free whell you can push
    def torque_mode(self):
        self.motor[0].set_torque_mode()
        self.motor[1].set_torque_mode()
        self.isRunMode = False
        print("diff_car go into the Torque mode")
##########################################################################
    # Enter run_mode, you can speed control
    def run_mode(self):
        self.motor[0].set_speed_mode()
        self.motor[1].set_speed_mode()

        self.isRunMode = True
        print("diff_car go into the run mode")

    # With new wheel information and car information
    def update_status(self):
        None
        self.set_odom()

# --------------------------
#   Test Car Script Below
# --------------------------
def test_set_car_vel(v,w):
    wheel_diameter = 0.06
    wheel_distance = 0.3
    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.run_mode()
    start = time.time()
    #a = SpeedMotor('/dev/ttyUSB0')
    #b = a.read_motor
    while True:
        diff_car.set_car_vel(v,w)
        #print("WOW", b)
        if (time.time() - start > 80):
            break

def test_car_run_mode():
    wheel_diameter = 0.100
    wheel_distance = 0.100
    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.run_mode()
    
def test_car_config_mode():
    wheel_diameter = 0.100
    wheel_distance = 0.100
    diff_car = car(wheel_diameter,wheel_distance)
    diff_car.config_mode()

if __name__ == '__main__':
    #test_car_config_mode()
    #test_car_run_mode()

    test_set_car_vel(75,0)
    # print(odom_w_r,odom_w_l)

