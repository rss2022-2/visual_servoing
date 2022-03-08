#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("/parking_controller/drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.direction = 1
        self.velocity = 1

        self.angle_tolerance = 5.0*np.pi/180.0 #5 degrees in rad 
        self.distance_tolerance = 0.1

        self.last_time = None
        self.prev_dist_err = 0
        self.sum_dist_err = 0
        self.windup = 5
        
        self.P = 1
        self.I = 0
        self.D = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        levi = AckermannDriveStamped()

        #calculate distance and angle to cone
        dist = np.sqrt( np.square(self.relative_x) + \
                        np.square(self.relative_y))

        angle = np.arctan2(self.relative_y, self.relative_x)

        #calculate and filter distance error
        dist_err = self.low_pass_filter(self.prev_dist_err,\
                       dist - self.parking_distance)
        
        #timing stuff
        current_time = rospy.get_time()
        if self.last_time is None: self.last_time = current_time
        delta_t = current_time - self.last_time

        #update net error
        if abs(dist_err) > self.distance_tolerance:
            self.sum_dist_err += dist_err*delta_t 
        if self.sum_dist_err > self.windup:
            self.sum_dist_err = self.windup
        elif self.sum_dist_err < -self.windup:
            self.sum_dist_err = -self.windup

        #update drive message
        levi.drive.speed, levi.drive.steering_angle = \
                self.controller(dist_err, angle, delta_t)
        self.prev_dist_err = dist_err
        self.last_time = current_time
        levi.drive.acceleration = 0
        levi.drive.steering_angle_velocity = 0.5
        levi.drive.jerk = 0
        levi.header.stamp = rospy.Time.now()
        self.drive_pub.publish(levi)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()
        error_msg.y_error = self.relative_y
        error_msg.x_error = self.relative_x - self.parking_distance
        error_msg.distance_error = np.sqrt(np.square(self.relative_y) + \
                np.square(self.relative_x)) - self.parking_distance
        self.error_pub.publish(error_msg)

    def low_pass_filter(self, prev_val, cur_val, alpha=0.5):
        return alpha*cur_val + (1-alpha)*prev_val

    def controller(self, dist_err, angle, delta_t):
        
        #case where we're near right distance but wrong angle
        if abs(dist_err) < self.distance_tolerance + 0.75  and \
                abs(angle) > self.angle_tolerance:
            #if we're too close or angle is way off, reverse
            if dist_err < 0 \
                    or (dist_err < self.distance_tolerance + 0.25 \
                    and abs(angle) > 20.0*np.pi/180.0): 
                self.direction = -1
            #if cone is behind car, go forward
            if abs(angle) > np.pi*2./3.:
                self.direction = 1
            return (self.direction*0.5*self.velocity, \
                    self.direction*angle)
            

        #case where we are parked
        elif abs(dist_err) < self.distance_tolerance:
            return (0, 0)

        #case where distance and/or angle are off
        else:
        
            self.direction = 1 if dist_err > 0 else -1

            P_err = self.P*dist_err
            I_err = self.I*self.sum_dist_err
            D_err = 0
            if delta_t > 0:
                D_err = self.D*(dist_err - self.prev_dist_err)/delta_t
            
            speed = self.direction*abs(P_err + I_err + D_err)
            if speed > 1: speed = 1
            elif speed < -1: speed = -1

            steering_angle = self.direction*angle \
                if abs(angle) > self.angle_tolerance else 0
            return (speed, steering_angle)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
