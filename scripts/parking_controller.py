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
        self.last_angle = 0
        self.direction = 1
        self.velocity = 1
        self.angle_tolerance = 5.0*np.pi/180.0 #3 degrees in radians
        self.distance_tolerance = 0.05
        self.last_time = None
        
        self.P = 4.2
        self.D = 2


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        levi = AckermannDriveStamped()

        distance = np.sqrt(\
                np.square(self.relative_x) + \
                np.square(self.relative_y))
        angle = np.arctan(self.relative_y/self.relative_x)
        print("angle: " + str(angle) + "\t dist: " + str(distance))
        dist_err = distance - self.parking_distance
        if abs(dist_err) < self.distance_tolerance: dist_err = 0

        current_time = rospy.get_time()
        if self.last_time is None: self.last_time = current_time
        delta_t = current_time - self.last_time

        #case where we're near right distance but wrong angle
        if abs(dist_err) < self.distance_tolerance + 0.25  and abs(angle) > self.angle_tolerance:
            if dist_err < -0.05: self.direction = -1
            levi.drive.speed = self.direction*0.5*self.velocity
            levi.drive.steering_angle = self.direction*self.P*angle
            

        #case where we are parked
        elif dist_err == 0:
            levi.drive.speed = 0 
            levi.drive.steering_angle = 0
        
        #case where angle and distance are off
        else:
        
            self.direction = 1 if dist_err > 0 else -1

            P_err = self.P*angle
            D_err = 0
            if delta_t > 0:
                D_err = self.D*(angle - self.last_angle)/delta_t
            
            levi.drive.speed = self.direction*min(self.velocity, abs(dist_err) + 0.1)
            levi.drive.steering_angle = self.direction*(P_err + D_err) \
                    if abs(angle) > self.angle_tolerance else 0
        
        self.last_time = current_time
        levi.drive.acceleration = 0
        levi.drive.steering_angle_velocity = 0.5
        levi.drive.jerk = 0
        levi.header.stamp = rospy.Time.now()
        self.drive_pub.publish(levi)
        #self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
