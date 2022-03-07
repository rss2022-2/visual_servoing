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

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.sum_error = 0
        self.last_error = 0
        self.windup = 10
        self.last_time = None
        self.direction = 1
        self.velocity = 1
        self.angle_tolerance = 1
        self.distance_tolerance = self.parking_distance/10
        
        self.P = 6.9
        self.D = 1.3
        self.I = 0


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        distance = np.sqrt(\
                np.square(self.relative_x) + \
                np.square(self.relative_y))
        theta = np.arctan(self.relative_y/self.relative_x)

        current_time = rospy.get_time()
        if self.last_time is None: self.last_time = current_time
        delta_time = current_time - self.last_time

        err = distance - self.parking_distance
        if abs(err) < self.distance_tolerance: err = 0
        self.sum_error += err
        
        #case where we're at right distance but wrong angle
        if err == 0  and abs(theta) > 3:
            pass

        #case where we are parked
        elif err == 0:
            pass
        
        #case where angle and distance are off
        else:
        
            self.direction = 1 if \
                    distance > (self.parking_distance + self.distance_tolerance) \
                    else -1 if  distance < (self.parking_distance - self.distance_tolerance) \
                    else self.direction
        

            P_err = self.P*err
            D_err = 0
            if delta_time > 0:
                D_err = err - self.last_error

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

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
