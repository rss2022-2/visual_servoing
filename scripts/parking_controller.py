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

        self.parking_distance = 0.7 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.last_angle = 0
        self.last_dist_err = 0
        self.direction = 1
        self.velocity = 0.25
        self.angle_tolerance = 10*np.pi/180.0 #3 degrees in radians
        self.distance_tolerance = 0.05
        self.last_time = None
        
        self.P = 0.4
        self.D = 0.0
        self.I = 0.3
        self.I_err = 0
        self.min_gain = 0.16


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        print(msg.x_pos, msg.y_pos)
        levi = AckermannDriveStamped()

        distance = np.sqrt(\
                np.square(self.relative_x) + \
                np.square(self.relative_y))
        angle = np.arctan2(self.relative_y,self.relative_x)
        # print("angle: " + str(angle) + "\t dist: " + str(distance))
        dist_err = distance - self.parking_distance
        # if abs(dist_err) < self.distance_tolerance: dist_err = 0

        current_time = rospy.get_time()
        if self.last_time is None: self.last_time = current_time
        delta_t = current_time - self.last_time

        # #case where we're near right distance but wrong angle
        # if abs(dist_err) < self.distance_tolerance + 0.25  and abs(angle) > self.angle_tolerance:
        #     if dist_err < -0.05: self.direction = -1
        #     levi.drive.speed = self.direction*0.5*self.velocity 
        #     levi.drive.steering_angle = self.direction*self.P*angle
        #     print("right dist wrong angl")
            

        #case where we are parked
        if abs(dist_err) < self.distance_tolerance and abs(angle) < self.angle_tolerance:
            levi.drive.speed = 0 
            levi.drive.steering_angle = 0
            #self.I_err = 0
            print("parked")
        
        #case where angle and distance are off
        else:
        
            # self.direction = 1 if dist_err > 0 else -1

            P_err = self.P*dist_err
            D_err = 0
            if delta_t > 0:
                D_err = self.D*(dist_err - self.last_dist_err)/delta_t
                self.I_err = self.I_err + self.I*(dist_err)*delta_t
                if abs(self.I_err) > abs(self.velocity):
                    self.I_err = np.sign(self.I_err) * abs(self.velocity)
            
            levi.drive.speed = P_err + D_err + self.I_err
            levi.drive.speed += np.sign(levi.drive.speed)*self.min_gain
            levi.drive.steering_angle = angle * np.sign(levi.drive.speed)
            print("P ", P_err, "I ", self.I_err, "D ", D_err)
            print("all off")
        
        if abs(levi.drive.speed) > abs(self.velocity):
           levi.drive.speed = np.sign(levi.drive.speed) * abs(self.velocity)

        print("angle ", levi.drive.steering_angle, "speed ", levi.drive.speed)
        self.last_time = current_time
        self.last_dist_err = dist_err
        levi.drive.acceleration = 0.1
        levi.drive.steering_angle_velocity = 0
        levi.drive.jerk = 0.1
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

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
