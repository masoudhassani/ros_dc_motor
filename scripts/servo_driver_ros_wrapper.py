#!/usr/bin/env python3

import rospy
from motor_pwm.servo_driver import ServoDriver
from rospy.client import init_node

from std_msgs.msg import Float32, Int16
from std_srvs.srv import Trigger

class ServoDriverROSWrapper:

    def __init__(self):
        max_speed = rospy.get_param("~max_speed", 100)
        driver_pin = rospy.get_param("~driver_pin", 23)
        pwm_freq = rospy.get_param("~pwm_freq", 50)
        init_angle = rospy.get_param("~init_angle", 90)
        min_angle = rospy.get_param("~min_angle", 0)
        max_angle = rospy.get_param("~max_angle", 180)

        # publihing frequency
        publish_current_angle_frequency = rospy.get_param("~publish_current_angle_frequency", 5.0)
        
        # initialize the motor
        self.servo = ServoDriver(max_speed=max_speed, driver_pin=driver_pin, pwm_freq=pwm_freq,
                                init_angle=init_angle, min_angle=min_angle, max_angle=max_angle)

        # subscribe to motor angle command
        rospy.Subscriber("angle_command", Float32, self.callback_angle_command)
        
        # initialize a service
        rospy.Service("reset_servo", Trigger, self.callback_reset)

        # set up publishers
        self.current_angle_pub = rospy.Publisher("current_angle", Float32, queue_size=10)

        rospy.Timer(rospy.Duration(1.0/publish_current_angle_frequency), self.publish_current_angle)

    '''
    publish current servo angle
    '''
    def publish_current_angle(self, event=None):
        self.current_angle_pub.publish(self.servo.get_angle())

    def reset(self):
        self.servo.reset()

    def callback_angle_command(self, msg):
        self.servo.set_pwm(msg.data)
        
    def callback_reset(self, req):
        self.reset()
        return {"success": True, "message": "servo has been reset"}

if __name__ == "__main__":
    rospy.init_node("servo_driver")

    servo_driver_wrapper = ServoDriverROSWrapper()
    rospy.on_shutdown(servo_driver_wrapper.reset)

    rospy.loginfo("servo driver is now started, ready to get commands.")
    rospy.spin()