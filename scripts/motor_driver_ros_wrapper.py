#!/usr/bin/env python3

import rospy
from motor_pwm.motor_driver import MotorDriver

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# the following are commented since they give error
# from diagnostic_msgs.msg import DiagnosticStatus
# from diagnostic_msgs.msg import KeyValue

class MotorDriverROSWrapper:

    def __init__(self):
        max_speed = rospy.get_param("~max_speed", 100)
        driver_pins = rospy.get_param("~driver_pins", [20, 21, 24])
        pwm_freq = rospy.get_param("~pwm_freq", 200)

        # publihing frequency
        publish_current_speed_frequency = rospy.get_param("~publish_current_speed_frequency", 5.0)
        # publish_motor_status_frequency = rospy.get_param("~publish_motor_status_frequency", 1.0)
        
        # initialize the motor
        self.motor = MotorDriver(max_speed=max_speed, driver_pins=driver_pins, pwm_freq=pwm_freq)

        # subscribe to motor speed command
        rospy.Subscriber("speed_command", Float32, self.callback_speed_command)

        # subscribe to cmd_vel from teleop_keyboard
        rospy.Subscriber("cmd_vel", Twist, self.callback_speed_cmd_vel)
        
        # initialize a service
        rospy.Service("stop_motor", Trigger, self.callback_stop)

        # set up publishers
        self.current_speed_pub = rospy.Publisher("current_speed", Float32, queue_size=10)
        # self.motor_status_pub = rospy.Publisher("motor_status", DiagnosticStatus, queue_size=1)

        rospy.Timer(rospy.Duration(1.0/publish_current_speed_frequency), self.publish_current_speed)
        # rospy.Timer(rospy.Duration(1.0/publish_motor_status_frequency), self.publish_motor_status)

    '''
    publish current motor speed
    '''
    def publish_current_speed(self, event=None):
        self.current_speed_pub.publish(self.motor.get_speed())

    '''
    TODO: publish motor status
    def publish_motor_status(self, event=None):
        status = self.motor.get_status()
        data_list = []
        for key in status:
            data_list.append(KeyValue(key, str(status[key])))

        msg = DiagnosticStatus()
        msg.values = data_list

        self.motor_status_pub.publish(msg)
    '''
    
    def stop(self):
        self.motor.stop()

    def callback_speed_command(self, msg):
        self.motor.set_pwm(msg.data)

    def callback_speed_cmd_vel(self, msg):
        self.motor.set_pwm(msg.linear.x)
        rospy.loginfo("received {} from teleop".format(msg.linear.x))

    def callback_stop(self, req):
        self.stop()
        return {"success": True, "message": "Motor has been stopped"}

if __name__ == "__main__":
    rospy.init_node("motor_driver")

    motor_driver_wrapper = MotorDriverROSWrapper()
    rospy.on_shutdown(motor_driver_wrapper.stop)

    rospy.loginfo("Motor driver is now started, ready to get commands.")
    rospy.spin()