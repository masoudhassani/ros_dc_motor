#!/usr/bin/env python3

import rospy  # import the ros python binding
from std_msgs.msg import String     # import string from the standard ros messages
import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)   # bcm pin numbering
GPIO.setwarnings(False)

# Set variables for the GPIO motor pins
motor_pins = [20, 21, 24]    # [fwd, rev, pwm signal]
GPIO.setup(motor_pins[0], GPIO.OUT)
GPIO.setup(motor_pins[1], GPIO.OUT)
GPIO.setup(motor_pins[2], GPIO.OUT)

# setup pwm
frequency = 200
motor_signal = GPIO.PWM(motor_pins[2], frequency) # pin number, frequency
duty_cycle = 70
motor_signal.start(0)   

# initialize direction pins
GPIO.output(motor_pins[0], GPIO.LOW)
GPIO.output(motor_pins[1], GPIO.LOW)

# turn all motors off
def stop_motor():
    motor_signal.ChangeDutyCycle(0)
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.LOW)   

#  forward direction
def forward():
    motor_signal.ChangeDutyCycle(duty_cycle)
    GPIO.output(motor_pins[0], GPIO.HIGH)
    GPIO.output(motor_pins[1], GPIO.LOW)

# Turn both motors backwards
def reverse():
    motor_signal.ChangeDutyCycle(duty_cycle)
    GPIO.output(motor_pins[0], GPIO.LOW)
    GPIO.output(motor_pins[1], GPIO.HIGH)

# Message handler
def command_callback(commandMessage):
    command = commandMessage.data
    if command == 'forward':
        print('Moving forwards')
        forward()
    elif command == 'reverse':
        print('Moving backwards')
        reverse()
    elif command == 'stop':
        print('Stopping')
        stop_motor()
    else:
        print('Unknown command, stopping instead')
        stop_motor()

rospy.init_node('driver')

# subscribe to command topic with a string type message and call command_callback
rospy.Subscriber('command', String, command_callback)

# wait for a message to come in and loop forever
rospy.spin()

# in case of ctrl+c
print('Shutting down: stopping motors')
stop_motor()
GPIO.cleanup()
