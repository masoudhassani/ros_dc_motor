import RPi.GPIO as io

class ServoDriver:
    def __init__(self, max_speed=100, driver_pin=23, pwm_freq=50,
                init_angle=90, min_angle=0, max_angle=180):

        self.max_speed = max_speed   # 0~100% of pwm duty cycle
        self.current_angle = 0
        self.driver_pin = driver_pin    # [fwd, rev, pwm signal] 
        self.pwm_freq = pwm_freq
        self.init_angle = init_angle
        self.min_angle = min_angle
        self.max_angle = max_angle

        # GPIO setting
        io.setmode(io.BCM)     # bcm pin numbering
        io.setwarnings(False)

        # setup the pwm pin
        io.setup(driver_pin, io.OUT)

        # initialize pwm
        self.motor_pwm = io.PWM(driver_pin, pwm_freq)  # pin number, frequency
        self.motor_pwm.start(self.angle_to_duty(self.init_angle))   # servo starts at init angle

    def angle_to_duty(self, angle):
        angle = min(max(self.min_angle, angle), self.max_angle)
        duty = angle / 18 + 2
        return duty 

    # reset the servo to its init ngle
    def reset(self):
        center_duty = self.angle_to_duty(self.init_angle)
        self.motor_pwm.ChangeDutyCycle(center_duty)

    def set_pwm(self, command):
        """
        expects a command between -1 and 1 and converts it to duty cycle
        """
        command = min(max(command, -1), 1)
        
        # convert command to angle
        self.current_angle = self.min_angle + (command + 1) * (self.max_angle - self.min_angle) / 2  
        
        # calculate duty cycle and set the pwm signal
        duty = self.angle_to_duty(self.current_angle)
        self.motor_pwm.ChangeDutyCycle(duty)

    def get_angle(self):
        """
        return current angle
        """
        return self.current_angle
