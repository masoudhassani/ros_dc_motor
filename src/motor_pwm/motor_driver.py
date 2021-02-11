import RPi.GPIO as io

class MotorDriver:
    def __init__(self, max_speed=100, driver_pins=[20, 21, 24], pwm_freq=1000):
        self.max_speed = max_speed   # 0~100% of pwm duty cycle
        self.current_speed = 0
        self.driver_pins = driver_pins    # [fwd, rev, pwm signal] 
        self.pwm_freq = pwm_freq

        # GPIO setting
        io.setmode(io.BCM)     # bcm pin numbering
        io.setwarnings(False)

        # vnh driver with fwd, rev, and pwm signal pin
        if len(driver_pins) == 3:
            # set the GPIO modes 
            io.setup(driver_pins[0], io.OUT)
            io.setup(driver_pins[1], io.OUT)
            io.setup(driver_pins[2], io.OUT)

            # initialize pwm
            self.motor_pwm = io.PWM(driver_pins[2], pwm_freq) # pin number, frequency
            self.motor_pwm.start(0)  

            # initialize direction pins
            io.output(driver_pins[0], io.LOW)
            io.output(driver_pins[1], io.LOW)

        # drivers with 2 pwm pin as input such as TB9051FTG
        elif len(driver_pins) == 2:
            # set the GPIO modes 
            io.setup(driver_pins[0], io.OUT)
            io.setup(driver_pins[1], io.OUT)

            # initialize pwm
            fwd_pwm = io.PWM(driver_pins[0], pwm_freq)   # forward pwm signal
            rev_pwm = io.PWM(driver_pins[1], pwm_freq)   # reverse pwm signal
            fwd_pwm.start(0)
            rev_pwm.start(0)

        #TODO:a
        self.voltage = 12
        self.temperature = 47

    # turn all motors off
    def stop(self):
        if len(self.driver_pins) == 3:
            self.motor_pwm.ChangeDutyCycle(0)
            io.output(self.driver_pins[0], io.LOW)
            io.output(self.driver_pins[1], io.LOW)   

    #  forward direction
    def forward(self, duty_cycle):
        if len(self.driver_pins) == 3:
            self.motor_pwm.ChangeDutyCycle(duty_cycle)
            io.output(self.driver_pins[0], io.HIGH)
            io.output(self.driver_pins[1], io.LOW)

    #  reverse direction
    def reverse(self, duty_cycle):
        self.motor_pwm.ChangeDutyCycle(duty_cycle)
        io.output(self.driver_pins[0], io.LOW)
        io.output(self.driver_pins[1], io.HIGH)

    def set_pwm(self, command):
        """
        expects a command between -1 and 1 and converts it to duty cycle
        """
        command = min(max(command, -1), 1)
        self.current_speed = float(command*self.max_speed)

        if command < 0:
            self.reverse(-self.current_speed)

        else:
            self.forward(self.current_speed)

    def get_speed(self):
        """
        return current speed
        """
        return self.current_speed

    def get_status(self):
        """
        TODO: get hardware information from the motor
        """
        return {
            'temperature': self.temperature,
            'voltage': self.voltage
        }