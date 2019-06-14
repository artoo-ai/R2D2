from adafruit_servokit import ServoKit
import time

class AdafruitServoControl:
    """
    Use the Maestro board to control the servos.

    https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi?view=all

    pip install adafruit-circuitpython-servokit

    sudo apt-get install python-smbus
    sudo apt-get install i2c-tools

    """

    def __init__(self, min_servo=750, max_servo=2500):
        """
        Initialize the maestro.  Set the comm port if you need to set the specific.  Set the min
        and max servo pulse width if not default.
        :param comm_port: Comm port of the maestro.
        :param min_servo: Min servo pulse width in quarter seconds.
        :param max_servo: Max servo pulse width in quarter seconds.
        """
        self.kit = None                                         # Hold the Adafruit Servokit

        self.min_servo = min_servo                              # Min pulse width of servo in quarter seconds
        self.max_servo = max_servo                              # Max pulse width of servo in quarter seconds

        self.setup_controller()                                 # Setup maestro

    def setup_controller(self):
        """
        Setup the maestro controller
        :return:
        """
        self.kit = ServoKit(channels=16)
        self.kit.servo[0].set_pulse_width_range(self.min_servo, self.max_servo)
        self.kit.servo[1].set_pulse_width_range(self.min_servo, self.max_servo)
        self.kit.servo[0].angle = 0
        self.kit.servo[1].angle = 0

    def center(self):
        """
        Center the servo.
        :return:
        """
        self.set_angle(0.0)  # CENTER (6000)

    def set_angle(self, chan, angle):
        """
        Set the servo to the given angle.
        :param chan: Servo channel to move
        :param angle: Angle to move the servo
        :return:
        """
        if angle > 180:
            angle = 180
        if angle < 0:
            angle = 0

        self.kit.servo[chan].angle = angle
        print("Chan: " + str(chan) + " angle: " + str(self.kit.servo[chan].angle))


if __name__ == '__main__':
    servo = AdafruitServoControl()
    time.sleep(1.0)
    servo.set_angle(0, 180)
    servo.set_angle(1, 180)
    time.sleep(1.5)
    servo.set_angle(0, 0)
    servo.set_angle(1, 0)
