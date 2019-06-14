import RPi.GPIO as GPIO
import time
import sys


class Rpi3ServoControl:
    """
    Use the RPI3 to control the servos.
    """

    def __init__(self, servo_pin, init_angle=0.0):
        self.servo_pin = servo_pin
        self.gpio_pin = None
        self.last_angle = 0.0

        self.setup_pin()            # Setup GPIO pin
        self.set_angle(init_angle)

    def setup_pin(self):
        """
        Setup the GPIO pin.  Use the GPIO pin given in the constructor.
        :return:
        """
        GPIO.setmode(GPIO.BCM)                          # Setup GPIO pin to BCM
        GPIO.setup(self.servo_pin, GPIO.OUT)            # Set GPIO pin to output
        self.gpio_pin = GPIO.PWM(self.servo_pin, 50)    # GPIO PWM with 50Hz
        self.gpio_pin.start(2.5)                        # Initialization

    def shutdown(self):
        """
        Shutdown the object.  This will properly shutdown everything.
        :return:
        """
        self.gpio_pin.stop()
        GPIO.cleanup()

    def center(self):
        """
        Center the servo.
        :return:
        """
        self.set_angle(0.0)  # CENTER (7.5)

    def set_angle(self, angle):
        """
        Set the servo to the given angle.
        :param angle: Angle to move the servo
        :return:
        """
        if angle > 180:
            angle = 180
        if angle < 0:
            angle = 0

        # Cacluate the duty cycle to move to that angle
        dc = Rpi3ServoControl.calc_duty_cycle_from_angle(angle)
        print("Pin: " + str(self.servo_pin) + " Angle: " + str(angle) + " DC: " + str(dc))

        # Move the servo
        self.gpio_pin.ChangeDutyCycle(dc)

        time.sleep(0.1)

        # Record the last angle
        self.last_angle = angle

    @staticmethod
    def calc_duty_cycle_from_angle(angle):
        """
        Calculate the duty cycle based off the angle given.  Then
        move the servo based off this angle.
        :param angle:
        :return: Duty cycle for servo based off angle.
        """
        #return ((angle / 180.0) + 1.0) * 5.0
        return angle / 18.0 + 3.0


if __name__ == '__main__':
    if len(sys.argv) > 1:
        print("Using Servo Pin " + sys.argv[1])
        rpi3_servo = Rpi3ServoControl(int(sys.argv[1]))
    else:
        print("Using Servo Pin 4")
        rpi3_servo = Rpi3ServoControl(4)

    if len(sys.argv) > 2:
        angle_list = sys.argv[2:]
        print("Angles to try: " + str(sys.argv[2:]))
        for idx in range(len(angle_list)):
            rpi3_servo.set_angle(float(angle_list[idx]))
            time.sleep(0.5)
    rpi3_servo.shutdown()
