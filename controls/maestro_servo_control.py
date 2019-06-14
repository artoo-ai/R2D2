import maestro
import time
import sys


class MaestroServoControl:
    """
    Use the Maestro board to control the servos.

    1. Disable console serial
    sudo raspi-config select interfacing options -> Serial -> No -> Yes save & exit

    2. Install pyserial
    python -m pip install pyserial (may be a bit slow)

    3. Clone Repo
    git clone https://github.com/FRC4564/Maestro

    4. Disable bluetooth uart
    sudo nano /boot/config.txt
    append to bottom: dtoverlay=pi3-disable-bt
    save

    """

    def __init__(self, comm_port='/dev/ttyACM0', min_servo=3000.0, max_servo=9000.0):
        """
        Initialize the maestro.  Set the comm port if you need to set the specific.  Set the min
        and max servo pulse width if not default.
        :param comm_port: Comm port of the maestro.
        :param min_servo: Min servo pulse width in quarter seconds.
        :param max_servo: Max servo pulse width in quarter seconds.
        """
        self.comm_port = comm_port                              # Serial comm port
        self.maestro = None                                     # Hold the maestro controller

        self.min_servo = min_servo                              # Min pulse width of servo in quarter seconds
        self.max_servo = max_servo                              # Max pulse width of servo in quarter seconds

        self.setup_controller()                                 # Setup maestro

    def setup_controller(self):
        """
        Setup the maestro controller
        :return:
        """
        self.maestro = maestro.Controller(self.comm_port)
        self.maestro.setRange(0, self.min_servo, self.max_servo)
        print("Min Servo: " + str(self.maestro.getMin(0)))
        print("Max Servo: " + str(self.maestro.getMax(0)))

    def shutdown(self):
        """
        Shutdown the object.  This will properly shutdown everything.
        :return:
        """
        self.maestro.close()

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

        curr_pos = self.maestro.getPosition(chan)
        print("Current Servo Position: " + str(chan) + " pw: " + str(curr_pos))

        pw = self.calc_quater_sec_pulse_width(angle)

        print("Servo: " + str(chan) + " angle: " + str(angle) + " pw: " + str(pw))

        self.maestro.setTarget(chan, int(pw))

        new_pos = self.maestro.getPosition(chan)
        print("New Servo Position: " + str(chan) + " pw: " + str(new_pos))

    def calc_quater_sec_pulse_width(self, angle):
        """
        Calculate the pulse width in quarter seconds based off the angle given.
        :param angle:
        :return: Duty cycle for servo based off angle.
        """
        if angle <= 0.0:
            return self.min_servo
        if angle >= 180.0:
            return self.max_servo

        return (self.max_servo + self.min_servo) / 180.0 * angle

    def set_speed(self, chan, speed):
        self.maestro.setSpeed(chan, speed)

    def set_accel(self, chan, speed):
        self.maestro.setAccel(chan, speed)


if __name__ == '__main__':
    #servos = MaestroServoControl(comm_port="/dev/ttyAMA0")
    #servos.set_speed(0, 10)
    #servos.set_speed(1, 10)
    #servos.set_angle(0, 90.0)
    #servos.set_angle(1, 90.0)
    #servos.shutdown()

    import maestro

    servo = maestro.Controller()
    servo.setAccel(0, 4)  # set servo 0 acceleration to 4
    servo.setTarget(0, 6000)  # set servo to move to center position
    servo.setSpeed(1, 10)  # set speed of servo 1
    x = servo.getPosition(1)  # get the current position of servo 1
    servo.close()
