from r2d2.controls.adafruit_servo_driver import AdafruitServoControl
import time


class Arms:

    def __init__(self, top_arm_pin=0, bottom_arm_pin=1):
        """
        Set the pins and close the arms.
        :param top_arm_pin: Top Servo Pin.
        :param bottom_arm_pin: Bottom Servo Pin.
        """
        self.r2_top_arm = top_arm_pin
        self.r2_bottom_arm = bottom_arm_pin
        self.arm_servos = AdafruitServoControl()

        self.close_arms()

    def shutdown(self):
        """
        Shutdown the object.
        :return:
        """

    def open_arms(self):
        """
        Open the arms.
        :return:
        """
        self.arm_servos.set_angle(self.r2_top_arm, 180.0)
        self.arm_servos.set_angle(self.r2_bottom_arm, 180.0)

    def open_top_arm(self):
        """
        Open the top arm.
        :return:
        """
        self.arm_servos.set_angle(self.r2_top_arm, 180.0)

    def open_bottom_arm(self):
        """
        Open the bottom arm.
        :return:
        """
        self.arm_servos.set_angle(self.r2_bottom_arm, 180.0)

    def close_arms(self):
        """
        Close the arms.
        :return:
        """
        self.arm_servos.set_angle(self.r2_top_arm, 0.0)
        self.arm_servos.set_angle(self.r2_bottom_arm, 0.0)

    def close_top_arm(self):
        """
        Close the top arm.
        :return:
        """
        self.arm_servos.set_angle(self.r2_top_arm, 0.0)

    def close_bottom_arm(self):
        """
        Close the top arm.
        :return:
        """
        self.arm_servos.set_angle(self.r2_bottom_arm, 0.0)

    def top_say_hello(self):
        """
        Have the top arm wave.
        :return:
        """
        self.arm_servos.set_angle(self.r2_top_arm, 0.0)
        time.sleep(0.2)
        self.arm_servos.set_angle(self.r2_top_arm, 20.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_top_arm, 140.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_top_arm, 20.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_top_arm, 140.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_top_arm, 0.0)
        time.sleep(0.5)

    def bottom_say_hello(self):
        """
        Have the bottom arm wave.
        :return:
        """
        self.arm_servos.set_angle(self.r2_bottom_arm, 0.0)
        time.sleep(0.2)
        self.arm_servos.set_angle(self.r2_bottom_arm, 20.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_bottom_arm, 140.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_bottom_arm, 20.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_bottom_arm, 140.0)
        time.sleep(0.5)
        self.arm_servos.set_angle(self.r2_bottom_arm, 0.0)
        time.sleep(0.5)

    def test_arms(self):
        self.open_arms()
        time.sleep(0.8)
        self.close_arms()
        time.sleep(0.8)
        self.top_say_hello()
        time.sleep(0.3)
        self.bottom_say_hello()
        time.sleep(0.8)


if __name__ == '__main__':
    arms = Arms(17, 4)
    arms.open_arms()
    time.sleep(0.8)
    arms.close_arms()
    time.sleep(0.8)
    arms.top_say_hello()
    time.sleep(0.3)
    arms.bottom_say_hello()
    time.sleep(0.8)
    arms.shutdown()
