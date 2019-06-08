from controls import rpi3_servo_control
import time


class R2d2Arms:

    def __init__(self, top_arm_pin, bottom_arm_pin):
        """
        Set the pins and close the arms.
        :param top_arm_pin: Top Servo Pin.
        :param bottom_arm_pin: Bottom Servo Pin.
        """
        self.r2_top_arm = rpi3_servo_control.Rpi3ServoControl(top_arm_pin)
        self.r2_bottom_arm = rpi3_servo_control.Rpi3ServoControl(bottom_arm_pin)

        self.close_arms()

    def shutdown(self):
        """
        Shutdown the object.
        :return:
        """
        self.r2_top_arm.shutdown()
        self.r2_bottom_arm.shutdown()

    def open_arms(self):
        """
        Open the arms.
        :return:
        """
        self.r2_top_arm.set_angle(180.0)
        self.r2_bottom_arm.set_angle(180.0)

    def close_arms(self):
        """
        Close the arms.
        :return:
        """
        self.r2_top_arm.set_angle(0.0)
        self.r2_bottom_arm.set_angle(0.0)

    def top_say_hello(self):
        """
        Have the top arm wave.
        :return:
        """
        self.r2_top_arm.set_angle(0.0)
        time.sleep(0.2)
        self.r2_top_arm.set_angle(20.0)
        time.sleep(0.5)
        self.r2_top_arm.set_angle(140.0)
        time.sleep(0.5)
        self.r2_top_arm.set_angle(20.0)
        time.sleep(0.5)
        self.r2_top_arm.set_angle(140.0)
        time.sleep(0.5)
        self.r2_top_arm.set_angle(0.0)
        time.sleep(0.5)

    def bottom_say_hello(self):
        """
        Have the bottom arm wave.
        :return:
        """
        self.r2_bottom_arm.set_angle(0.0)
        time.sleep(0.2)
        self.r2_bottom_arm.set_angle(20.0)
        time.sleep(0.5)
        self.r2_bottom_arm.set_angle(140.0)
        time.sleep(0.5)
        self.r2_bottom_arm.set_angle(20.0)
        time.sleep(0.5)
        self.r2_bottom_arm.set_angle(140.0)
        time.sleep(0.5)
        self.r2_bottom_arm.set_angle(0.0)
        time.sleep(0.5)


if __name__ == '__main__':
    arms = R2d2Arms(17, 4)
    arms.open_arms()
    time.sleep(0.8)
    arms.close_arms()
    time.sleep(0.8)
    arms.top_say_hello()
    time.sleep(0.3)
    arms.bottom_say_hello()
    time.sleep(0.8)
    arms.shutdown()
