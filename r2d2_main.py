import time
from frame.arms import Arms

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
