try:
    from subsystems.launch.wheels import *
    from subsystems.launch.arms import *
    from tachometer import *
    from PIDobj import *
except ImportError as e:
    print(e)
    print("---------------")
    from robot.subsystems.launch.wheels import *
    from robot.subsystems.launch.arms import *
    from robot.tachometer import *
    from robot.PIDobj import *

class Launch:
    def __init__(self):
        self.arms = Arms()
        self.launch_wheels = LaunchWheels()

        # self.angle_tach = Tachometer(angle_port)
        #
        # self.angle_PID = PID(0.027, 0.01, 0.001)
        # self.angle_PID.set_on_target_error(5, 0.3)
        # self.angle_PID.set_max_power(0.75)
        # self.angle_PID.set_min_power(-0.5)

    def set_angle(self, angle):
        self.arms.set_angle(angle)

    def set_power(self, power):
        self.launch_wheels.set_follow_power(power)

    def main(self):
        self.arms.main()
        # self.angle_tach.set_direction(self.arms.arm_motor.get())
        # self.angle_PID.update_position(self.angle_tach.get_ticks())
        # self.angle_PID.main_loop()

    def disable(self):
        self.arms.disable()
        self.launch_wheels.disable()
