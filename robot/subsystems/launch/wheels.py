try:
    from tachometer import *
    from IO_ports import *
    from PIDobj import *
except ImportError as e:
    print(e)
    print("---------------")
    from robot.tachometer import *
    from robot.IO_ports import *
    from robot.PIDobj import *

from wpilib import Talon, DigitalInput, DutyCycle
import wpilib
from time import time

class LaunchWheels:
    def __init__(self):
        self.r_launch_motor = Talon(left_launch_motor_port)
        self.l_launch_motor = Talon(right_launch_motor_port)

        #self.left_tach = Tachometer(2)
        #self.right_tach = Tachometer(1)
        self.left_duty = DutyCycle(DigitalInput(left_launch_tach_port))
        self.right_duty = DutyCycle(DigitalInput(right_launch_tach_port))

        self.dif_PID = PID(1.6, 0.0, 0)
        self.dif_PID.set_max_power(0.25)
        self.dif_PID.set_min_power(-0.25)
        self.dif_PID.set_target(0)  # aim for 0 RPM dif
        self.last_loop = time()

        self.power_out = 0
        self.last_set_power = 0

        self.motor_cal = 0.9


    def set_follow_RPM(self, RPM):
        raise NotImplementedError

    def set_follow_power(self, power):
        if time()-self.last_loop > 2:
            self.dif_PID.reset_int()
            print("PID Reset")

        if power != 0:
            if abs(self.power_out-power) > 0.15 and abs(self.last_set_power-power) > 0.03:
                self.power_out = power*self.motor_cal
            else:
                #print(self.left_duty.getFrequency()*60)
                #print(self.right_duty.getFrequency()*60)
                r_direction = power/abs(power)
                if self.l_launch_motor.get() == 0:
                    l_direction = 1
                else:
                    l_direction = self.l_launch_motor.get()/abs(self.l_launch_motor.get())
                dif = self.right_duty.getFrequency()*-r_direction-self.left_duty.getFrequency()*-l_direction
                self.dif_PID.update_position(dif)
                self.dif_PID.main_loop()

                # RPM_mult = 0.012*power/abs(power)
                # output = (self.right_duty.getFrequency()-self.left_duty.getFrequency())*RPM_mult

                #print("Power: {}, RPM-dif: {}, Power-dif: {}".format(power, self.left_duty.getFrequency()-self.right_duty.getFrequency(),
                #                                                    output))

                self.r_launch_motor.set(power)
                self.power_out += power+self.dif_PID.get_power()*0.001
                self.l_launch_motor.set(self.power_out)

                wpilib.SmartDashboard.putNumber("rpm dif", dif)
                wpilib.SmartDashboard.putNumber("rpm pow", self.power_out)
                # print("{} - {} = {} => 0: {} + {}".format(self.right_duty.getFrequency(), self.left_duty.getFrequency(), dif, power, self.dif_PID.get_power()*direction ))
        else:
            self.l_launch_motor.set(0)
            self.r_launch_motor.set(0)

        self.last_loop = time()
        self.last_set_power = power

    def disable(self):
        self.r_launch_motor.set(0)
        self.l_launch_motor.set(0)
        self.dif_PID.reset_int()

    def start_tachs(self):
        self.left_tach.start_thread()
        self.right_tach.start_thread()

    def stop_tachs(self):
        self.left_tach.stop_thread()
        self.right_tach.stop_thread()
