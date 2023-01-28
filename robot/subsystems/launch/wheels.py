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

        self.left_PID = PID(0.01, 0.001, 0)
        self.left_PID.set_max_power(0.25)
        self.left_PID.set_min_power(-0.25)
        self.left_PID.set_target(0)  # aim for 0 RPM dif
        self.left_PID.on_target_pos_error = 8

        self.right_PID = PID(self.left_PID.P, self.left_PID.I, self.left_PID.D)
        self.right_PID.set_max_power(0.25)
        self.right_PID.set_min_power(-0.25)
        self.right_PID.set_target(0)  # aim for 0 RPM dif
        self.right_PID.on_target_pos_error = 8

        self.PID_mult = 1.0

        self.last_loop = time()

        self.power_out = 0
        self.last_set_power = 0

        self.motor_cal = 0.9

        self.r_RPM = 0
        self.l_RPM = 0


    def set_follow_RPM(self, RPM):
        #raise NotImplementedError
        
        feed_forward = (RPM+4.4)/158.18 #(RPM+4.4)/158.18
        lpower = feed_forward
        rpower = feed_forward
        self.update_PIDs(RPM, RPM)
        if abs(self.l_RPM-RPM) < 15:
            lpower+=self.left_PID.get_power()*self.PID_mult
            rpower+=self.right_PID.get_power()*self.PID_mult
        else:
            self.left_PID.reset_int()
            self.right_PID.reset_int()
        self.l_launch_motor.set(lpower)
        self.r_launch_motor.set(rpower)
        wpilib.SmartDashboard.putNumber("l_rpm", self.l_RPM)
        wpilib.SmartDashboard.putNumber("r_rpm", self.r_RPM)
        wpilib.SmartDashboard.putNumber("PID_L", self.left_PID.get_power())
        wpilib.SmartDashboard.putNumber("PID_R", self.right_PID.get_power())

    def set_follow_power(self, power):
        if time()-self.last_loop > 2:
            self.left_PID.reset_int()
            print("PID Reset")

        if power != 0:
            if abs(self.power_out-power) > 0.15 and abs(self.last_set_power-power) > 0.03:
                self.power_out = power*self.motor_cal
            else:
                #print(self.left_duty.getFrequency()*60)
                #print(self.right_duty.getFrequency()*60)
                
                self.update_PIDs(self.r_RPM, None)

                # RPM_mult = 0.012*power/abs(power)
                # output = (self.right_duty.getFrequency()-self.left_duty.getFrequency())*RPM_mult

                #print("Power: {}, RPM-dif: {}, Power-dif: {}".format(power, self.left_duty.getFrequency()-self.right_duty.getFrequency(),
                #                                                    output))

                self.r_launch_motor.set(power)
                wpilib.SmartDashboard.putNumber("power", power)
                #self.l_launch_motor.set(power)
                wpilib.SmartDashboard.putNumber("l_rpm", self.l_RPM)
                wpilib.SmartDashboard.putNumber("r_rpm", self.r_RPM)
                self.power_out = power+self.left_PID.get_power()*self.PID_mult
                
                self.l_launch_motor.set(self.power_out)

                #
                wpilib.SmartDashboard.putNumber("PID_L", self.left_PID.get_power())
                # print("{} - {} = {} => 0: {} + {}".format(self.right_duty.getFrequency(), self.left_duty.getFrequency(), dif, power, self.left_PID.get_power()*direction ))
        else:
            self.l_launch_motor.set(0)
            self.r_launch_motor.set(0)

        self.last_loop = time()
        self.last_set_power = power

    def update_PIDs(self, l_target, r_target):
        if self.r_launch_motor.get() == 0:
            r_direction = 1
        else:
            r_direction = self.r_launch_motor.get()/abs(self.r_launch_motor.get())
        
        if self.l_launch_motor.get() == 0:
            l_direction = 1
        else:
            l_direction = self.l_launch_motor.get()/abs(self.l_launch_motor.get())
        
        self.l_RPM = self.right_duty.getFrequency()*l_direction
        self.r_RPM = self.left_duty.getFrequency()*r_direction

        l_dif =  l_target - self.l_RPM
        if r_target is not None:
            r_dif = r_target - self.r_RPM
        
        wpilib.SmartDashboard.putNumber("l_dir", l_dif)
        wpilib.SmartDashboard.putNumber("l_pow", self.l_launch_motor.get())
        wpilib.SmartDashboard.putNumber("r_pow", self.r_launch_motor.get())

        self.left_PID.update_position(-l_dif)
        self.left_PID.main_loop()

        if r_target is not None:
            self.right_PID.update_position(-r_dif)
            self.right_PID.main_loop()
        else:
            self.right_PID.update_position(0)
            self.right_PID.reset_int()

    def disable(self):
        self.r_launch_motor.set(0)
        self.l_launch_motor.set(0)
        self.left_PID.reset_int()

    def start_tachs(self):
        self.left_tach.start_thread()
        self.right_tach.start_thread()

    def stop_tachs(self):
        self.left_tach.stop_thread()
        self.right_tach.stop_thread()


    def on_target(self):
        if self.left_PID.I == 0:
            return self.right_PID.on_target()
        else:
            return self.right_PID.on_target() and self.right_PID.on_target()