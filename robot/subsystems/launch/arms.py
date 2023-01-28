from wpilib import Talon, DigitalInput, AnalogInput, Counter, AnalogTriggerOutput, AnalogTrigger, AnalogTriggerType
import wpilib
try:
    from tachometer import *
    from PIDobj import *
    from IO_ports import *
except ImportError as e:
    print(e)
    print("---------------")
    from robot.tachometer import *
    from robot.PIDobj import *
    from robot.IO_ports import *

class Arms:
    def __init__(self):
        self.arm_motor = Talon(arm_motor_port)

        #self.angle_tach = Tachometer(arm_tach_port, delay=0.01)  # hall sensor measures 174.9 times per rotation
        #self.analog = AnalogInput(arm_tach_port)
        self.analog_trigger = AnalogTrigger(arm_tach_port)
        self.analog_trigger.setLimitsVoltage(3.0, 4.0)
        self.analog_trigger_output = AnalogTriggerOutput(self.analog_trigger, AnalogTriggerType.kRisingPulse)
        self.angle_counter = Counter(self.analog_trigger_output)
        self.error = 0
        self.angle = 0
        self.last = 0
        self.direction = False  # True: up, False: down

        self.top_limit = DigitalInput(top_launch_limit_port)
        self.lower_limit = DigitalInput(bottom_launch_limit_port)

        self.angle_PID = PID(0.03,0, 0)#, 0.01, 0.0)  #0.027, 0.01, 0.001)
        self.angle_PID.set_on_target_error(3, 0.3)
        self.angle_PID.set_max_power(0.75)
        self.angle_PID.set_min_power(-0.5)

    def set_angle(self, angle):
        self.angle_PID.set_target(angle)

    def set(self, power):
        _power = power
        if not self.top_limit.get():
            _power = min(_power, 0)
        if not self.lower_limit.get():
            _power = max(_power, 0)
        self.arm_motor.set(_power)

    def main(self, auto):

        if self.arm_motor.get() >= 0:
            self.direction = 1
        else:
            self.direction = -1

        raw_angle = self.angle_counter.get()
        delta_angle = int((raw_angle-self.last)/174.9*360)
        self.angle += delta_angle*self.direction

        self.angle_PID.update_position(self.angle)
        self.angle_PID.main_loop()
        power = self.angle_PID.get_power()
        #print("power "+str(power), "direction "+str(self.direction))

        if not self.top_limit.get():
            power = min(power, 0)
            self.angle = 61
        if not self.lower_limit.get():
            power = max(power, 0)
            self.angle = 0

        if self.angle == 0 and self.lower_limit.get():
            self.angle +=2

        if auto:
            self.arm_motor.set(power)
        self.last = raw_angle
        wpilib.SmartDashboard.putNumber("arm_angle", self.angle)

    def disable(self):
        self.arm_motor.disable()
        self.angle_PID.reset_int()
