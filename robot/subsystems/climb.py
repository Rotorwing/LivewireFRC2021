from wpilib import Talon, PWMVictorSPX, Servo

try:
    from IO_ports import *
except ImportError as e:
    from robot.IO_ports import *

class Climb:
    def __init__(self):
        self.rail = Talon(climb_motor_port)
        self.lift = PWMVictorSPX(climb_angle_motor_port)

        self.lock_in = 55
        self.lock_out = 0
        self.latch = Servo(climb_latch_servo_port)
        self.locked = False

        self.extended = 0
        # Possibly use current sensors in PDB the detect end points?

    def disable(self):
        self.rail.set(0)
        self.lift.set(0)

    def lock(self):
        self.latch.setAngle(self.lock_in)
        self.disable()
        self.locked = True

    def unlock(self):
        self.latch.setAngle(self.lock_out)
        self.locked = False

    def extend(self):

        if not self.locked and not self.extended == 1:
            self.rail.set(0.75)
        else:
            self.rail.set(0)

    def retract(self):
        if not self.locked and not self.extended == -1:
            self.rail.set(-1)
        else:
            self.rail.set(0)

    def raise_arm(self):
        self.lift.set(1)

    def lower_arm(self):
        self.lift.set(-1)
