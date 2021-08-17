from wpilib import Talon, PWMVictorSPX, Servo

try:
    from IO_ports import *
except ImportError as e:
    from robot.IO_ports import *

class Climb:
    def __init__(self):
        self.rail = Talon(climb_motor_port)
        self.lift = PWMVictorSPX(climb_angle_motor_port)

        self.lock_in = 90
        self.lock_out = 0
        self.lock = Servo(climb_latch_servo_port)
        self.locked = False


        # Possibly use current sensors in PDB the detect end points?

    def disable(self):
        self.rail.set(0)
        self.lift.set(0)

    def lock(self):
        self.lock.set(self.lock_in)
        self.disable()
        self.locked = True

    def unlock(self):
        self.lock.set(self.lock_out)
        self.locked = False

    def extend(self):
        if not self.locked:
            self.rail.set(-0.75)

    def retract(self):
        self.rail.set(1)

    def raise_arm(self):
        self.lift(1)

    def lower_arm(self):
        self.lift.set(-1)
