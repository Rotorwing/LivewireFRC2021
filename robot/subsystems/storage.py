from wpilib import Talon

class Storage:
    def __init__(self, intake_port, store_port):
        self.intake_motor = Talon(intake_port)
        self.store_motor = Talon(store_port)

    def disable(self):
        self.intake_motor.set(0)
        self.store_motor.set(0)
