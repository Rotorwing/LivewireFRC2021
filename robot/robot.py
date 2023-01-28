import wpilib
try:
    from tachometer import *
    from IO_ports import *
    from button_map import *
    from subsystems.drive import *
    from subsystems.intake import *
    from subsystems.storage import *
    from subsystems.launch.launch import *
    from subsystems.climb import *
except ImportError as e:
    print(e)
    print("---------------")
    from robot.tachometer import *
    from robot.IO_ports import *
    from robot.button_map import *
    from robot.subsystems.drive import *
    from robot.subsystems.intake import *
    from robot.subsystems.storage import *
    from robot.subsystems.launch.launch import *
    from robot.subsystems.climb import *

import time
from wpilib.trajectory import TrajectoryConfig, constraint
from wpilib.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        print("< Robot Code Started >")

        self.PDB = wpilib.PowerDistributionPanel()

        #self.launchWheels = LaunchWheels()
        self.drivetrain = Drive()
        self.drivetrain.set_trajectory_config(TrajectoryConfig(1.5, 0.25))
        #self.drivetrain.trajectory.addConstraint(autoVoltageConstraint)


        self.launch = Launch()
        self.climb = Climb()

        self.drive_stick = wpilib.Joystick(0)
        self.aux_stick = wpilib.Joystick(1)

        self.timer = wpilib.Timer()


        self.intake_motor = Talon(intake_motor_port)
        self.stage_motor = Talon(staging_motor_port)
        self.storage_motor = Talon(storage_motor_port)

        self.table = NetworkTables.getTable("datatable")
        self.RPM_entry = self.table.getEntry("RPM")
        self.angle_entry = self.table.getEntry("angle")
        self.X_entry = self.table.getEntry("X")
        self.vision_freeze = self.table.getEntry("frozen")

        self.auto_mode = 0  # 0: fold out, 1: shoot, 2:drive
        self.t_timer = 0

        self.autoing_launch = False
        self.lock_vision = False

        self.climb_amp_timer = 0

    def autonomousInit(self):
        self.vision_freeze.setBoolean(False)
        self.timer.reset()
        self.timer.start()
        #self.drivetrain.set_pathweaver_json("/home/lvuser/py/paths/PathWeaver/output/main.wpilib.json")
        #self.drivetrain.odometry.resetPosition(self.drivetrain.trajectory.sample(0).pose, Rotation2d(0))  # Pose2d(Translation2d(0.6298702570379434, 0.621787025703795), Rotation2d(0)), Rotation2d(0))
        # self.drivetrain.set_waypoints(
        # # Start at the origin facing the +X direction
        # Pose2d(0, 0, Rotation2d(0)),
        # # Pass through these two interior waypoints, making an 's' curve path
        # [
        #     Translation2d(1, 1),
        #     Translation2d(2, -1),
        #     Translation2d(3, 0),
        #     Translation2d(5, 0.5)
        # ],
        # # End 3 meters straight ahead of where we started, facing forward
        # Pose2d(6.5, 2, Rotation2d.fromDegrees(90))
        # )
        #for timer in range(0, 50):
        #    timer /= 10
        #    ramsete_out = self.drivetrain.ramsete.calculate(self.drivetrain.odometry.getPose(), self.drivetrain.trajectory.sample(timer))
        #    print(ramsete_out)
        #print(" < end preview > ")
        self.drivetrain.left_encoder.reset()
        self.drivetrain.right_encoder.reset()
        self.drivetrain.disable()
        self.launch.launch_wheels.left_PID.reset_int()
        self.launch.launch_wheels.right_PID.reset_int()
        self.launch.arms.angle_PID.reset_int()
        self.t_timer = 0

    def autonomousPeriodic(self):
        # if self.timer.get() < 3.5:
        #     self.climb.lift.set(-1)
        #     self.launch.arms.set(-0.25)
        # else:
        #     self.climb.lift.set(0)
        #     self.launch.arms.set(0)
        #     self.intake()
        if self.auto_mode == 0:
            self.auto_mode = 1

        #self.drivetrain.run_trajectory()
        if self.auto_mode == 1:
            if True:  # not self.RPM_entry.getDouble(-1) == -1:
                self.launch.launch_wheels.set_follow_RPM(90)
                self.launch.arms.set_angle(55)
                self.drivetrain.setBatV(self.ds.getBatteryVoltage())
                self.launch.arms.main(True)
                
                self.vision_freeze.setBoolean(True)
                if self.launch.arms.angle_PID.on_target():
                    if self.launch.launch_wheels.on_target():
                        self.lock_vision = True
                else:
                    self.vision_freeze.setBoolean(False)
                
                if self.lock_vision:
                    if self.timer.get()-self.t_timer > 1:
                        self.feed_out(0.4)
                    if self.timer.get()-self.t_timer > 8:
                        self.disable_feed()
                        self.launch.launch_wheels.disable()
                        self.launch.arms.set_angle(0)
                        self.auto_mode = 2
                        self.t_timer = self.timer.get()
                else:
                    self.t_timer = self.timer.get()
            else:
                print("<VISION ERROR>")
        if self.auto_mode == 2:
            self.launch.launch_wheels.set_follow_power(0.2)
            self.launch.arms.set_angle(0)
            self.launch.arms.main(True)
            if self.timer.get()-self.t_timer < 2:
                self.drivetrain.arcade_drive(0, -0.8, 1, 1)
            if self.timer.get()-self.t_timer < 4.5:
                self.climb.lower_arm()
            self.intake()
            if self.timer.get()-self.t_timer > 5.8:
                self.auto_mode = 3
        
        

    def teleopInit(self):
        self.climb.unlock()
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic(self):

        # <*> <*> <*> <*> <*> <*> <*>
        #  ------- < Drive > -------
        x_axis = self.drive_stick.getX()
        y_axis = max(-1, min(1, self.drive_stick.getY()*1.2))

        speed = 0.75
        if self.get_button(speed_boost_button):
            speed = 1

        self.drivetrain.setBatV(self.ds.getBatteryVoltage())
        self.drivetrain.arcade_drive(x_axis, -y_axis, speed, 1)

        #print(self.drivetrain.left_encoder.get(), self.drivetrain.right_encoder.get())

        # <*> <*> <*> <*> <*> <*> <*>
        # ------- < Launch > -------
        if self.get_button(manual_fire_button):
            manual_power = self.aux_stick.getThrottle()
            self.launch.launch_wheels.set_follow_power(manual_power)
            self.autoing_launch = False
        elif self.get_button(auto_fire_button) or self.get_button(trench_fire_button):
            if True: # not self.RPM_entry.getDouble(-1) == -1:
                if self.get_button(manual_fire_button):
                    self.feed_out()
                if self.get_button(trench_fire_button):
                    self.launch.launch_wheels.set_follow_RPM(110)
                    self.launch.arms.set_angle(45)
                else:
                    self.launch.launch_wheels.set_follow_RPM(90)
                    self.launch.arms.set_angle(55)

                if not self.autoing_launch:
                    self.drivetrain.angle_PID.reset_int()
                self.autoing_launch = True
                self.vision_freeze.setBoolean(True)
                if self.launch.arms.angle_PID.on_target():
                    
                    if self.launch.launch_wheels.on_target():
                        self.feed_out(0.75)
                        self.lock_vision = True
                else:
                    self.vision_freeze.setBoolean(False)
                
                if self.lock_vision:
                    pass
                else:
                    self.disable_feed()
        else:
            self.launch.launch_wheels.disable()
            if self.lock_vision:
                self.launch.arms.set_angle(0)
                self.lock_vision = False
            if self.launch.arms.angle_PID.on_target():
                self.autoing_launch = False
        
        self.launch.arms.main(self.autoing_launch)    
        
        #wpilib.SmartDashboard.putNumber("Launch Power", manual_power)
        wpilib.SmartDashboard.putNumber("Launch Angle", self.launch.arms.angle)

        if not self.autoing_launch:
            self.launch.arms.set(self.aux_stick.getY()*0.75)
            # print(self.launch.arms.angle)  # , self.launch.arms.analog.getVoltage()

        # <*> <*> <*> <*> <*> <*> <*>
        # ------- < Intake > -------
        if self.get_button(sweep_button):
            self.intake()
            self.launch.arms.set_angle(0)
        elif not self.autoing_launch:
            self.disable_feed()

        if self.get_button(back_sweep_button):
            self.intake_motor.set(1)

        # <*> <*> <*> <*> <*> <*> <*>
        #  ------- < Climb > -------
        wpilib.SmartDashboard.putNumber("climb_current", self.PDB.getCurrent(3))
        if self.get_button(climb_extend_button):
            if self.climb.extended == -1:
                self.climb.extended = 0

            if self.climb_amp_timer > 10 and self.PDB.getCurrent(3) > 12:
                self.climb.extended = 1
            self.climb.extend()
            self.climb_amp_timer += 1
        elif self.get_button(climb_retract_button):
            if self.climb.extended ==1:
                self.climb.extended = 0
            
            #if self.PDB.getCurrent() > 1000:
            #    self.climb.extended = -1
            self.climb.retract()
        else:
            self.climb.rail.set(0)
            self.climb_amp_timer = 0

        if self.get_button(climb_raise_button):
            self.climb.raise_arm()
        elif self.get_button(climb_lower_button):
            self.climb.lower_arm()
        else:
            self.climb.lift.set(0)

        if self.get_button(climb_lock_button):
            self.climb.lock()
        elif self.get_button(climb_unlock_button):
            self.climb.unlock()

        #if wpilib.DriverStation.getMatchTime() < 2:
        #    self.climb.lock()

    def disabledInit(self):
        self.launch.launch_wheels.disable()
        self.drivetrain.disable()
        self.disable_feed()
        self.climb.disable()
        self.launch.disable()

    def disable_feed(self):
        self.stage_motor.set(0)
        self.storage_motor.set(0)
        self.intake_motor.set(0)

    def feed_out(self, speed):
        self.storage_motor.set(speed)
        self.stage_motor.set(-1)

    def intake(self):
        self.stage_motor.set(1)
        self.launch.launch_wheels.set_follow_power(-0.3)
        self.storage_motor.set(-0.5)
        self.intake_motor.set(-0.75)

    def get_button(self, button):
        if button[1] == "drive":
            return self.drive_stick.getRawButton(button[0])
        else:
            return self.aux_stick.getRawButton(button[0])


if __name__ == "__main__":
    wpilib.run(MyRobot)
