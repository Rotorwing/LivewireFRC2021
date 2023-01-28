from wpilib import Talon, drive, Encoder, ADXRS450_Gyro

try:
    from IO_ports import *
    from PIDobj import *
except ImportError as e:
    from robot.IO_ports import *
    from robot.PIDobj import *

from wpilib.trajectory import TrajectoryGenerator, Trajectory, TrajectoryUtil
from wpilib.kinematics import DifferentialDriveOdometry
from wpilib.controller import RamseteController
from wpilib.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
import wpilib
from math import sqrt
from time import time

class Drive:
    def __init__(self):
        self.gyro = ADXRS450_Gyro()
        self.left_motor = Talon(left_drive_motor_port)
        self.right_motor = Talon(right_drive_motor_port)

        self.left_encoder = Encoder(left_drive_encoder_port, left_drive_encoder_port+1)
        self.right_encoder = Encoder(right_drive_encoder_port, right_drive_encoder_port+1, reverseDirection=True)
        self.encoder_to_Meters = 0.6/360

        self.left_encoder.setDistancePerPulse(self.encoder_to_Meters)
        self.right_encoder.setDistancePerPulse(self.encoder_to_Meters)

        self.odometry = DifferentialDriveOdometry(Rotation2d(self.gyro.getAngle()), Pose2d())
        self.ramsete = RamseteController(15, 0.8)

        self.drive = drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.trajectory = Trajectory()
        self.trajectory_config = None

        self.batV = 13.0
        self.limit_avg = []
        self.limit_length = 3
        self.ttime = 0
        self.last_time = time()
        self.start_time = 0

        self.vx_PID = PID(0.07, 0.0, 0.01)  # PID(1.2, 0.05, 0.04)
        self.vx_PID.set_max_power(0.05)  # ramp up (20 frames)
        self.vx_PID.set_min_power(-0.5)  # ramp down (2 frames)
        self.omega_PID = PID(0.09, 0.02, 0.01)

        self.angle_PID = PID(0.01, 0.002, 0.0) #(0.3, 0.3, 0.05)
        self.angle_PID.set_on_target_error(10, 1)
        self.angle_PID.max_power = 0.25
        self.angle_PID.min_power = -0.25

        self.power_out = 0
        self.turn_out = 0

        self.last_left = 0
        self.last_right = 0
        self.last_gyro = 0

        self.E_Stop = False

    def turn_ramp(self, power):
        abs_power = abs(power)
        if abs_power < 0.1:
            return power*2.5
        elif abs_power < 0.4:
            return power*1.5+0.1
        elif abs_power < 1:
            return power*0.5+0.5
        else:
            return power

    def arcade_drive(self, x_axis, y_axis, drive_speed, turn_speed):
        time_change = time() - self.last_time
        self.last_time = time()
        #wpilib.SmartDashboard.putNumber("Left M", self.left_encoder.getDistance())
        #wpilib.SmartDashboard.putNumber("Right M", self.right_encoder.getDistance())
        self.odometry.update(Rotation2d.fromDegrees(-self.gyro.getAngle()), self.left_encoder.getDistance(), self.right_encoder.getDistance())
        pose = self.odometry.getPose()

        wpilib.SmartDashboard.putNumberArray("Odo", [float(pose.X()), float(pose.Y()), float(pose.rotation().degrees())])
        instant_vx = (self.left_encoder.getDistance()-self.last_left+self.right_encoder.getDistance()-self.last_right)/2*time_change
        wpilib.SmartDashboard.putNumber("Vx", instant_vx)
        V_limit = 1
        avg_V_limit = 1
        minV = 7.5
        if self.batV < 10:
            V_limit = max(0, (self.batV - minV) / (10.5 - minV))

        self.limit_avg.append(V_limit)
        if len(self.limit_avg) > self.limit_length:
            self.limit_avg = self.limit_avg[-self.limit_length:]

        avg_V_limit = sum(self.limit_avg)/len(self.limit_avg)
        #print("V limit: "+str(avg_V_limit))

        x_axis *= turn_speed*avg_V_limit
        y_axis *= drive_speed*avg_V_limit
        self.drive.arcadeDrive(-y_axis, x_axis)

    def disable(self):
        self.left_motor.set(0)
        self.right_motor.set(0)
        self.vx_PID.reset_int()
        self.omega_PID.reset_int()
        self.angle_PID.reset_int()
        self.left_encoder.reset()
        self.right_encoder.reset()

    def set_trajectory_config(self, config):
        """TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                            // Add kinematics to ensure max speed is actually obeyed
                            .setKinematics(DriveConstants.kDriveKinematics)
                            // Apply the voltage constraint
                            .addConstraint(autoVoltageConstraint);"""
        self.config = config

    def set_waypoints(self, start, waypoints, end):
        self.trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, self.config)

    def set_pathweaver_json(self, json_file):
        self.trajectory = TrajectoryUtil.fromPathweaverJson(json_file)

    def setBatV(self, V):
        self.batV = V

    def run_trajectory(self):
        time_change = time() - self.last_time
        self.last_time = time()

        if self.start_time == 0:
            self.start_time = time()

        # < odometry >
        self.odometry.update(Rotation2d.fromDegrees(-self.gyro.getAngle()), self.left_encoder.get()*self.encoder_to_Meters, self.right_encoder.get()*self.encoder_to_Meters)
        ramsete_out = self.ramsete.calculate(self.odometry.getPose(), self.trajectory.sample(self.ttime))
        instant_vx = (self.left_encoder.getDistance()-self.last_left+self.right_encoder.getDistance()-self.last_right)/2*time_change

        # < Run PIDs >
        self.vx_PID.set_target(ramsete_out.vx)
        self.omega_PID.set_target(ramsete_out.omega)
        self.main()

        # < SmartDashboard >
        wpilib.SmartDashboard.putNumber("Time", self.ttime)
        wpilib.SmartDashboard.putNumberArray("ram_out", [float(ramsete_out.vx), float(ramsete_out.vy), float(ramsete_out.omega)])
        wpilib.SmartDashboard.putString("vx", "{} -> {}".format(instant_vx, ramsete_out.vx))
        wpilib.SmartDashboard.putNumber("Power", self.power_out)

        # < Manage Target >
        max_speed = 0.9  # The maximum speed the target can go in seconds
        speed_m = 0.4  # speed multiplier for the target
        dif = self.odometry.getPose().relativeTo(self.trajectory.sample(self.ttime).pose)  # the x, y distance from the robot to the target
        mag = sqrt(dif.X()**2+dif.Y()**2)  # The distance from the robot to the target
        wpilib.SmartDashboard.putNumber("Dist", mag)

        #  Calculate target movement
        if ramsete_out.vx < 0:
            time_shift = max_speed*1
        else:
            if mag != 0:
                time_shift = min(max_speed, speed_m/mag)
            else:
                time_shift = max_speed

        #  Ramp up the target movement
        #if time() - self.start_time < 1:
        #    time_shift *= 0.3
        # print("Ramsete ("+str(self.ttime)+"): "+str(ramsete_out))
        wpilib.SmartDashboard.putNumber("Time Shift", time_shift)
        time_shift *= time_change*0.5
        self.ttime += time_shift


        self.last_left, self.last_right = self.left_encoder.getDistance(), self.right_encoder.getDistance()
        self.last_gyro = -self.gyro.getAngle()


    def main(self):
        # < Velocity >
        time_change = time() - self.last_time
        instant_vx = (self.left_encoder.getDistance()-self.last_left+self.right_encoder.getDistance()-self.last_right)/2*time_change
        self.vx_PID.update_position(instant_vx)

        if -0.04 < self.vx_PID.target < -0.04:
            self.power_out = 0
            self.vx_PID.reset_int()
        else:
            self.vx_PID.main_loop()
            self.power_out += self.vx_PID.get_power()
            self.power_out = max(-0.75, min(0.75, self.power_out))

        # < Omega >
        self.omega_PID.update_position((-self.gyro.getAngle()-self.last_gyro)*time_change)
        self.omega_PID.main_loop()

        if not self.E_Stop:
            if self.ttime > 0.1:
                angle_pwr = -self.omega_PID.get_power()
                self.arcade_drive(self.turn_ramp(angle_pwr), self.power_out, 1, 1)
        else:
            self.left_motor.set(0)
            self.right_motor.set(0)

        # self.last_time = time()

    def target_to(self, angle):
        self.angle_PID.update_position(angle)
        self.angle_PID.main_loop()
        pwr = self.angle_PID.get_power()
        wpilib.SmartDashboard.putNumber("X_pwr", pwr)
        if abs(angle) > 8:
            self.left_motor.set(-pwr)
            self.right_motor.set(-pwr)
        else:
            self.left_motor.set(0)
            self.right_motor.set(0)