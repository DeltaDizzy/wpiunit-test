// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class DriveSim {
    private final TalonFXSimState left, right;
    private final Pigeon2SimState gyro;
    private final DifferentialDrivetrainSim drivetrainSim;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();

    public DriveSim(TalonFXSimState left, TalonFXSimState right, Pigeon2SimState gyro, DifferentialDrivePoseEstimator poseEstimator) {
        this.gyro = gyro;
        
        this.left = left;
        this.right = right;
        
        this.left.Orientation = ChassisReference.CounterClockwise_Positive;
        this.right.Orientation = ChassisReference.Clockwise_Positive;

        drivetrainSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),
            DriveConstants.gearRatio,
            DriveConstants.MOI,
            DriveConstants.mass.in(Kilograms),
            DriveConstants.wheelRadius.in(Meters),
            DriveConstants.trackWidth.in(Meters),
            null
        );

        this.poseEstimator = poseEstimator;
        SmartDashboard.putData(field);
    }

    private double metersToMotorRotations(double meters) {
        // meters = wheel rotations * wheel circumference
        // wheel rotation = meters / wheel circumference
        double wheelRotations = meters / DriveConstants.wheelCircumference.in(Meters);
        // 1 wheel rotation = gear ratio motor rotations
        double motorRotations = wheelRotations * DriveConstants.gearRatio;
        return motorRotations;
    }

    public void run() {
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        left.setSupplyVoltage(RobotController.getBatteryVoltage());
        right.setSupplyVoltage(RobotController.getBatteryVoltage());
        gyro.setSupplyVoltage(RobotController.getBatteryVoltage());

        drivetrainSim.setInputs(left.getMotorVoltage(), right.getMotorVoltage());
        drivetrainSim.update(0.02);

        left.setRawRotorPosition(metersToMotorRotations(drivetrainSim.getLeftPositionMeters()));
        left.setRotorVelocity(metersToMotorRotations(drivetrainSim.getLeftVelocityMetersPerSecond()));
        right.setRotorVelocity(metersToMotorRotations(drivetrainSim.getRightVelocityMetersPerSecond()));
        right.setRawRotorPosition(metersToMotorRotations(drivetrainSim.getRightPositionMeters()));
        
        gyro.setRawYaw(drivetrainSim.getHeading().getDegrees());
    }
}
