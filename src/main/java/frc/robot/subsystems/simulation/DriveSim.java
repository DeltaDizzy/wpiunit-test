// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class DriveSim {
    private final TalonFX frontLeft, frontRight, backLeft, backRight;
    private final TalonFXSimState frontLeftSim, frontRightSim, backLeftSim, backRightSim;
    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final DifferentialDrivetrainSim drivetrainSim;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private final StructPublisher<Pose2d> posePub;

    public DriveSim(TalonFX frontLeft, TalonFX frontRight, TalonFX backLeft, TalonFX backRight, Pigeon2 gyro, DifferentialDrivePoseEstimator poseEstimator, StructPublisher<Pose2d> pose) {
        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        
        gyroSim = this.gyro.getSimState();
        frontLeftSim = this.frontLeft.getSimState();
        frontRightSim = this.frontRight.getSimState();
        backLeftSim = this.backLeft.getSimState();
        backRightSim = this.backRight.getSimState();

        frontLeftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        backLeftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        frontRightSim.Orientation = ChassisReference.Clockwise_Positive;
        backRightSim.Orientation = ChassisReference.Clockwise_Positive;

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
        posePub = pose;

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
        frontLeftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        frontRightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        gyroSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        drivetrainSim.setInputs(frontLeftSim.getMotorVoltage(), frontRightSim.getMotorVoltage());
        drivetrainSim.update(0.02);

        frontLeftSim.setRawRotorPosition(metersToMotorRotations(drivetrainSim.getLeftPositionMeters()));
        frontLeftSim.setRotorVelocity(metersToMotorRotations(drivetrainSim.getLeftVelocityMetersPerSecond()));
        frontRightSim.setRotorVelocity(metersToMotorRotations(drivetrainSim.getRightVelocityMetersPerSecond()));
        frontRightSim.setRawRotorPosition(metersToMotorRotations(drivetrainSim.getRightPositionMeters()));

        backLeftSim.setRawRotorPosition(metersToMotorRotations(drivetrainSim.getLeftPositionMeters()));
        backLeftSim.setRotorVelocity(metersToMotorRotations(drivetrainSim.getLeftVelocityMetersPerSecond()));
        backRightSim.setRotorVelocity(metersToMotorRotations(drivetrainSim.getRightVelocityMetersPerSecond()));
        backRightSim.setRawRotorPosition(metersToMotorRotations(drivetrainSim.getRightPositionMeters()));
        
        gyroSim.setRawYaw(drivetrainSim.getHeading().getDegrees());

        field.setRobotPose(poseEstimator.getEstimatedPosition());

        poseEstimator.update(gyro.getRotation2d(), frontLeft.getPosition().getValue(), frontRight.getPosition().getValue());
        posePub.set(poseEstimator.getEstimatedPosition());
    }
}
