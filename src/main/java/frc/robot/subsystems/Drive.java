// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.sysid.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.simulation.DriveSim;

public class Drive extends SubsystemBase {
  // Hardware
  TalonFX frontLeft = new TalonFX(0);
  TalonFX backLeft = new TalonFX(1);
  TalonFX frontRight = new TalonFX(2);
  TalonFX backRight = new TalonFX(3);
  Pigeon2 gyro = new Pigeon2(4);

  // Status signals
  StatusSignal<Double> leftDistance = frontLeft.getPosition();
  StatusSignal<Double> rightDistance = frontLeft.getPosition();

  // Control Requests
  DutyCycleOut percentOut = new DutyCycleOut(0, false, false, false, false);
  VoltageOut voltage = new VoltageOut(0, false, false, false, false);
  
  // SysId
  private final Unit<Velocity<Voltage>> VoltsPerSecond = Volts.per(Second);
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(VoltsPerSecond.of(1), Volts.of(7), Seconds.of(10)), 
    new SysIdRoutine.Mechanism(this::acceptSysIdVolts, this::sysidLog, this)
  );
  MutableMeasure<Voltage> sysidVolts = Volts.of(0).mutableCopy();
  MutableMeasure<Distance> sysidDistance = Meters.of(0).mutableCopy();
  MutableMeasure<Velocity<Distance>> sysidVelocity = MetersPerSecond.of(0).mutableCopy();
  Trigger sysidSkip;

  // Controls
  DifferentialDrive drive = new DifferentialDrive(new MotorControllerGroup(frontLeft, backLeft), new MotorControllerGroup(frontRight, backRight));
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth.in(Meters));
  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), leftDistance.getValue(), rightDistance.getValue(), new Pose2d(0, 3, new Rotation2d()));
  DriveSim sim;
  private final StructPublisher<Pose2d> posePublish = NetworkTableInstance.getDefault().getTable("Robot").getStructTopic("Pose", Pose2d.struct).publish();

  /** Creates a new Drive. */
  public Drive() {
    configureMotors();
    sim = new DriveSim(frontLeft, frontRight, backLeft, backRight, gyro, poseEstimator, posePublish);
  }

  private void configureMotors() {
    // create global motor config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyTimeThreshold = 1.275;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 1 / DriveConstants.wheelCircumference.in(Meters) * DriveConstants.gearRatio; // TODO: write out dimensional analysis

    // left side
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontLeft.getConfigurator().apply(config);
    backLeft.getConfigurator().apply(config);

    // right side
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontRight.getConfigurator().apply(config);
    backRight.getConfigurator().apply(config);

    // Followers
    backLeft.setControl(new Follower(frontLeft.getDeviceID(), false));
    backRight.setControl(new Follower(frontRight.getDeviceID(), false));
  }

  public Command arcadeDrive(DoubleSupplier throttle, DoubleSupplier turn) {
    return run(() -> {
      drive.arcadeDrive(throttle.getAsDouble(), turn.getAsDouble());
    });
  }

  private void acceptSysIdVolts(Measure<Voltage> volts) {
    frontLeft.setControl(voltage.withOutput(volts.in(Volts)));
    frontRight.setControl(voltage.withOutput(volts.in(Volts)));
  }

  private void sysidLog(MotorLog motorLog) {
    sysidVolts.mut_replace(frontLeft.getMotorVoltage().getValue(), Volts);
    sysidDistance.mut_replace(frontLeft.getPosition().getValue(), Meters);
    sysidVelocity.mut_replace(frontLeft.getVelocity().getValue() * DriveConstants.wheelCircumference.in(Meters), MetersPerSecond);
    motorLog.recordFrameLinear(sysidVolts, sysidDistance, sysidVelocity, "left");

    sysidVolts.mut_replace(frontRight.getMotorVoltage().getValue(), Volts);
    sysidDistance.mut_replace(frontRight.getPosition().getValue(), Meters);
    sysidVelocity.mut_replace(frontLeft.getVelocity().getValue() * DriveConstants.wheelCircumference.in(Meters), MetersPerSecond);
    motorLog.recordFrameLinear(sysidVolts, sysidDistance, sysidVelocity, "right");
  }

  public void setSysidSkipTrigger(Trigger skipButton) {
    sysidSkip = skipButton;
  }

  public Command characterize() {
    return Commands.sequence(
      runOnce(() -> SignalLogger.start()),
      Commands.waitSeconds(1),
      sysIdRoutine.quasistatic(Direction.kForward).until(sysidSkip).withTimeout(15),
      Commands.waitSeconds(1),
      sysIdRoutine.quasistatic(Direction.kReverse).until(sysidSkip).withTimeout(15),
      Commands.waitSeconds(1),
      sysIdRoutine.dynamic(Direction.kForward).until(sysidSkip).withTimeout(5),
      Commands.waitSeconds(1),
      sysIdRoutine.dynamic(Direction.kReverse).until(sysidSkip).withTimeout(5),
      runOnce(() -> SignalLogger.stop())
    ).withName(getName() + " Characterization");
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      poseEstimator.update(gyro.getRotation2d(), leftDistance.getValue(), rightDistance.getValue());
      posePublish.set(poseEstimator.getEstimatedPosition());
    }
  }

  @Override
  public void simulationPeriodic() {
    sim.run();
  }
}
