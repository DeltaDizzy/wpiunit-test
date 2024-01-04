// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  CommandXboxController driver = new CommandXboxController(0);
  Drive drive = new Drive();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setSysidSkipTrigger(driver.leftBumper().or(driver.rightBumper()));
    drive.setDefaultCommand(drive.arcadeDrive(this::getDriverThrottle, driver::getRightX));
  }

  private double getDriverThrottle() {
    double forward = MathUtil.applyDeadband(driver.getLeftTriggerAxis(), Constants.triggerDeadband);
    double reverse = MathUtil.applyDeadband(driver.getRightTriggerAxis(), Constants.triggerDeadband);
    if (forward >= 0) {
      return forward;
    } else return reverse;
  }

  public Command getAutonomousCommand() {
    return drive.characterize(); //drive.arcadeDrive(() -> 0.5, () -> 0.);
  }
}
