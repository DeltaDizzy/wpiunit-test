// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class Constants {
    public static final double triggerDeadband = 0.05;
    public static class DriveConstants {
        public static final double gearRatio = 26.0;
        public static final Measure<Distance> wheelRadius = Inches.of(6.0);
        public static final Measure<Distance> wheelCircumference = wheelRadius.times(2).times(Math.PI);
        public static Measure<Distance> trackWidth = Inches.of(20);
        public static double MOI = 2.1; //kg m^2
        public static Measure<Mass> mass = Pounds.of(125);
    }
}
