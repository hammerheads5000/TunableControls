// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;

public class ElevatorConstants {
    // Motor Configs
    public static final double GEAR_RATIO = 76.0 / 18;
    public static final Distance DRUM_RADIUS = Inches.of(1.125);

    public static final Mass CARRIAGE_MASS = Pounds.of(10);

    // Setpoints (from floor)
    public static final Distance MIN_HEIGHT = Meters.of(0);
    public static final Distance MAX_HEIGHT = Meters.of(2);

    public static final Distance LOW_HEIGHT = Meters.of(0.2);
    public static final Distance HIGH_HEIGHT = Meters.of(1.8);

    // Gains (input: meters, output: volts)
    public static final ControlConstants CONTROL_CONSTANTS = new ControlConstants()
            .withPID(10, 0, 0)
            .withFeedforward(2.9, 0.0)
            .withPhysical(0.0, 0.2)
            .withProfile(2.0, 3.0)
            .withTolerance(0.04, 0.1)
            .withIZone(0.4);

    public static final TunableControlConstants TUNABLE_CONSTANTS =
            new TunableControlConstants("Elevator", CONTROL_CONSTANTS);
}
