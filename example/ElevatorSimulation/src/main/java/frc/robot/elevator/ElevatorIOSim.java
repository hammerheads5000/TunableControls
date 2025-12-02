// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(2);

    private double voltsOutput = 0;

    private final ElevatorSim sim = new ElevatorSim(
            gearbox,
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.CARRIAGE_MASS.in(Kilograms),
            ElevatorConstants.DRUM_RADIUS.in(Meters),
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            ElevatorConstants.MAX_HEIGHT.in(Meters),
            true,
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            0.01,
            0.0);

    public ElevatorIOSim() {}

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.setInput(voltsOutput);
        sim.update(0.02);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        inputs.position = Meters.of(sim.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

        inputs.outputCurrent = Amps.of(sim.getCurrentDrawAmps());
        // inputs.outputVoltage = Volts.of(gearbox.getVoltage(
        //         gearbox.getTorque(sim.getCurrentDrawAmps()),
        //         sim.getVelocityMetersPerSecond() / ElevatorConstants.DRUM_RADIUS.in(Meters)));
        inputs.outputVoltage = Volts.of(voltsOutput);
    }

    @Override
    public void setVoltageOut(Voltage output) {
        voltsOutput = output.in(Volts);
    }
}
