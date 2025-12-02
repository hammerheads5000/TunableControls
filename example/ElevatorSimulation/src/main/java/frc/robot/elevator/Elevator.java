// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableControls.TunableProfiledController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorVisualizer measuredVisualizer;
    private final ElevatorVisualizer setpointVisualizer;

    private final TunableProfiledController controller;

    public Elevator(ElevatorIO io) {
        this.io = io;

        controller = new TunableProfiledController(ElevatorConstants.TUNABLE_CONSTANTS);

        measuredVisualizer = new ElevatorVisualizer("Measured");
        setpointVisualizer = new ElevatorVisualizer("Setpoint");

        SmartDashboard.putData("Low", goToHeightCommand(true, ElevatorConstants.LOW_HEIGHT));
        SmartDashboard.putData("High", goToHeightCommand(true, ElevatorConstants.HIGH_HEIGHT));

        setGoal(ElevatorConstants.LOW_HEIGHT);
    }

    @Override
    public void periodic() {
        double thign = controller.calculate(inputs.position.in(Meters));
        io.setVoltageOut(Volts.of(thign));

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);


        // Logging
        Logger.recordOutput("Elevator/Goal", controller.getGoal());
        Logger.recordOutput("Elevator/SetpointPos", controller.getSetpoint().position);
        Logger.recordOutput("Elevator/SetpointVel", controller.getSetpoint().velocity);
        Logger.recordOutput("Elevator/PosError", controller.getPositionError());
        Logger.recordOutput("Elevator/VelError", controller.getVelocityError());
        Logger.recordOutput("Elevator/AccumulatedError", controller.getAccumulatedError());

        measuredVisualizer.update(inputs.position);
        setpointVisualizer.update(Meters.of(controller.getSetpoint().position));
    }

    /** Sets the desired height to go to (measured from ground) */
    public void setGoal(Distance goal) {
        controller.setGoal(goal.in(Meters));
    }

    public Distance getGoal() {
        return Meters.of(controller.getGoal());
    }

    public LinearVelocity getVelocity() {
        return inputs.velocity;
    }

    public Distance getHeight() {
        return inputs.position;
    }

    @AutoLogOutput
    /** Whether position is within tolerance of goal */
    public boolean atGoal() {
        return controller.atGoal();
    }

    public Command goToHeightCommand(boolean instant, Distance goal) {
        if (instant) {
            return this.runOnce(() -> setGoal(goal));
        }
        return this.startEnd(() -> setGoal(goal), () -> {}).until(this::atGoal);
    }
}
