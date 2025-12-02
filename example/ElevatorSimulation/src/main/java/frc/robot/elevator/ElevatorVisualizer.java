// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizer for elevator and algae manipualtor that includes Mechanism2d and 3d. Also visualizes currently held game
 * pieces
 */
public class ElevatorVisualizer {
    private final String name;

    private final LoggedMechanism2d mechanism =
            new LoggedMechanism2d(Inches.of(30).in(Meters), Inches.of(95).in(Meters), new Color8Bit(Color.kAliceBlue));
    private final LoggedMechanismLigament2d ligament;

    public ElevatorVisualizer(String name) {
        this.name = name;

        LoggedMechanismRoot2d root = mechanism.getRoot(name + " Root", 0.6, 0.1);
        ligament = root.append(new LoggedMechanismLigament2d(
                name + "Elevator", Inches.of(26).in(Meters), 90, 4.0, new Color8Bit(Color.kBlueViolet)));
    }

    public void update(Distance height) {
        ligament.setLength(height.in(Meters));
        Logger.recordOutput("Mechanism2d/" + name, mechanism);
    }
}
