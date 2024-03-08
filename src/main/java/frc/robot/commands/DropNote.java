// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Shooter;

public class DropNote extends ParallelCommandGroup {
    /** Creates a new DropNote. */
    public DropNote(Shooter shooter) {
        addCommands(
                new InstantCommand(() -> shooter.setShooterRpms(1000, 1000)),
                new InstantCommand(shooter::speakerShot));
    }
}
