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
        this(shooter, 1000.0);
    }
     
    public DropNote(Shooter shooter, double rpm) {
        addCommands(
                new InstantCommand(() -> shooter.setShooterRpms(rpm, rpm)),
                new InstantCommand(shooter::speakerShot)
        );
    }
}
