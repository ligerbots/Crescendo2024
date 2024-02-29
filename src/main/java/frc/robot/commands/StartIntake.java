// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class StartIntake extends SequentialCommandGroup {
    /** Creates a new startIntake. */
    public StartIntake(Intake intake, Shooter shooter, ShooterPivot pivot, Elevator elevator) {
        addCommands(
                new Stow(shooter, pivot, elevator),
                new InstantCommand(shooter::turnOnFeeder, shooter),
                new InstantCommand(intake::intake, intake)
        // new RunIntakeWaitNote(intake),
        // new Rumble(xbox)
        );
    }
}
