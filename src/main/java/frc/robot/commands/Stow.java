// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class Stow extends ParallelCommandGroup {
  /** Creates a new StowingNotes. */
  public Stow(Shooter shooter, ShooterPivot shooterPivot, Elevator elevator) {
    addCommands(
        // Turn off shooter, just in case
        new InstantCommand(shooter::turnOffShooter),
        new SetElevatorLength(elevator, Elevator.STOW_LENGTH),
        new SetPivotAngle(shooterPivot, ShooterPivot.STOW_ANGLE_RADIANS)
    );
  }
}
