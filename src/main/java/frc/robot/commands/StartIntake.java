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
  public StartIntake(Intake intake, Shooter shooter, Elevator elevator, ShooterPivot pivot) {
    addCommands(
      // turn off shooter completely, just in case
      new InstantCommand(shooter::turnOffShooter, shooter),
      // make sure the shooter is in the correct position
      new SetElevatorLength(elevator, Elevator.STOW_LENGTH)
          .alongWith(new SetPivotAngle(pivot, ShooterPivot.STOW_ANGLE_RADIANS)),
      new InstantCommand(shooter::turnOnFeeder, shooter),
      new InstantCommand(intake::intake, intake)
      // new RunIntakeWaitNote(intake),
      // new BackupFeed(shooter),
      // new Rumble(xbox)
    );
  }
}
