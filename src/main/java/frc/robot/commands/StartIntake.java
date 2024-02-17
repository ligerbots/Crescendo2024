// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StartIntake extends SequentialCommandGroup {
  /** Creates a new startIntake. */
  public StartIntake(Intake intake, Shooter shooter, XboxController xbox, Elevator elevator, DoubleSupplier length, DoubleSupplier angle) /*add shooter pivot*/{
    addCommands(
      new SetElevatorLength(elevator, length),
      // set angle???
      new InstantCommand(shooter::turnOnFeeder),
      new RunIntakeWaitNote(intake),
      new BackupFeed(shooter),
      new Rumble(xbox)
    );
  }
}
