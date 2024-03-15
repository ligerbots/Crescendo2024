// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;


public class PrepareAmpShot extends ParallelCommandGroup {
  public PrepareAmpShot(DriveTrain driveTrain, Elevator elevator, ShooterPivot shooterPivot, Shooter shooter) {

    addCommands(
      new SetElevatorLength(elevator, Elevator.AMP_SCORE_LENGTH),
      new SetPivotAngle(shooterPivot, ShooterPivot.AMP_SCORE_ANGLE_RADIANS, true),
      new InstantCommand(() -> shooter.setSpeakerShootMode(false)),
      new DriveToAmp(driveTrain)
    );
  }
}
