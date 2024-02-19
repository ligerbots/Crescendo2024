// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepAmpShot extends SequentialCommandGroup {
  /** Creates a new PrepAmpShot. */
  public PrepAmpShot(Elevator elevator, Shooter shooter, ShooterPivot shooterPivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new SetElevatorLength(elevator, Elevator.AMP_SHOT_HEIGHT)
            .alongWith(new SetPivotAngle(shooterPivot, ShooterPivot.AMP_SHOT_ANGLE_RADIANS))

    );
  }
}
