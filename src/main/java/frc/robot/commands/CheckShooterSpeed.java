// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CheckShooterSpeed extends SequentialCommandGroup {
  /** Creates a new CheckShooterSpeed. */
  public CheckShooterSpeed(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(shooter::setupSysId, shooter),
      shooter.sysIdQuasistatic(Direction.kForward),
      shooter.sysIdQuasistatic(Direction.kReverse),
      shooter.sysIdDynamic(Direction.kForward),
      shooter.sysIdDynamic(Direction.kReverse)
    );
  }
}
