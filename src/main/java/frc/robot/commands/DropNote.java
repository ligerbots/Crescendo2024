// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropNote extends ParallelCommandGroup {
  /** Creates a new DropNote. */
  ShooterPivot m_shooterPivot; 
  Shooter m_shooter;
  public DropNote(ShooterPivot shooterPivot, Shooter shooter ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_shooterPivot = shooterPivot;
    m_shooter = shooter;
    
    addCommands(
      new SetPivotAngle(m_shooterPivot, Math.toRadians(0)),//random values that might work 
      new InstantCommand(m_shooter::turnOnFeeder),
      new InstantCommand(()-> m_shooter.setShooterRpms(250, 250))
    );
  }
}
