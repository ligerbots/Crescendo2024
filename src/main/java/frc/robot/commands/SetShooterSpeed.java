// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends Command {
  private final Shooter m_shooter;
  private final DoubleSupplier m_leftRpm, m_rightRpm;
  private static final double TOLERANCE = 100; // TODO Tune this later
  
  public SetShooterSpeed(Shooter shooter, DoubleSupplier leftRpm, DoubleSupplier rightRpm) {
    addRequirements(shooter);
    m_shooter = shooter;
    m_leftRpm = leftRpm;
    m_rightRpm = rightRpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterRpms(m_leftRpm.getAsDouble(), m_rightRpm.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getLeftRpm() - m_leftRpm.getAsDouble()) < TOLERANCE
        && Math.abs(m_shooter.getRightRpm() - m_rightRpm.getAsDouble()) < TOLERANCE;
  }
}
