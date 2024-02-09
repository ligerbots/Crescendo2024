// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetPivotAngle extends Command {
  /** Creates a new SetShooterPivot. */
  ShooterPivot m_shooterPivot;
  DoubleSupplier m_angleRadians;
  double m_wantedAngleRadians;

  public SetPivotAngle(ShooterPivot shooterPivot, DoubleSupplier angleRadians) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterPivot = shooterPivot;
    m_angleRadians = angleRadians;
    addRequirements(m_shooterPivot);
  }

  public SetPivotAngle(ShooterPivot shooterPivot, double angleRadians) {
    // Use addRequirements() here to declare subsystem dependencies.
      this(shooterPivot, () -> angleRadians);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterPivot.setAngle(m_angleRadians.getAsDouble());
    m_wantedAngleRadians = m_angleRadians.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currAngle = m_shooterPivot.getAngleRadians();
    return (Math.abs(currAngle - m_angleRadians.getAsDouble()) < ShooterPivot.ANGLE_TOLERANCE_RADIAN);
    
  }
}
