// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class ActiveTiltShooter extends Command {

  private final ShooterPivot m_shooterPivot;
  private final DoubleSupplier m_goalAngleSupplier;

  /** Creates a new ActiveTiltShooter. */
  public ActiveTiltShooter(ShooterPivot shooterPivot, DoubleSupplier goalAngleSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterPivot);
    m_shooterPivot = shooterPivot;
    m_goalAngleSupplier = goalAngleSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivot.setAngle(m_goalAngleSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
