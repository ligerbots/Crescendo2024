// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class OutTakeTransferRotations extends Command {
  private Shooter m_shooter;
  private double m_initalRotations;
  private final double NUMBER_OF_ROTATIONS = 2; //Number of times to outtake
  /** Creates a new OutTakeTransferRotations. */
  public OutTakeTransferRotations(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initalRotations = m_shooter.getFeederRotations();
    m_shooter.setFeederSpeed(Shooter.BACKUP_SHOOTER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.turnOffFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getFeederRotations() - m_initalRotations) >= NUMBER_OF_ROTATIONS;
  }
}
