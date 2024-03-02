// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ResetClimber extends Command {
  Climber m_climber;
  private double m_initalRotationsLeft;
  private double m_initalRotationsRight;
  private final DoubleSupplier m_rotationsLeftSupplier; //Number of times to outtake
  private final DoubleSupplier m_rotationsRightSupplier; //Number of times to outtake

  private final double RESET_SPEED = 0.2; //TODO: Pick good value

  /** Creates a new ResetClimber. */
  public ResetClimber(Climber climber, DoubleSupplier rotationsLeftSupplier, DoubleSupplier rotationsRightSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(climber);
    m_rotationsLeftSupplier = rotationsLeftSupplier;
    m_rotationsRightSupplier = rotationsRightSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initalRotationsRight = m_climber.getRightPosition();
    m_initalRotationsLeft = m_climber.getLeftPosition();
    m_climber.run(RESET_SPEED, RESET_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_climber.getLeftPosition() - m_initalRotationsLeft) >= m_rotationsLeftSupplier.getAsDouble()) {
      m_climber.run(0, RESET_SPEED);
    } 
    if (Math.abs(m_climber.getRightPosition() - m_initalRotationsRight) >= m_rotationsRightSupplier.getAsDouble()) {
      m_climber.run(RESET_SPEED, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.run(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_climber.getLeftPosition() - m_initalRotationsLeft) >= m_rotationsLeftSupplier.getAsDouble() && Math.abs(m_climber.getRightPosition() - m_initalRotationsRight) >= m_rotationsRightSupplier.getAsDouble();
    
  }
}
