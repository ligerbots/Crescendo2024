// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunWinchUntilHeight extends Command {
  /** Creates a new RunWinchUntil. */
  Climber m_climber;
  double m_wantedPosition;

  public RunWinchUntilHeight(Climber climber, double wantedPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_wantedPosition = wantedPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.run(1, 1);

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
    if (m_climber.getLeftPosition() >= m_wantedPosition && m_climber.getRightPosition() >= m_wantedPosition){
      return true;
    }
    return false ;
  }
}
