// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class CheckPrepStatsAndRumble extends Command {
  /** Creates a new CheckPrepStatsAndRumble. */
  private final ShooterPivot m_shooterPivot;
  private final Shooter m_shooter;
  private final DriveTrain m_driveTrain;
  private final CommandXboxController m_controller;
  private final double m_intensity = 0.3; //Value bewteen 0-1. 0 is nothing 1 is alot. 0.3 should be nonintrusive to the driver

  public CheckPrepStatsAndRumble(ShooterPivot shooterPivot, Shooter shooter, DriveTrain driveTrain, CommandXboxController commandXboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterPivot = shooterPivot;
    m_shooter = shooter;
    m_driveTrain = driveTrain;
    m_controller = commandXboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterPivot.isWithinTolerence() && m_shooter.shooterSpeedIsWithinTolerence() && m_driveTrain.getOnGoalForActiveTurnRumble()) {
      //Sets rumble
      m_controller.getHID().setRumble(RumbleType.kLeftRumble, m_intensity);
      m_controller.getHID().setRumble(RumbleType.kRightRumble, m_intensity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Turns it off once done
    m_controller.getHID().setRumble(RumbleType.kLeftRumble, 0);
    m_controller.getHID().setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
