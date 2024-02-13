/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;


public class ActiveTurnToHeadingWithDriving extends Command {

  private final DriveTrain m_driveTrain;
  private final PIDController m_turnHeadingPID;
  private Double m_wantedDegrees;
  private final Supplier<Rotation2d> m_wantedHeadingSupplier;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final static double KP = 0.2; 
  private final static double KI = 0.0; 
  private final static double KD = 0.0; 

  /**
   * Creates a new autoAim.
   */
  public ActiveTurnToHeadingWithDriving(DriveTrain driveTrain, Supplier<Rotation2d> wantedHeading, DoubleSupplier translationXSupplier,

  DoubleSupplier translationYSupplier) {
    m_driveTrain = driveTrain;
    m_wantedHeadingSupplier = wantedHeading;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;

    m_turnHeadingPID = new PIDController(KP,KI,KD);// random number TODO
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnHeadingPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // auto aiming using PID
      m_wantedDegrees = m_wantedHeadingSupplier.get().getDegrees();
      double speed = m_turnHeadingPID.calculate(m_driveTrain.getHeading().getDegrees(), m_wantedDegrees);
      m_driveTrain.joystickDrive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), -speed );

      //For rumble command
      m_driveTrain.setOnGoalForActiveTurnRumble(Math.abs(m_driveTrain.getHeading().getDegrees() - m_wantedDegrees) < DriveTrain.ANGLE_TOLERANCE_DEGREES);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {return false;}
}