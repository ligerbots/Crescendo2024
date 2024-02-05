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


public class TurnToHeadingWithDriving extends Command {

  private final DriveTrain m_driveTrain;
  private final PIDController turnHeadingPID;
  private Double wantedDegrees;
  private final Supplier<Rotation2d> m_wantedHeadingSupplier;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final static double kp = 0.2; 
  private final static double ki = 0.0; 
  private final static double kd = 0.0; 

  /**
   * Creates a new autoAim.
   */
  public TurnToHeadingWithDriving(DriveTrain driveTrain, Supplier<Rotation2d> wantedHeading, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    m_driveTrain = driveTrain;
    m_wantedHeadingSupplier = wantedHeading;
    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;

    turnHeadingPID = new PIDController(kp,ki,kd);// random number TODO
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnHeadingPID.reset();
    wantedDegrees = m_wantedHeadingSupplier.get().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // auto aiming using PID
      double speed = turnHeadingPID.calculate(m_driveTrain.getHeading().getDegrees(), wantedDegrees);
      m_driveTrain.joystickDrive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), -speed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getHeading().getDegrees() - wantedDegrees) < DriveTrain.ANGLE_TOLERANCE_DEGREES;
  }
}