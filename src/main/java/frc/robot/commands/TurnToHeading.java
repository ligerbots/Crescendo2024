/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;

public class TurnToHeading extends Command {

  private final DriveTrain m_driveTrain;
  private ChassisSpeeds chassisSpeeds;
  private PhoenixPIDController turnHeadingPID;
  private double m_wantedHeading;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;

  /**
   * Creates a new autoAim.
   */
  public TurnToHeading(DriveTrain driveTrain, double wantedHeading, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    m_driveTrain = driveTrain;
    m_wantedHeading = wantedHeading;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;

    turnHeadingPID = new PhoenixPIDController(0.1, 0, 0);// random number TODO
    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnHeadingPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // auto aiming using PID
      double speed = turnHeadingPID.calculate(m_driveTrain.getHeading().getDegrees(), m_wantedHeading, 0);
      m_driveTrain.joystickDrive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), -speed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getHeading().getDegrees() - m_wantedHeading) < 0.1;
  }
}