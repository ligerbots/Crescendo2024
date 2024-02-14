// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareShooter extends ParallelCommandGroup {
  /** Creates a new PrepareShooter. */
  private final Supplier<Pose2d> m_robotPose;

  public PrepareShooter(ShooterPivot shooterPivot, Shooter shooter, DriveTrain driveTrain, CommandXboxController commandXboxController, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
    m_robotPose = () -> driveTrain.getPose();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActiveTiltShooter(shooterPivot, this::getShooterPitch),
      new ActiveTurnToHeadingWithDriving(driveTrain, this::getWantedHeading, translationXSupplier, translationYSupplier),
      new ActiveSpeedUpShooter(shooter, this::getLeftRPM, this::getRightRPM),
      new CheckPrepStatsAndRumble(shooterPivot, shooter, driveTrain, commandXboxController)
    );
  }

  private double getDistance() {
    return m_robotPose.get().getTranslation().getDistance(FieldConstants.flipTranslation(FieldConstants.SPEAKER));
  }

  private double getRightRPM() {
     return Shooter.calculateShooterSpeeds(getDistance()).leftRPM; //Will get distance update every time the function is called?
  }

  private double getLeftRPM() {
    return Shooter.calculateShooterSpeeds(getDistance()).rightRPM;
  }

  private double getShooterPitch() {
    return Shooter.calculateShooterSpeeds(getDistance()).shootAngle;
  }

  private Rotation2d getWantedHeading() {
    return FieldConstants.flipTranslation(FieldConstants.SPEAKER).minus(m_robotPose.get().getTranslation()).getAngle();
  }
}
