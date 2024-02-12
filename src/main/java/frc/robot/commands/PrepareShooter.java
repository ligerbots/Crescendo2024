// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  private final Pose2d m_robotPose;

  public PrepareShooter(ShooterPivot ShooterPivot, Shooter Shooter, DriveTrain DriveTrain, CommandXboxController CommandXboxController) {
    m_robotPose = DriveTrain.getPose();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActiveTiltShooter(ShooterPivot, this::getShooterPitch),
      //TODO: figure out if need to write new command for turn while driving for dynamic turning. Current one turns once.
      new TurnToHeadingWithDriving(DriveTrain, this::getWantedHeading, this::getRobotX, this::getRobotY),
      new ActiveSpeedUpShooter(Shooter, this::getLeftRPM, this::getRightRPM),
      new PrepareShooter(ShooterPivot, Shooter, DriveTrain, CommandXboxController)
    );
  }

  private double getDistance() {
    return m_robotPose.getTranslation().getDistance(FieldConstants.flipTranslation(FieldConstants.SPEAKER));
  }

  private double getRightRPM() {
     return Shooter.calculateShooterSpeeds(getDistance()).rightSpeed; //Will get distance update every time the function is called?
  }

  private double getLeftRPM() {
    return Shooter.calculateShooterSpeeds(getDistance()).leftSpeed;
  }

  private double getShooterPitch() {
    return Shooter.calculateShooterSpeeds(getDistance()).shootAngle;
  }

  private double getRobotX() {
    return m_robotPose.getX();
  }

  private double getRobotY() {
    return m_robotPose.getY();
  }

  private Rotation2d getWantedHeading() {
    return FieldConstants.flipTranslation(FieldConstants.SPEAKER).minus(m_robotPose.getTranslation()).getAngle();
  }
}
