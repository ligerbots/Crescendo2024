// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareShooter extends ParallelCommandGroup {
  /** Creates a new PrepareShooter. */
  private final Pose2d m_robotPosition;

  public PrepareShooter(Pose2d robotPosition, ShooterPivot ShooterPivot, Shooter Shooter, DriveTrain DriveTrain) {
    m_robotPosition = robotPosition;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ActiveTiltShooter(ShooterPivot, this::getShooterPitch),
      new TurnToHeadingWithDriving(DriveTrain, this::getWantedHeading, this::getRobotX, this::getRobotY),
      new ActiveSpeedUpShooter(Shooter, this::getLeftRPM, this::getRightRPM)
      //TODO: Need to add rumble when done command. May want to add get goal offset methods to some subsystems.
    );
  }

  private double getDistance() {
    return m_robotPosition.getTranslation().getDistance(FieldConstants.flipPose(FieldConstants.SPEAKER).getTranslation());
  }

  private double getRightRPM() {
     return Shooter.calculateShooterSpeeds(getDistance()).rightSpeed; //TODO: Will get distance update?
  }

  private double getLeftRPM() {
    return Shooter.calculateShooterSpeeds(getDistance()).leftSpeed;
  }

  private double getShooterPitch() {
    return Shooter.calculateShooterSpeeds(getDistance()).shootAngle;
  }

  private double getRobotX() {
    return m_robotPosition.getX();
  }

  private double getRobotY() {
    return m_robotPosition.getY();
  }

  private Rotation2d getWantedHeading() {
    //TODO: Need to fill this in, have not gotton a chance yet. Will most likely do later today(2/10/2024) or tommorow
    return new Rotation2d(2,2);
  }
}
