// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class PrepareShooter extends ParallelCommandGroup {
    /** Creates a new PrepareShooter. */
    private final Supplier<Pose2d> m_robotPose;

    public PrepareShooter(ShooterPivot shooterPivot, Shooter shooter, DriveTrain driveTrain,
            XboxController xboxController, 
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        m_robotPose = () -> driveTrain.getPose();

        addCommands(
                new ActiveTiltShooter(shooterPivot, this::getShooterPitch),
                new ActiveTurnToHeadingWithDriving(driveTrain, this::getWantedHeading, translationXSupplier, translationYSupplier),
                new ActiveSpeedUpShooter(shooter, this::getLeftRPM, this::getRightRPM),
                new CheckPrepStatsAndRumble(shooterPivot, shooter, driveTrain, xboxController)
        );
    }

    private double getDistance() {
        return m_robotPose.get().getTranslation().getDistance(FieldConstants.flipTranslation(FieldConstants.SPEAKER));
    }

    private double getRightRPM() {
        // Will get distance update every time the function is called?
        return Shooter.calculateShooterSpeeds(getDistance()).leftRPM; 
    }

    private double getLeftRPM() {
        return Shooter.calculateShooterSpeeds(getDistance()).rightRPM;
    }

    private double getShooterPitch() {
        return Shooter.calculateShooterSpeeds(getDistance()).shootAngle;
    }

    private Rotation2d getWantedHeading() {
        return FieldConstants.flipTranslation(FieldConstants.SPEAKER).minus(m_robotPose.get().getTranslation())
                .getAngle();
    }
}
