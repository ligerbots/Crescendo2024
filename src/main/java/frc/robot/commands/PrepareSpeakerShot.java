// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class PrepareSpeakerShot extends ParallelCommandGroup {
    private final DriveTrain m_driveTrain;

    /** Creates a new PrepareSpeakerShot. */
    public PrepareSpeakerShot(DriveTrain driveTrain, Shooter shooter, ShooterPivot shooterPivot,
            XboxController xboxController, DoubleSupplier leftJoystickXSupplier, DoubleSupplier leftJoystickYSupplier, DoubleSupplier rightJoystickXSupplier) {
        m_driveTrain = driveTrain;

        

        addCommands(
                // set shoot mode, so that TriggerShot can be a single command/button
                new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
                new ActiveSetShooter(shooter, shooterPivot, this::getShootValues),
                new CheckPrepStatsAndRumble(shooterPivot, shooter, driveTrain, xboxController)
                // NOTE do NOT turn off the shooter wheels
        );
        if (rightJoystickXSupplier.getAsDouble() != 0){
            addCommands(new Drive(driveTrain, leftJoystickXSupplier, leftJoystickYSupplier, rightJoystickXSupplier));
        }else{
        addCommands(new ActiveTurnToHeadingWithDriving(driveTrain, this::getWantedHeading, leftJoystickXSupplier, leftJoystickYSupplier));
        }
    }

    private double getDistance() {
        return m_driveTrain.getPose().getTranslation().getDistance(FieldConstants.flipTranslation(FieldConstants.SPEAKER));
    }

    private Shooter.ShooterValues getShootValues() {
        double distance = getDistance();
        SmartDashboard.putNumber("shooter/speakDistance", distance);
        return Shooter.calculateShooterSpeeds(distance); 
    }

    private Rotation2d getWantedHeading() {
        return FieldConstants.flipTranslation(FieldConstants.SPEAKER).minus(m_driveTrain.getPose().getTranslation())
                .getAngle();
    }


}
