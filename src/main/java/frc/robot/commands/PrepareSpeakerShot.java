// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class PrepareSpeakerShot extends ParallelCommandGroup {
    private final DriveTrain m_driveTrain;

    /** Creates a new PrepareSpeakerShot. */
    public PrepareSpeakerShot(DriveTrain driveTrain, Shooter shooter, ShooterPivot shooterPivot, XboxController xboxController) {
        m_driveTrain = driveTrain;

        addCommands(
                // set shoot mode, so that TriggerShot can be a single command/button
                new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
                // this backs up the NOTE before turning on the shooter motors
                new ActiveSetShooter(shooter, shooterPivot, () -> shooter.getShootValues(m_driveTrain)),
                new CheckPrepStatsAndRumble(shooterPivot, shooter, xboxController)
                // NOTE do NOT turn off the shooter wheels
        );
        
    }
}
