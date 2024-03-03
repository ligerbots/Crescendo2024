// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class AutoSpeakerShot extends SequentialCommandGroup {
    DriveTrain m_driveTrain;

    /** Creates a new AutoSpeakerShot. */
    public AutoSpeakerShot(DriveTrain driveTrain, Shooter shooter, ShooterPivot shooterPivot) {

        m_driveTrain = driveTrain;

        addCommands(
            new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
            new ActiveSetShooter(shooter, shooterPivot, this::getShootValues)
                        .until(() -> (shooterPivot.angleWithinTolerance() && shooter.rpmWithinTolerance()))
                        .withTimeout(2.0),
            new TriggerShot(shooter)
        );
    }
    
    private Shooter.ShooterValues getShootValues() {
        return Shooter.calculateShooterSpeeds(m_driveTrain.getSpeakerDistance());
    }
}
