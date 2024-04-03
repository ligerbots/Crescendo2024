// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// For testing. Needs to get separate left and right RPMs
public class PrepareTrapShot extends SequentialCommandGroup {

    /** Creates a new TrapShot. */
    public PrepareTrapShot(Shooter shooter, ShooterPivot shooterPivot, XboxController xboxController) {

        addCommands(
            new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
            new ActiveSetShooter(shooter, shooterPivot, this::getShootValues),
            new CheckPrepStatsAndRumble(shooterPivot, shooter, xboxController)
        );

        // addRequirements(shooter);
    }

    private Shooter.ShooterValues getShootValues() {
        return new Shooter.ShooterValues(Shooter.TRAP_RPM, Shooter.TRAP_RPM, ShooterPivot.STOW_ANGLE_RADIANS);
    }

}
