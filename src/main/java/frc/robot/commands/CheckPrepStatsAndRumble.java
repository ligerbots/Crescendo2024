// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class CheckPrepStatsAndRumble extends Command {
    private final ShooterPivot m_shooterPivot;
    private final Shooter m_shooter;
    private final DriveTrain m_driveTrain;
    private final XboxController m_controller;

    // Value between 0-1. 0 is nothing 1 is a lot. 
    // 0.3 should be nonintrusive to the driver
    private static final double RUMBLE_INTENSITY = 0.3; 

    /** Creates a new CheckPrepStatsAndRumble. */
    public CheckPrepStatsAndRumble(ShooterPivot shooterPivot, Shooter shooter, DriveTrain driveTrain, XboxController xboxController) {
        m_shooterPivot = shooterPivot;
        m_shooter = shooter;
        m_driveTrain = driveTrain;
        m_controller = xboxController;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_shooterPivot.isWithinTolerence() && m_shooter.shooterSpeedIsWithinTolerence()
                && m_driveTrain.getOnGoalForActiveTurnRumble()) {
            // Sets rumble
            m_controller.setRumble(RumbleType.kBothRumble, RUMBLE_INTENSITY);
        } else {
            m_controller.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Turns it off once done
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
