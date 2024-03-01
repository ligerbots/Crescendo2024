// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class BackupFeed extends Command {
    Shooter m_shooter;
    Timer m_timer = new Timer();

    /** Creates a new BackupFeed. */
    public BackupFeed(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.setFeederSpeed(Shooter.BACKUP_FEED_SPEED);
        m_shooter.setShooterSpeeds(Shooter.BACKUP_SHOOTER_SPEED, Shooter.BACKUP_SHOOTER_SPEED);
        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.turnOffShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(Shooter.BACKUP_FEED_TIME);
    }
}
