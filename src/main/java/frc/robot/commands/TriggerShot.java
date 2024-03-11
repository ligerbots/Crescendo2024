// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TriggerShot extends Command { 
    private static final double SHOOT_TIME = 0.25;  // seconds
    
    Shooter m_shooter;
    Timer m_timer = new Timer();

    /** Creates a new TriggerShot. */
    public TriggerShot(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {     
        // System.out.println("TriggerShot initialized");

        if (m_shooter.getSpeakerShootMode()) {
            // speaker shot
            m_shooter.speakerShot();
        } else {
            // amp shot
            m_shooter.ampShot();
        }

        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("TriggerShot ended interrupted = " + interrupted);
        m_shooter.turnOffShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(SHOOT_TIME);
    }
}
