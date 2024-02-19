// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ActiveSpeedUpShooter extends Command {

    private final Shooter m_shooter;
    private final DoubleSupplier m_leftRpm;
    private final DoubleSupplier m_rightRpm;

    /** Creates a new ActiveSpeedUpShooter. */
    public ActiveSpeedUpShooter(Shooter shooter, DoubleSupplier leftRpm, DoubleSupplier rightRpm) {
        m_shooter = shooter;
        m_leftRpm = leftRpm;
        m_rightRpm = rightRpm;

        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setShooterRpms(m_leftRpm.getAsDouble(), m_rightRpm.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
