// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class ActiveSetShooter extends Command {
    private final Shooter m_shooter;
    private final ShooterPivot m_shooterPivot;
    private final Supplier<Shooter.ShooterValues>  m_valueSupplier;

    private static final double PIVOT_WAIT_TIME = 0.1;

    enum State {START, WAIT_FOR_PIVOT, BACKUP_NOTE, SPEED_UP_SHOOTER};
    private State m_state;
    Timer m_timer = new Timer();

    /** Creates a new ActiveSpeedUpShooter. */
    public ActiveSetShooter(Shooter shooter, ShooterPivot shootPivot, Supplier<Shooter.ShooterValues> valueSupplier) {
        m_shooter = shooter;
        m_shooterPivot = shootPivot;
        m_valueSupplier = valueSupplier;

        addRequirements(shooter);
        addRequirements(shootPivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_state = State.START;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_state == State.START) {
            // Start the Pivot turning
            Shooter.ShooterValues shootValues = m_valueSupplier.get();
            m_shooterPivot.setAngle(shootValues.shootAngle);
            m_timer.restart();
            m_state = State.WAIT_FOR_PIVOT;
        }

        if (m_state == State.WAIT_FOR_PIVOT && m_timer.hasElapsed(PIVOT_WAIT_TIME)) {
            // start the feeder motor and timer to back the NOTE a bit
            m_shooter.setFeederSpeed(Shooter.BACKUP_FEED_SPEED);
            m_shooter.setShooterSpeeds(Shooter.BACKUP_SHOOTER_SPEED, Shooter.BACKUP_SHOOTER_SPEED);
            m_timer.restart();
            m_state = State.BACKUP_NOTE;
        }

        if (m_state == State.BACKUP_NOTE) {
            // TODO change this end condition
            if (m_timer.hasElapsed(Shooter.BACKUP_FEED_TIME)) {
                // NOTE should be out of the shooter wheels. Start the spin up.
                Shooter.ShooterValues shootValues = m_valueSupplier.get();
                m_shooter.turnOffFeeder();
                m_shooter.setShooterRpms(shootValues.leftRPM, shootValues.rightRPM);
                // does not actually do anything
                m_state = State.SPEED_UP_SHOOTER;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
