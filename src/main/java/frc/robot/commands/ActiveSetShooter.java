// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class ActiveSetShooter extends Command {
    private final Shooter m_shooter;
    private final ShooterPivot m_shooterPivot;
    private final Supplier<Shooter.ShooterValues> m_valueSupplier;

    // Number of motor rotations
    private final double NUMBER_OF_ROTATIONS = 1.0;

    private static final double PIVOT_WAIT_TIME = 0.1; //0.2;

    enum State {START, WAIT_FOR_PIVOT, BACKUP_NOTE, SPEED_UP_SHOOTER};
    private State m_state;
    Timer m_timer = new Timer();
    double m_initialRotations;

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
        m_initialRotations = m_shooter.getFeederRotations();

        m_timer.restart();
        m_state = State.WAIT_FOR_PIVOT;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // always adjust the pivot
        Shooter.ShooterValues shootValues = m_valueSupplier.get();
        m_shooterPivot.setAngle(shootValues.shootAngle, true);

        if (m_state == State.WAIT_FOR_PIVOT && m_timer.hasElapsed(PIVOT_WAIT_TIME)) {
            // (m_shooterPivot.angleWithinTolerance() || m_timer.hasElapsed(PIVOT_WAIT_TIME))) {

            // start the feeder motor and timer to back the NOTE a bit
            m_shooter.setFeederSpeed(Shooter.BACKUP_FEED_SPEED);
            m_shooter.setShooterSpeeds(Shooter.BACKUP_SHOOTER_SPEED, Shooter.BACKUP_SHOOTER_SPEED);
            m_state = State.BACKUP_NOTE;
            m_timer.restart();
        }

        if (m_state == State.BACKUP_NOTE) {
            double pullback = Math.abs(m_shooter.getFeederRotations() - m_initialRotations);
            SmartDashboard.putNumber("shooter/pullback", pullback);

            boolean doneBackup = pullback >= NUMBER_OF_ROTATIONS;
            if (m_timer.hasElapsed(1)) {
                // test timer separately so we can log a problem
                DriverStation.reportError("Note pullback timed out", false);
                doneBackup = true;
            }

            if (doneBackup) {
                // NOTE should be out of the shooter wheels. Start the spin up.
                m_shooter.turnOffFeeder();
                m_state = State.SPEED_UP_SHOOTER;
            }
        }

        if (m_state == State.SPEED_UP_SHOOTER) {
            // Keep updating speed
            m_shooter.setShooterRpms(shootValues.leftRPM, shootValues.rightRPM);
        }

        SmartDashboard.putString("shooter/mode", m_state.toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("ActiveSetShooter end interrupt = " + interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
