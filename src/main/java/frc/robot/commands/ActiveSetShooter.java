// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class ActiveSetShooter extends Command {
    private final Shooter m_shooter;
    private final ShooterPivot m_shooterPivot;
    private final Supplier<Shooter.ShooterValues>  m_valueSupplier;

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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Shooter.ShooterValues shootValues = m_valueSupplier.get();
        m_shooter.setShooterRpms(shootValues.leftRPM, shootValues.rightRPM);
        m_shooterPivot.setAngle(shootValues.shootAngle);
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