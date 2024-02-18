// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterRpmsAndWait extends Command {
    private final Shooter m_shooter;
    private final Supplier<Shooter.ShooterValues> m_valueSupplier;

    public SetShooterRpmsAndWait(Shooter shooter, Supplier<Shooter.ShooterValues> valueSupplier) {
        addRequirements(shooter);
        m_shooter = shooter;
        m_valueSupplier = valueSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Shooter.ShooterValues values = m_valueSupplier.get();
        m_shooter.setShooterRpms(values.leftRPM, values.rightRPM);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shooter.rpmWithinTolerance();
    }
}
