// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetPivotAngle extends Command {
    ShooterPivot m_shooterPivot;
    DoubleSupplier m_angleProvider;

    public SetPivotAngle(ShooterPivot shooterPivot, DoubleSupplier angleRadians) {
        m_shooterPivot = shooterPivot;
        m_angleProvider = angleRadians;

        addRequirements(m_shooterPivot);
    }

    public SetPivotAngle(ShooterPivot shooterPivot, double angleRadians) {
        this(shooterPivot, () -> angleRadians);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterPivot.setAngle(m_angleProvider.getAsDouble());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shooterPivot.angleWithinTolerance();
    }
}
