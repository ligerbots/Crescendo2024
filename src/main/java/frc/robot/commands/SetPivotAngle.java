// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetPivotAngle extends Command {
    private final ShooterPivot m_shooterPivot;
    private final DoubleSupplier m_angleProvider;
    private final boolean m_inclAdjustment;

    public SetPivotAngle(ShooterPivot shooterPivot, DoubleSupplier angleRadians, boolean includeAdjustment) {
        m_shooterPivot = shooterPivot;
        m_angleProvider = angleRadians;
        m_inclAdjustment = includeAdjustment;

        addRequirements(m_shooterPivot);
    }

    public SetPivotAngle(ShooterPivot shooterPivot, double angleRadians, boolean includeAdjustment) {
        this(shooterPivot, () -> angleRadians, includeAdjustment);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterPivot.setAngle(m_angleProvider.getAsDouble(), m_inclAdjustment);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shooterPivot.angleWithinTolerance();
    }
}
