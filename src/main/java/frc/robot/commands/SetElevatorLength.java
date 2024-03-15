// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorLength extends Command {
    Elevator m_elevator;
    DoubleSupplier m_lengthSupplier;
    final boolean m_inclAdjustment;

    /** Creates a new SetElevatorLength. */
    public SetElevatorLength(Elevator elevator, DoubleSupplier length, boolean includeAdjustment) {
        m_elevator = elevator;
        m_lengthSupplier = length;
        m_inclAdjustment = includeAdjustment;

        addRequirements(m_elevator);
    }

    // convenience constructor, takes a constant
    public SetElevatorLength(Elevator elevator, double length, boolean includeAdjustment) {
        this(elevator, ()->length, includeAdjustment);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setLength(m_lengthSupplier.getAsDouble(), m_inclAdjustment);
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
        return m_elevator.lengthWithinTolerance();
    }
}