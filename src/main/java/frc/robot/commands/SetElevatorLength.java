// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorLength extends Command {

    /** Creates a new SetElevatorLength. */
    Elevator m_elevator;
    DoubleSupplier m_lengthSupplier;
    boolean m_useDoubleSupplier;
    double m_length;

    public SetElevatorLength(Elevator elevator, DoubleSupplier length, boolean useDoubleSupplier) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_elevator = elevator;
        m_lengthSupplier = length;
        m_useDoubleSupplier = useDoubleSupplier;

        addRequirements(m_elevator);
    }

    public SetElevatorLength(Elevator elevator, Double length, boolean useDoubleSupplier) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_elevator = elevator;
        m_useDoubleSupplier = useDoubleSupplier;

        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_useDoubleSupplier) {
            m_length = Elevator.limitElevatorLength(m_length);
        } else {
            m_length = Elevator.limitElevatorLength(m_lengthSupplier.getAsDouble());
        }
        m_elevator.setLength(m_length);
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
        double curLength = m_elevator.getLength(); // Could use string pot but currently uses motor one
        return Math.abs(curLength - m_length) < Elevator.LENGTH_TOLERANCE_METERS;
    }
}