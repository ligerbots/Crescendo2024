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
    DoubleSupplier m_length;

    public SetElevatorLength(Elevator elevator, DoubleSupplier length) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_elevator = elevator;
        m_length = length;

        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setLength(m_length.getAsDouble());
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
        double curLength = m_elevator.getLength(); //Could use string pot but currently uses motor one
        return Math.abs(curLength - m_length.getAsDouble()) < Elevator.REACHER_OFFSET_TOLERANCE_METERS;
    }
}