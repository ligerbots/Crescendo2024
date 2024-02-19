// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeWaitNote extends Command {

    static final double TIME_TO_RUN_INTAKE = 3.0;
    Intake m_intake;
    Timer m_timer = new Timer();

    /** Creates a new RunIntakeWaitNote. */
    public RunIntakeWaitNote(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();

        m_intake.intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stop(); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(TIME_TO_RUN_INTAKE);
        // another way could be a beam break sensor.
        // another way could be how the motors slow down
    }
}