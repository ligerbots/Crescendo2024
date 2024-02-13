// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeWaitNote extends Command {


    /** Creates a new RunIntakeWaitNote. */
    static final double TIME_TO_RUN_INTAKE = 3.0;
    Intake m_intake;
    Timer m_timer;
    
   
      public RunIntakeWaitNote(Intake intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = intake;
        m_timer = new Timer();

        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();

        m_intake.intake(); 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intake.stop(); // check what goes inside the parenthesis
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(TIME_TO_RUN_INTAKE);
        // another way could be garage sensors.
        // another way could be how the motors slow down
    }
}