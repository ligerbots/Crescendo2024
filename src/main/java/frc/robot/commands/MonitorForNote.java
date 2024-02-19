// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants;
import frc.robot.subsystems.NoteVision;

public class MonitorForNote extends Command {
    // distance to NOTE where we just go for it
    private static final double MIN_DISTANCE_END_CHECKS = 1.0; // meters

    private static final int NUM_SUCCESSIVE_FAILURES = 10;

    private final Supplier<Pose2d> m_poseProvider;
    private final Translation2d m_blueNotePosition;
    private final NoteVision m_noteVision;
    private final Command m_commandToCancel;

    private Translation2d m_notePosition;
    private int m_missedNoteCount;
    private double m_distanceToNote;

    public MonitorForNote(NoteVision noteVision, Supplier<Pose2d> poseProvider, Translation2d blueNotePosition, Command commandToCancel) {
        m_poseProvider = poseProvider;
        m_blueNotePosition = blueNotePosition;
        m_noteVision = noteVision;
        m_commandToCancel = commandToCancel;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_notePosition = FieldConstants.flipTranslation(m_blueNotePosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        Pose2d robotPose = m_poseProvider.get();
        m_distanceToNote = robotPose.getTranslation().getDistance(m_notePosition);

        if (m_distanceToNote >= NoteVision.MIN_VISIBLE_DISTANCE && m_distanceToNote <= NoteVision.MAX_VISIBLE_DISTANCE) {
            if (m_noteVision.checkForNote(robotPose, m_notePosition)) 
                m_missedNoteCount = 0;
            else
                m_missedNoteCount++;
        }
        else
            m_missedNoteCount = 0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (m_distanceToNote < MIN_DISTANCE_END_CHECKS) {
            // got to this distance seeing the NOTE, so just go for it. No more checks
            return true;
        }

        // if we are in vision range of the NOTE and have not seen it for N cycles, 
        //  abort this command group
        if (m_missedNoteCount >= NUM_SUCCESSIVE_FAILURES) {
            System.out.println("MonitorForNote cancelling command. distance=" + m_distanceToNote);
            if (m_commandToCancel != null) 
                CommandScheduler.getInstance().cancel(m_commandToCancel);
            return true;
        }

        return false;
    }
}