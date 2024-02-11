// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NoteVision;

// this is only for the center 5 notes 
// we can make a different command if we want to generate a trajectory to the note on the fly 
public class CheckNoteAndDrive extends Command {
    private DriveTrain m_driveTrain;
    private NoteVision m_noteVision;

    private Command m_followTrajectory;

    private int m_wantedNote;
    private int m_backUpNote;

    private static final Map<Integer, Translation2d> NOTE_POSITIONS = new HashMap<Integer, Translation2d>() {
        {
            put(1, FieldConstants.NOTE_C_1);
            put(2, FieldConstants.NOTE_C_2);
            put(3, FieldConstants.NOTE_C_3);
            put(4, FieldConstants.NOTE_C_4);
            put(5, FieldConstants.NOTE_C_5);
        }
    };

    private static final Map<Integer, PathPlannerPath> ROBOT_PATHS = new HashMap<Integer, PathPlannerPath>() {
        {
            put(1, PathPlannerPath.fromPathFile("Test"));
            put(2, PathPlannerPath.fromPathFile("Test"));
            put(3, PathPlannerPath.fromPathFile("Test"));
            put(4, PathPlannerPath.fromPathFile("Test"));
            put(5, PathPlannerPath.fromPathFile("Test"));
        }
    };

    public CheckNoteAndDrive(DriveTrain driveTrain, NoteVision noteVision, int wantedNote, int backUpNote) {
        m_driveTrain = driveTrain;
        m_noteVision = noteVision;
        m_backUpNote = backUpNote;
        m_wantedNote = wantedNote;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_followTrajectory = null;
        Translation2d wantedNoteTranslation = NOTE_POSITIONS.get(m_wantedNote);

        if (m_noteVision.checkForNote(m_driveTrain.getPose(), wantedNoteTranslation))
            m_followTrajectory = m_driveTrain.FollowPath(ROBOT_PATHS.get(m_wantedNote));

        if (m_followTrajectory == null)
            m_followTrajectory = m_driveTrain.FollowPath(ROBOT_PATHS.get(m_backUpNote));

        m_followTrajectory.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_followTrajectory != null)
            m_followTrajectory.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // if interrupted, stop the follow trajectory
        DriverStation.reportError(String.format("NoteCheckAndDrive end interrupted = %s", interrupted), false);

        if (m_followTrajectory != null)
            m_followTrajectory.end(interrupted);
        m_followTrajectory = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_followTrajectory == null || m_followTrajectory.isFinished();
    }
}
