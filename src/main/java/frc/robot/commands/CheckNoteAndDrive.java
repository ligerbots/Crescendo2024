// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import javax.swing.text.Position;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NoteVision;

public class CheckNoteAndDrive extends Command {
  /** Creates a new checkNoteAndDrive. */
  private DriveTrain m_driveTrain;

  private final Command NoteCheck2ToNoteC5 = m_driveTrain.makePathFollowingCommand(null);
  private final Command NoteCheck2ToNoteC4 = m_driveTrain.makePathFollowingCommand(null);
  private final Command NoteCheck1ToNoteC1 = m_driveTrain.makePathFollowingCommand(null);
  private final Command NoteCheck1ToNoteC2 = m_driveTrain.makePathFollowingCommand(null);

  private int m_wantedNote;
  private int m_backUpNote;

  private static final Map<Integer, Pose2d> NOTE_POSITIONS = new HashMap<Integer, Pose2d>() {
    {
      put(1, FieldConstants.NOTE_C_1);
      put(2, FieldConstants.NOTE_C_2);
      put(3, FieldConstants.NOTE_C_3);
      put(4, FieldConstants.NOTE_C_4);
      put(5, FieldConstants.NOTE_C_5);
    }

  };

  private NoteVision m_noteVision;

  public CheckNoteAndDrive(DriveTrain driveTrain, NoteVision noteVision, int wantedNote, int backUpNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_noteVision = noteVision;
    m_backUpNote = backUpNote;
    m_wantedNote = wantedNote;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Pose2d> notes = m_noteVision.getNotes();
    if (notes.isEmpty()) {

    }
    for (Pose2d note : notes) {
      // goes through the notes in the note list and checks if the wanted note pose is
      // in there .2 is arbitrary and will need to be tuned for accuracy 
      if (note.getTranslation().getDistance(NOTE_POSITIONS.get(m_wantedNote).getTranslation()) <= .2) {

      }
    }

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
    return false;
  }
}
