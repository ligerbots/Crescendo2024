// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants;
import frc.robot.subsystems.NoteVision;

public class MonitorForNote extends Command {
  /** Creates a new MonitorForNote. */
  private final Supplier<Pose2d> m_poseProvider;
  private final Pose2d m_notePose;
  private boolean m_isNoteThere;
  private int m_noteThereCount;
  private final NoteVision m_noteVision;
  private final Command m_commandToCancel;

  public MonitorForNote(Supplier<Pose2d> poseProvider, Pose2d notePose, NoteVision noteVision,
      Command commandToCancel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_poseProvider = poseProvider;
    m_notePose = notePose;
    m_noteVision = noteVision;
    m_commandToCancel = commandToCancel;
    m_isNoteThere = true;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = m_poseProvider.get();
    m_isNoteThere = m_noteVision.checkForNote(robotPose, m_notePose);

    if (m_isNoteThere == true || robotPose.getX() < 4.5 || robotPose.getX() > 6.0) {
      m_noteThereCount = 0;
    } else if (m_isNoteThere == false) {
      m_noteThereCount++;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // random nums
    if (m_poseProvider.get().getTranslation().getDistance(FieldConstants.NOTE_C_1.getTranslation()) > 2.0) {
      return false;
    }
    if (m_isNoteThere == false && m_noteThereCount > 10) {
      CommandScheduler.getInstance().cancel(m_commandToCancel);
      return true;
    }
    if (m_poseProvider.get().getTranslation().getDistance(FieldConstants.NOTE_C_1.getTranslation()) < 0.2) {
      return true;
    }

    return false;
  }
}
