// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.NoteVision;

public class CameraMode extends Command {
    NoteVision m_noteVision;
    AprilTagVision m_aprilTagVision;

    /** Creates a new CameraMode. */
    public CameraMode(NoteVision noteVision, AprilTagVision tagVision) {
        m_aprilTagVision = tagVision;
        m_noteVision = noteVision;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_aprilTagVision.setDriverMode(false);
        m_noteVision.setDriverMode(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
