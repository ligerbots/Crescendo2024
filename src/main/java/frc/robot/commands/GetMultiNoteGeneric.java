// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetMultiNoteGeneric extends SequentialCommandGroup {
    public GetMultiNoteGeneric(Translation2d[] noteLocations, DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        // add all the fetching+shooting NOTE blocks
        for (Translation2d note: noteLocations) {
            addCommands(new GetNoteX(note, driveTrain, noteVision, shooter, intake));
        }
    }
}
