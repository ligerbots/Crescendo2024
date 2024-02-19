// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetMultiNoteGeneric extends SequentialCommandGroup {
    public GetMultiNoteGeneric(Translation2d[] noteLocations, DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        List<Command> commandList = new ArrayList<>();
        Arrays.asList(noteLocations).forEach((n) -> commandList.add(new GetNoteX(n, driveTrain, noteVision, shooter, intake)));

        addCommands(commandList.toArray(new Command[commandList.size()]));
    }
}
