// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.FieldConstants;
import frc.robot.subsystems.*;

public class GetMultiNoteGeneric extends SequentialCommandGroup {

    private static final Map<String, Translation2d> s_noteNameMap = new HashMap<>() {
        {
            put("C1", FieldConstants.NOTE_C_1);
            put("C2", FieldConstants.NOTE_C_2);
            put("C3", FieldConstants.NOTE_C_3);
            put("C4", FieldConstants.NOTE_C_4);
            put("C5", FieldConstants.NOTE_C_5);

            put("S1", FieldConstants.BLUE_NOTE_S_1);
            put("S2", FieldConstants.BLUE_NOTE_S_2);
            put("S3", FieldConstants.BLUE_NOTE_S_3);

            put("W", FieldConstants.DUMMY_NOTE_WAIT_FLAG);
        }
    };

    private static final double WAIT_INTERVAL_SECONDS = 1.0;

    // public GetMultiNoteGeneric(String noteSequence, DriveTrain driveTrain, NoteVision noteVision,
    //         Shooter shooter, ShooterPivot shooterPivot, Intake intake, Elevator elevator) {
    //     // Convenience constructor to use an encoded note sequence string i.e.
    //     // "C1-C2-S3"
    //     this(buildNoteList(noteSequence), driveTrain, noteVision, shooter, shooterPivot, intake, elevator);
    // }

    public GetMultiNoteGeneric(Translation2d[] noteLocations, DriveTrain driveTrain, NoteVision noteVision,
            Shooter shooter, ShooterPivot shooterPivot, Intake intake, Elevator elevator) {

        // Shoot the preloaded NOTE. Do not worry about turning (add??)
        addCommands(new AutoSpeakerShot(driveTrain, shooter, shooterPivot));

        // add all the fetching+shooting NOTE blocks
        for (int i=0; i < noteLocations.length; i++) {
            Translation2d note = noteLocations[i];

            if (FieldConstants.DUMMY_NOTE_WAIT_FLAG.equals(note)) {
                addCommands(new PrintCommand("Auto Wait"),  // TODO remove print when ready
                            new WaitCommand(WAIT_INTERVAL_SECONDS));
            } else if (FieldConstants.isCenterNote(note)) {
                boolean alwaysDriveBack = (i+1 < noteLocations.length) && ! FieldConstants.isCenterNote(noteLocations[i+1]);
                addCommands(new GetCenterNoteX(note, driveTrain, noteVision, shooter, shooterPivot, intake, elevator, alwaysDriveBack));
            } else {
                addCommands(new GetStageNoteX(note, driveTrain, noteVision, shooter, shooterPivot, intake, elevator));
            }
        }
    }

    public static Translation2d[] buildNoteList(String noteSequence) {

        String[] separateNoteNames = noteSequence.split("\\s*-\\s*");
        Translation2d[] noteCoordList = new Translation2d[separateNoteNames.length];

        for (int i = 0; i < separateNoteNames.length; i++) {
            Translation2d foundNote = s_noteNameMap.get(separateNoteNames[i].toUpperCase());
            if (foundNote == null) {
                throw new IllegalArgumentException("note name not found: " + separateNoteNames[i]);
            }
            noteCoordList[i] = foundNote;
        }
        return noteCoordList;
    }

    // public static void main(String[] args) {
    //     System.out.println(Arrays.asList(buildNoteList("S2-S3-S1")));
    // }
}
