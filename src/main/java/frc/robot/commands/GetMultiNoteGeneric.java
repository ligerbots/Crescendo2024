// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class GetMultiNoteGeneric extends SequentialCommandGroup {

    DriveTrain m_driveTrain;

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

    public GetMultiNoteGeneric(String noteSequence, DriveTrain driveTrain, NoteVision noteVision,
            Shooter shooter, ShooterPivot shooterPivot, Intake intake) {
        // Convenience constructor to use an encoded note sequence string i.e.
        // "C1-C2-S3"
        this(buildNoteList(noteSequence), driveTrain, noteVision, shooter, shooterPivot, intake);
    }

    public GetMultiNoteGeneric(Translation2d[] noteLocations, DriveTrain driveTrain, NoteVision noteVision,
            Shooter shooter, ShooterPivot shooterPivot, Intake intake) {

        m_driveTrain = driveTrain;

        // Shoot the preloaded NOTE. Do not worry about turning (add??)
        addCommands(
            new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
            new ActiveSetShooter(shooter, shooterPivot, this::getShootValues)
                .until(()-> (shooterPivot.angleWithinTolerance() && shooter.rpmWithinTolerance()) ),
            new TriggerShot(shooter),
            new Stow(shooter, shooterPivot)
        );

        // add all the fetching+shooting NOTE blocks
        for (Translation2d note : noteLocations) {
            if (FieldConstants.DUMMY_NOTE_WAIT_FLAG == note) {
                addCommands(new PrintCommand("starting wait " + WAIT_INTERVAL_SECONDS + " seconds"),
                            new WaitCommand(WAIT_INTERVAL_SECONDS),
                            new PrintCommand("finished wait " + WAIT_INTERVAL_SECONDS + " seconds"));
            } else if (FieldConstants.isCenterNote(note)) {
                addCommands(new GetCenterNoteX(note, driveTrain, noteVision, shooter, intake));
            } else {
                addCommands(new GetStageNoteX(note, driveTrain, noteVision, shooter, intake));
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

    private Shooter.ShooterValues getShootValues() {
        return Shooter.calculateShooterSpeeds(m_driveTrain.getSpeakerDistance()); 
    }

    public static void main(String[] args) {
        System.out.println(Arrays.asList(buildNoteList("S2-S3-S1")));
    }
}
