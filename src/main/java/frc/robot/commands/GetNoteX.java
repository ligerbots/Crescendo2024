// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public abstract class GetNoteX extends SequentialCommandGroup {

    // NOTE: the last path in the list is the "return" path done before shooting (which is optional, set to null if not needed)
    protected static final Map<Translation2d, String[]> s_pathLookup = new HashMap<>() {
        {
            put(FieldConstants.NOTE_C_1, new String[] { "Start_1 to Note_C_1", "Start_2 to Note_C_1", "Note_C_1 to Shoot_1" });
            put(FieldConstants.NOTE_C_2, new String[] { "Start_1 to Note_C_2", "Start_2 to Note_C_2", "Shoot_1 to Note_C_2", "Note_C_2 to Shoot_1" });
            put(FieldConstants.NOTE_C_3, new String[] { "Start_2 to Note_C_3", "Note_C_3 to Shoot_3" });
            put(FieldConstants.NOTE_C_4, new String[] { "Start_3 to Note_C_4", "Note_C_4 to Shoot_2" });
            put(FieldConstants.NOTE_C_5, new String[] { "Start_3 to Note_C_5", "Note_C_5 to Shoot_2" });

            put(FieldConstants.BLUE_NOTE_S_1, new String[] { "Start_1 to Note_S_1", "Start_2 to Note_S_1", "Start_3 to Note_S_1", "Note_S_2 to Note_S_1", null });
            put(FieldConstants.BLUE_NOTE_S_2, new String[] { "Start_1 to Note_S_2", "Start_2 to Note_S_2", "Start_3 to Note_S_2", "Note_S_1 to Note_S_2", "Note_S_3 to Note_S_2", null });
            put(FieldConstants.BLUE_NOTE_S_3, new String[] { "Start_1 to Note_S_3", "Start_2 to Note_S_3", "Start_3 to Note_S_3", "Note_S_2 to Note_S_3", null });

            // put(FieldConstants.TEST_NOTE_S_3, new String[] { "Test_1 to Note_S_3", null });
        }
    };

    protected final Map<Pose2d,PathPlannerPath> m_candidateStartPaths = new LinkedHashMap<>();
    protected final DriveTrain m_driveTrain;
    protected final Translation2d m_targetNote; 

    private void initPaths(String[] pathnameArray) {
        for(int i=0; i<pathnameArray.length-1; i++) {
            PathPlannerPath path = DriveTrain.loadPath(pathnameArray[i]);
            if (path != null) {
                Pose2d startPose = path.getStartingDifferentialPose();
                m_candidateStartPaths.put(startPose, path);
            }
        }
    }

    public GetNoteX(Translation2d targetNote, DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        m_targetNote = targetNote;
        m_driveTrain = driveTrain;
        initPaths(s_pathLookup.get(targetNote));

        // addCommands(new PrintCommand("**** GetNoteX-- Starting Auto target note: "+ targetNote));
    }
}
