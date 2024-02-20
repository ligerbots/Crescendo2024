// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetNoteX extends SequentialCommandGroup {

    // NOTE: the last path in the list is the "return" path done before shooting
    public static final Map<Translation2d, String[]> s_pathLookup = new HashMap<>() {
        {
            put(FieldConstants.NOTE_C_1, new String[] { "Start_1 to Note_C_1", "Start_2 to Note_C_1", "Note_C_1 to Shoot_1" });
            put(FieldConstants.NOTE_C_2, new String[] { "Start_1 to Note_C_2", "Start_2 to Note_C_2", "Shoot_1 to Note_C_2", "Note_C_2 to Shoot_1" });
            // put(FieldConstants.NOTE_C_3, "");
            put(FieldConstants.NOTE_C_4, new String[] { "Start_3 to Note_C_4" });
            put(FieldConstants.NOTE_C_5, new String[] { "Start_3 to Note_C_5" });

            put(FieldConstants.BLUE_NOTE_S_1, new String[] { "Start_1 to Note_S_1", "Start_2 to Note_S_1", "Start_3 to Note_S_1", "Note_S_2 to Note_S_1", null });
            put(FieldConstants.BLUE_NOTE_S_2, new String[] { "Start_1 to Note_S_2", "Start_2 to Note_S_2", "Start_3 to Note_S_2", "Note_S_1 to Note_S_2", "Note_S_3 to Note_S_2", null });
            put(FieldConstants.BLUE_NOTE_S_3, new String[] { "Start_1 to Note_S_3", "Start_2 to Note_S_3", "Start_3 to Note_S_3", "Note_S_2 to Note_S_3", null });
        }
    };

    private final Map<Pose2d,PathPlannerPath> m_candidateStartPaths = new LinkedHashMap<>();

    private PathPlannerPath m_returnPath; 

    private void initPaths(String[] pathnameArray) {
        for(int i=0; i<pathnameArray.length-1; i++) {
            PathPlannerPath path = DriveTrain.loadPath(pathnameArray[i]);
            if (path != null) {
                Pose2d startPose = path.getStartingDifferentialPose();
                m_candidateStartPaths.put(startPose, path);
            }
        }

        // return path is last item
        if (null != pathnameArray[pathnameArray.length-1]) {
            m_returnPath = DriveTrain.loadPath(pathnameArray[pathnameArray.length-1]);
        }
     
    }

    private final DriveTrain m_driveTrain;
    private final Translation2d m_targetNote; 

    public GetNoteX(Translation2d targetNote, DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        m_targetNote = targetNote;
        m_driveTrain = driveTrain;
        initPaths(s_pathLookup.get(targetNote));

        addCommands(new PrintCommand("**** GetNoteX-- Starting Auto target note: "+ targetNote));

        if (FieldConstants.isCenterNote(targetNote)) {
            // Use note "monitoring" for center notes only
            addCommands(new DeferredCommand(() -> m_driveTrain.followPath(getInitialPath()), Set.of(m_driveTrain))
                        .alongWith(new MonitorForNote(noteVision, () -> m_driveTrain.getPose(), m_targetNote, this)));
        } else {
            addCommands(new DeferredCommand(() -> m_driveTrain.followPath(getInitialPath()), Set.of(m_driveTrain)));
        }

        if (null != m_returnPath) {
            addCommands(m_driveTrain.followPath(m_returnPath));
        }

       addCommands(new PrintCommand("GetNoteX-- Finished target note: "+ targetNote));
    }

    private PathPlannerPath getInitialPath() {
        Pose2d pose = m_driveTrain.getPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        System.out.println("Starting getInitialPath " + poseBlue);
        // this part used when in center note area, if intended center note is not found
        if (poseBlue.getX() > FieldConstants.BLUE_WING_LINE_X_METERS) {
            Rotation2d heading = m_targetNote.minus(poseBlue.getTranslation()).getAngle();
            List<PathPoint> pathPoints = List.of(new PathPoint(poseBlue.getTranslation()), // starting pose
                    new PathPoint(m_targetNote));
            return PathPlannerPath.fromPathPoints(
                    pathPoints, // position, heading
                    new PathConstraints(DriveTrain.PATH_PLANNER_MAX_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ACCELERATION,
                            DriveTrain.PATH_PLANNER_MAX_ANGULAR_VELOCITY,
                            DriveTrain.PATH_PLANNER_MAX_ANGULAR_ACCELERATION),
                    new GoalEndState(0, heading, true)// velocity, acceleration
            );
        }

        Pose2d closestPathStart = poseBlue.nearest(new ArrayList<>(m_candidateStartPaths.keySet()));
        System.out.println("getInitialPath nearest = " + closestPathStart);
        return m_candidateStartPaths.get(closestPathStart);       
    }
}
