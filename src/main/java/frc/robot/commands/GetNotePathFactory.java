// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetNotePathFactory extends AutoCommandInterface {

    private static final Map<String, Translation2d> noteLookup = new HashMap<>() {
        {
            put("NOTE_C_1", FieldConstants.NOTE_C_1);
            put("NOTE_C_2", FieldConstants.NOTE_C_2);
            put("NOTE_C_3", FieldConstants.NOTE_C_3);
            put("NOTE_C_4", FieldConstants.NOTE_C_4);
            put("NOTE_C_5", FieldConstants.NOTE_C_5);
        }
    };

    List<PathPlannerPath> m_pathList;


    private PathPlannerPath m_longPath;
    private PathPlannerPath m_middlePath;
    private PathPlannerPath m_returnPath;
    private Translation2d m_wantedNote;
    private DriveTrain m_driveTrain;

    public GetNotePathFactory(DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake,
            String wantedNote, List<PathPlannerPath> paths) {
        m_driveTrain = driveTrain;
        m_wantedNote = noteLookup.get(wantedNote);
        m_pathList = paths;
        m_longPath = m_pathList.get(0);
        m_middlePath = m_pathList.get(1);
        m_returnPath = m_pathList.get(2);

    }

    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_longPath.getStartingDifferentialPose());
    };

    private PathPlannerPath getInitialPath() {
        Pose2d pose = m_driveTrain.getPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        if (poseBlue.getX() < FieldConstants.BLUE_WHITE_LINE_X_METERS) {
            return m_longPath;
        }

        if (poseBlue.getX() > FieldConstants.BLUE_WING_LINE_X_METERS) {
            Rotation2d heading = m_wantedNote.minus(poseBlue.getTranslation()).getAngle();
            List<PathPoint> pathPoints = List.of(new PathPoint(poseBlue.getTranslation()), // starting pose
                    new PathPoint(m_wantedNote));
            return PathPlannerPath.fromPathPoints(
                    pathPoints, // position, heading
                    new PathConstraints(DriveTrain.PATH_PLANNER_MAX_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ACCELERATION,
                            DriveTrain.PATH_PLANNER_MAX_ANGULAR_VELOCITY,
                            DriveTrain.PATH_PLANNER_MAX_ANGULAR_ACCELERATION),
                    new GoalEndState(0, heading, true)// velocity, acceleration
            );
        }

        return m_middlePath;
    }
}
