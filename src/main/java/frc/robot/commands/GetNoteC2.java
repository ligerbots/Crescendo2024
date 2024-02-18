// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

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

// Note that AutoCommandInterface is a SequentialCommandGroup
public class GetNoteC2 extends AutoCommandInterface {
    /** Creates a new GetNoteC2. */

    List<String> m_pathNames = new ArrayList<String>() {
        {
            add("Start_2 to Note_C_2");
            add("Shoot_1 to Note_C_2");
            add("Note_C_2 to Shoot_1");
        }
    };

    List<PathPlannerPath> m_pathList = new ArrayList<PathPlannerPath>() {
        {
            m_pathNames.forEach((n) -> add(DriveTrain.loadPath(n)));
        }
    };

    private PathPlannerPath m_longPath  =  m_pathList.get(0);
    private PathPlannerPath m_middlePath = m_pathList.get(1);
    private PathPlannerPath m_returnPath = m_pathList.get(2);
    
    private DriveTrain m_driveTrain;
    private final Translation2d m_targetNote = FieldConstants.NOTE_C_2;

    public GetNoteC2(DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        m_driveTrain = driveTrain;

        addCommands(
                m_driveTrain.followPath(() -> getInitialPath())
                        .alongWith(new MonitorForNote(noteVision, () -> m_driveTrain.getPose(), m_targetNote, this)),
                m_driveTrain.followPath(m_returnPath),

                new InstantCommand(intake::intake)
        // .alongWith(new prepShooter())

        );
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

        return m_middlePath;
    }
}
