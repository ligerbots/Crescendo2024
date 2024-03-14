// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;

import frc.robot.subsystems.DriveTrain;

public class DriveToAmp extends SequentialCommandGroup {
    private final DriveTrain m_driveTrain;

    private static final Rotation2d FINAL_HEADING = new Rotation2d(-Math.PI / 2.0);
    // Position of center of the robot. 
    // TODO: start a little far away and test
    private static final Translation2d FINAL_POSITION = FieldConstants.BLUE_AMP.minus(new Translation2d(0, Units.inchesToMeters(20.0)));

    /** Creates a new DriveToAmp. */
    public DriveToAmp(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;

        addCommands(new DeferredCommand(() -> m_driveTrain.followPath(getPath()), Set.of(m_driveTrain)));
        
        addRequirements(driveTrain);
    }

    private PathPlannerPath getPath() {
        // starting pose
        Pose2d poseBlue = FieldConstants.flipPose(m_driveTrain.getPose());
        Rotation2d pathHeading = FINAL_POSITION.minus(poseBlue.getTranslation()).getAngle();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(poseBlue.getTranslation(), pathHeading),
                new Pose2d(FINAL_POSITION, pathHeading));

        // Create the path using the bezier points created above
        return new PathPlannerPath(
                bezierPoints,
                new PathConstraints(DriveTrain.PATH_PLANNER_MAX_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ACCELERATION,
                        DriveTrain.PATH_PLANNER_MAX_ANGULAR_VELOCITY,
                        DriveTrain.PATH_PLANNER_MAX_ANGULAR_ACCELERATION),
                new GoalEndState(0, FINAL_HEADING, false)
        );
    }
}
