// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetNoteC1 extends AutoCommandInterface {
  /** Creates a new GetNoteC1. */

  private PathPlannerPath m_longPath = PathPlannerPath.fromPathFile("Start_2 to Note_C_1");
  private PathPlannerPath m_middlePath = PathPlannerPath.fromPathFile("Start_2 to Note_C_1");
  private PathPlannerPath m_returnPath = PathPlannerPath.fromPathFile("Note_C_1 to Shoot_1");
  private DriveTrain m_driveTrain;

  


  public GetNoteC1(DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
    m_driveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        driveTrain.FollowPath(() -> getInitialPath()).alongWith(new MonitorForNote(()-> m_driveTrain.getPose(), FieldConstants.NOTE_C_1, noteVision, this)),

        new InstantCommand(intake::intake),
        driveTrain.makePathFollowingCommand(m_returnPath)
    // .alongWith(new prepShooter())j

    );
  }
 
  public Pose2d getInitialPose(){
    return FieldConstants.flipPose(m_longPath.getStartingDifferentialPose()); 
  };

  private PathPlannerPath getInitialPath() {
    Pose2d pose = m_driveTrain.getPose();
    Pose2d poseBlue = FieldConstants.flipPose(pose);
    if (poseBlue.getX() < FieldConstants.BLUE_WHITE_LINE_X_METERS) {
      return m_longPath;

    }
    if (poseBlue.getX() > FieldConstants.BLUE_WHITE_LINE_X_METERS) {
      Rotation2d heading = FieldConstants.NOTE_C_1.getTranslation().minus(poseBlue.getTranslation()).getAngle();
      List<PathPoint> pathPoints = List.of(new PathPoint(poseBlue.getTranslation()), // starting pose
          new PathPoint(FieldConstants.NOTE_C_1.getTranslation()));
      return PathPlannerPath.fromPathPoints(
          pathPoints, // position, heading
          new PathConstraints(DriveTrain.PATH_PLANNER_MAX_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ACCELERATION,
              DriveTrain.PATH_PLANNER_MAX_ANGULAR_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ANGULAR_ACCELERATION),
          new GoalEndState(0, heading, true)// velocity, acceleration

      );

    }
    return m_middlePath;

  }
  
}
