package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.*;

public class GetCenterNoteX extends GetNoteX {

    private PathPlannerPath m_returnPath;

    private void setReturnPath(Translation2d targetNote) {
        String[] pathnameArray = s_pathLookup.get(targetNote);
        // return path is last item
        if (null != pathnameArray[pathnameArray.length - 1]) {
            m_returnPath = DriveTrain.loadPath(pathnameArray[pathnameArray.length - 1]);
        }
    }

    public GetCenterNoteX(Translation2d targetNote, DriveTrain driveTrain, NoteVision noteVision, 
            Shooter shooter, ShooterPivot shooterPivot, Intake intake, Elevator elevator) {

        super(targetNote, driveTrain, noteVision, shooter, intake);

        if (!FieldConstants.isCenterNote(targetNote)) {
            throw new IllegalArgumentException("target note param must be a center note: C1 C2 C3 C4 C5");
        }

        // return paths are for center notes only
        setReturnPath(targetNote);

        // Use note "monitoring" for center notes only
        addCommands(
            new DeferredCommand(() -> m_driveTrain.followPath(getInitialPath()), Set.of(m_driveTrain))
                .andThen(new WaitCommand(2))
                .deadlineWith(
                    new MonitorForNote(noteVision, () -> m_driveTrain.getPose(), m_targetNote, this)
                        .andThen(new StartIntake(intake, shooter, shooterPivot, elevator))
                ),
            new InstantCommand(shooter::turnOffShooter),
            new WaitCommand(1.0),
            new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
            m_driveTrain.followPath(m_returnPath)
                .deadlineWith(new ActiveSetShooter(shooter, shooterPivot, this::getShootValues)),
            new TriggerShot(shooter)
            );
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

    private Shooter.ShooterValues getShootValues() {
        return Shooter.calculateShooterSpeeds(m_driveTrain.getSpeakerDistance());
    }
}
