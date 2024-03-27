package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
            // Drive out to the Note
            // Monitor for the Note and maybe abort
            // Turn on the Intake when we cross into the Center zone
            new DeferredCommand(() -> m_driveTrain.followPath(getInitialPath()), Set.of(m_driveTrain))
                .deadlineWith(
                    // new MonitorForNote(noteVision, () -> m_driveTrain.getPose(), m_targetNote, this),
                    new StartIntake(intake, shooter, shooterPivot, elevator)
                ),

            // wait up to 0.5 second to suck the Note in all the way
            new WaitCommand(INTAKE_EXTRA_WAIT_TIME),

            // drive to shoot position, and spin up Shooter while going (after feeder stops)
            m_driveTrain.followPath(m_returnPath)
                .raceWith(
                    new WaitCommand(0.5)
                        .andThen(
                            // turn off Shooter and intake
                            new InstantCommand(shooter::turnOffShooter),
                            new InstantCommand(intake::stop),
                            new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
                            // after turning stuff off, check if we have a Note
                            // skip the rest if no Note
                            new ConditionalCommand(
                                // we have a Note, so keep going
                                new WaitCommand(0.5).andThen(new ActiveSetShooter(shooter, shooterPivot, this::getShootValues)),
                                // no Note, skip
                                Commands.none(), intake::hasNote)
                        )
                ),

            // Shoot if we have a Note
            new ConditionalCommand(new TriggerShot(shooter).alongWith(new InstantCommand(intake::clearHasNote)), Commands.none(), intake::hasNote)
        );
    }

    private PathPlannerPath getInitialPath() {
        Pose2d pose = m_driveTrain.getPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        // System.out.println("Starting getInitialPath " + poseBlue);

        // this part is used when in center note area, if intended center note is not
        // found
        if (poseBlue.getX() > FieldConstants.BLUE_WING_LINE_X_METERS) {
            Rotation2d heading = m_targetNote.minus(poseBlue.getTranslation()).getAngle();

            // heading here is the heading along the path
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    new Pose2d(poseBlue.getTranslation(), heading),
                    new Pose2d(m_targetNote, heading));

            // Create the path using the bezier points created above
            // Note final Robot heading should be "backward" since the intake is on the back
            return new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(DriveTrain.PATH_PLANNER_MAX_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ACCELERATION,
                            DriveTrain.PATH_PLANNER_MAX_ANGULAR_VELOCITY,
                            DriveTrain.PATH_PLANNER_MAX_ANGULAR_ACCELERATION),
                    new GoalEndState(0, heading.rotateBy(Rotation2d.fromRadians(Math.PI)), true));
        }

        Pose2d closestPathStart = poseBlue.nearest(new ArrayList<>(m_candidateStartPaths.keySet()));
        // System.out.println("Center: getInitialPath nearest = " + closestPathStart);
        return m_candidateStartPaths.get(closestPathStart);       
    }

    private Shooter.ShooterValues getShootValues() {
        return Shooter.calculateShooterSpeeds(m_driveTrain.getSpeakerDistance());
    }
}
