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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

        addCommands(
            new PrintCommand("starting CenterNote"),
            // Drive out to the Note
            // Turn on the Intake when we cross into the Center zone
            new DeferredCommand(this::getInitialCommand, Set.of(m_driveTrain))
                .deadlineWith(
                    // safety: dump the note, in case it did not properly shoot
                    // make it a little stronger than normal in hopes of shooting it off the path
                    new DropNote(shooter, 1500).alongWith(new InstantCommand(intake::intake))
                        // need to let it run for a little
                        .andThen(new WaitCommand(0.5))
                        .andThen(new StartIntake(intake, shooter, shooterPivot, elevator))
                ),

            new PrintCommand("CenterNote: drive back done"),
            // wait up to 0.5 second to suck the Note in all the way
            // new WaitUntilCommand(intake::hasNote).withTimeout(INTAKE_EXTRA_WAIT_TIME),
            new WaitCommand(INTAKE_EXTRA_WAIT_TIME),

            // drive to shoot position, and spin up Shooter while going (after feeder stops)
            new PrintCommand("CenterNote: starting drivein"),
            m_driveTrain.followPath(m_returnPath)
                .deadlineWith(
                    new WaitCommand(0.5)
                        .andThen(
                            // turn off Shooter and intake
                            new InstantCommand(shooter::turnOffShooter),
                            new InstantCommand(intake::stop),
                            new InstantCommand(() -> shooter.setSpeakerShootMode(true)),
                            new WaitCommand(0.5)
                                .andThen(new ActiveSetShooter(shooter, shooterPivot, () -> shooter.getShootValues(m_driveTrain))))
                ),
            // Shoot
            new TriggerShot(shooter).alongWith(new InstantCommand(intake::clearHasNote))
        );
    }

    private Command getInitialCommand() {
        Pose2d pose = m_driveTrain.getPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        System.out.println("Starting getInitialPath " + poseBlue);

        // this part is used when in center note area, if intended center note is not found
        if (poseBlue.getX() > FieldConstants.BLUE_WING_LINE_X_METERS) {
            Rotation2d heading = m_targetNote.minus(poseBlue.getTranslation()).getAngle();
            // Note final Robot heading should be "backward" since the intake is on the back
            return m_driveTrain.driveToPose(new Pose2d(m_targetNote, heading.rotateBy(Rotation2d.fromRadians(Math.PI))));

            // // heading here is the heading along the path
            // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            //         new Pose2d(poseBlue.getTranslation(), heading),
            //         new Pose2d(m_targetNote, heading));

            // // Create the path using the bezier points created above
            // // Note final Robot heading should be "backward" since the intake is on the back
            // return new PathPlannerPath(
            //         bezierPoints,
            //         new PathConstraints(DriveTrain.PATH_PLANNER_MAX_VELOCITY, DriveTrain.PATH_PLANNER_MAX_ACCELERATION,
            //                 DriveTrain.PATH_PLANNER_MAX_ANGULAR_VELOCITY,
            //                 DriveTrain.PATH_PLANNER_MAX_ANGULAR_ACCELERATION),
            //         new GoalEndState(0, heading.rotateBy(Rotation2d.fromRadians(Math.PI)), true));
        }

        Pose2d closestPathStart = poseBlue.nearest(new ArrayList<>(m_candidateStartPaths.keySet()));
        System.out.println("getInitialPath nearest = " + closestPathStart);
        return m_driveTrain.followPath(m_candidateStartPaths.get(closestPathStart));
    }
}
