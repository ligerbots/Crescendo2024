package frc.robot.commands;

import java.util.ArrayList;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// Note this is a SequentialCommandGroup
public class GetStageNoteX extends GetNoteX {

    public GetStageNoteX(Translation2d targetNote, DriveTrain driveTrain, NoteVision noteVision, 
        Shooter shooter, ShooterPivot shooterPivot, Intake intake, Elevator elevator) {
        super(targetNote, driveTrain, noteVision, shooter, intake);

        if (FieldConstants.isCenterNote(targetNote)) {
            throw new IllegalArgumentException("target note param must be a stage note: S1 S2 S3");
        }

        addCommands(
            // drive back to the Note, and run Intake during the drive
            new DeferredCommand(() -> m_driveTrain.followPath(getInitialPath()), Set.of(m_driveTrain))
                .deadlineWith(new StartIntake(intake, shooter, shooterPivot, elevator)),

            // wait up to 1 more second to suck the Note in all the way
            //new WaitUntilCommand(intake::hasNote).withTimeout(2),
            new WaitCommand(0.5),
            
            // turn off Shooter and Intake
            new InstantCommand(shooter::turnOffShooter),
            new InstantCommand(intake::stop),
            // Wait for the Feeder to stop
            // new WaitUntilCommand(() -> (shooter.getFeederRpm() < Shooter.FEEDER_RPM_TOLERANCE)).withTimeout(1.0),
            new WaitCommand(0.5),

            // Spin up and shoot
            new AutoSpeakerShot(driveTrain, shooter, shooterPivot).alongWith(new InstantCommand(intake::clearHasNote))
        );
    }

    private PathPlannerPath getInitialPath() {
        Pose2d pose = m_driveTrain.getPose();
        Pose2d poseBlue = FieldConstants.flipPose(pose);
        System.out.println("Starting getInitialPath " + poseBlue);

        Pose2d closestPathStart = poseBlue.nearest(new ArrayList<>(m_candidateStartPaths.keySet()));
        System.out.println("getInitialPath nearest = " + closestPathStart);
        return m_candidateStartPaths.get(closestPathStart);
    }
}
