package frc.robot.commands;

import java.util.ArrayList;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetStageNoteX extends GetNoteX {

    public GetStageNoteX(Translation2d targetNote, DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        super(targetNote, driveTrain, noteVision, shooter, intake);

        if (FieldConstants.isCenterNote(targetNote)) {
            throw new IllegalArgumentException("target note param must be a stage note: S1 S2 S3");
        }

        addCommands(new DeferredCommand(() -> m_driveTrain.followPath(getInitialPath()), Set.of(m_driveTrain)));

        addCommands(new PrintCommand("GetStageNoteX-- Finished target note: " + targetNote));
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
