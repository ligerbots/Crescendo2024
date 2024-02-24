package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetStageNoteX extends GetNoteX {

    public GetStageNoteX(Translation2d targetNote, DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        super(targetNote, driveTrain, noteVision, shooter, intake);
        
        if(FieldConstants.isCenterNote(targetNote)) {
            throw new IllegalArgumentException("target note param must be a stage note: S1 S2 S3")
        }
    }

}
