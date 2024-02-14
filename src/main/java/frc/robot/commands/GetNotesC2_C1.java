// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteVision;
import frc.robot.subsystems.Shooter;

public class GetNotesC2_C1 extends AutoCommandInterface {
    private PathPlannerPath m_longPath = DriveTrain.loadPath("Start_2 to Note_C_1");

    public GetNotesC2_C1(DriveTrain driveTrain, NoteVision noteVision, Shooter shooter, Intake intake) {
        addCommands(
                new GetNoteC2(driveTrain, noteVision, shooter, intake),
                new GetNoteC1(driveTrain, noteVision, shooter, intake)
        );
    }

    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_longPath.getStartingDifferentialPose());
    };
}
