// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoteAuto extends AutoCommandInterface {
    private DriveTrain m_driveTrain;
    private Pose2d m_initPose;
    /** Creates a new NoteAuto. */
    public NoteAuto(DriveTrain driveTrain) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        m_driveTrain = driveTrain;

        PathPlannerPath startPath = PathPlannerPath.fromPathFile("Note_C_1 to Shoot_1");
        m_initPose = startPath.getStartingDifferentialPose();
        
        addCommands(m_driveTrain.makePathFollowingCommand(startPath));
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}
