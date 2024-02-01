// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.NoteAuto;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
    private AutoCommandInterface m_autonomousCommand;
    private AutoCommandInterface m_prevAutoCommand = null;
    
    private SendableChooser<AutoCommandInterface> m_chosenAuto = new SendableChooser<>();

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        DriveTrain driveTrain = m_robotContainer.getDriveTrain();
        // Initialize the list of available Autonomous routines
        m_chosenAuto.setDefaultOption("Test Auto", new NoteAuto(driveTrain));

        SmartDashboard.putData("Chosen Auto", m_chosenAuto);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.getDriveTrain().syncSwerveAngleEncoders();

        AutoCommandInterface autoCommandInterface = m_chosenAuto.getSelected();
        if (autoCommandInterface != null && autoCommandInterface != m_prevAutoCommand) {
            m_robotContainer.getDriveTrain().setPose(autoCommandInterface.getInitialPose());
            m_prevAutoCommand = autoCommandInterface;
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_chosenAuto.getSelected();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

  @Override
  public void teleopPeriodic() {
  }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
