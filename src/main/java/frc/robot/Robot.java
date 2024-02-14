// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.AutoCommandInterface;

public class Robot extends TimedRobot {
    private AutoCommandInterface m_autonomousCommand = null;
    private AutoCommandInterface m_prevAutoCommand = null;
    
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Disable the LiveWindow telemetry to lower the network load
        LiveWindow.disableAllTelemetry();

        // Enable local logging.
        // ** CAREFUL: this probably should be disabled during competition.
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        m_robotContainer = new RobotContainer();
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

        AutoCommandInterface autoCommand = m_robotContainer.getAutonomousCommand();
        if (autoCommand != null && autoCommand != m_prevAutoCommand) {
            m_robotContainer.getDriveTrain().setPose(autoCommand.getInitialPose());
            m_prevAutoCommand = autoCommand;
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
            m_autonomousCommand = null;
        }

        // don't think we need this, since it is set as the default command
        // m_robotContainer.getDriveCommand().schedule();
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
