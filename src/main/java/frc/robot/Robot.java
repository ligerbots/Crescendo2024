// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // set initial robot position
    Pose2d robotStartPose;
    if(DriverStation.getLocation().getAsInt() == 1)
      robotStartPose = FieldConstants.START_1;
    else if(DriverStation.getLocation().getAsInt() == 2)
      robotStartPose = FieldConstants.START_2;
    else
      robotStartPose = FieldConstants.START_3;
    
    // flip paths when red (have to do this because PathPlanner flips a path only)
    if(DriverStation.getAlliance().get().toString() == "Red")
      robotStartPose = new Pose2d(FieldConstants.FIELD_LENGTH - robotStartPose.getX(), robotStartPose.getY(), Rotation2d.fromDegrees(180.0));
      
    m_robotContainer.getDriveTrain().setPose(robotStartPose);
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // DEBUG
    List<Pose2d> notes = m_robotContainer.getNoteVision().getNotes();
    SmartDashboard.putNumber("noteVision/nFound", notes.size());
    if (notes.size() > 0) {
      SmartDashboard.putNumber("noteVision/x", notes.get(0).getX());
      SmartDashboard.putNumber("noteVision/x", notes.get(0).getX());
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
