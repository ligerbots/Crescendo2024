// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final Joystick m_farm = new Joystick(1);

    private final NoteVision m_noteVision = new NoteVision();
    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain(m_aprilTagVision, m_noteVision);
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    // Java problem: the encoder needs to be created outside the constructor
    private final ShooterPivot m_shooterPivot = new ShooterPivot(new DutyCycleEncoder(0));
    private final Elevator m_elevator = new Elevator();

    private final SendableChooser<AutoCommandInterface> m_chosenAuto = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutos();

        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    private void configureBindings() {
        // Intake
        m_controller.rightBumper().whileTrue(new StartEndCommand(m_intake::intake, m_intake::stop, m_intake));
        m_controller.leftBumper().whileTrue(new StartEndCommand(m_intake::outtake, m_intake::stop, m_intake));

        m_controller.y().onTrue(new TestShootSpeed(m_shooter,
            () -> SmartDashboard.getNumber("shooter/test_left_rpm", 0),
            () -> SmartDashboard.getNumber("shooter/test_right_rpm", 0)));
            
        m_controller.x().onTrue(new Shoot(m_shooter,
            ()->{ return SmartDashboard.getNumber("shooter/test_left_rpm", 0); },
            ()->{ return SmartDashboard.getNumber("shooter/test_right_rpm", 0); }));

        m_controller.b().onTrue(new Stow(m_shooterPivot, m_elevator, ()->Elevator.ELEVATOR_MIN_LENGTH_METERS, ()->ShooterPivot.MAX_ANGLE));

        JoystickButton farm1 = new JoystickButton(m_farm, 1);
        farm1.onTrue(new SetElevatorLength(m_elevator, ()->Elevator.ONSTAGE_RAISE_ELEVATOR));
        
        JoystickButton farm2 = new JoystickButton(m_farm, 2);
        farm2.onTrue(new SetElevatorLength(m_elevator, ()->Elevator.ONSTAGE_LOWER_ELEVATOR));

        JoystickButton farm3 = new JoystickButton(m_farm, 3);
        farm3.onTrue(new SetElevatorLength(m_elevator, 
                () -> SmartDashboard.getNumber("elevator/testGoalLength", 0)));
        

        // -----------------------------------------------
        // commands to run the characterization for the shooter
        // JoystickButton farm1 = new JoystickButton(m_farm, 1);
        // JoystickButton farm2 = new JoystickButton(m_farm, 2);
        // JoystickButton farm3 = new JoystickButton(m_farm, 3);
        // JoystickButton farm4 = new JoystickButton(m_farm, 4);

        // farm1.onTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // farm2.onTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // farm3.onTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // farm4.onTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureAutos() {
        // Initialize the list of available Autonomous routines
        m_chosenAuto.setDefaultOption("Test Auto", new NoteAuto(m_driveTrain));
        SmartDashboard.putData("Chosen Auto", m_chosenAuto);
    }

    public AutoCommandInterface getAutonomousCommand() {
        return m_chosenAuto.getSelected();
    }

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_controller.getLeftY()),
                () -> -modifyAxis(m_controller.getLeftX()),
                () -> -modifyAxis(m_controller.getRightX()));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
    
    public NoteVision getNoteVision() {
        return m_noteVision;
    }
}
