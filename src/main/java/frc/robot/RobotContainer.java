// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandXboxController m_mainController = new CommandXboxController(0);
    private final CommandXboxController m_controller2 = new CommandXboxController(2);
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
    private boolean m_isShootingAmp = false;

    public RobotContainer() {
        configureBindings();
        configureAutos();

        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    private void configureBindings() {
        // Intake
        m_mainController.leftBumper().whileTrue(new StartEndCommand(m_intake::intake, m_intake::stop, m_intake));
        m_mainController.rightBumper().whileTrue(new StartEndCommand(m_intake::outtake, m_intake::stop, m_intake));

        m_controller2.rightTrigger(.8).onTrue(new TestShootSpeed(m_shooter,
                () -> SmartDashboard.getNumber("shooter/test_left_rpm", 0),
                () -> SmartDashboard.getNumber("shooter/test_right_rpm", 0)));

        m_mainController.rightTrigger(.8).onTrue(new ScoreAmpOrShoot(m_shooter, m_isShootingAmp));

        m_mainController.back().onTrue(new InstantCommand(m_driveTrain::lockWheels, m_driveTrain));
        m_mainController.start().onTrue(new InstantCommand(m_driveTrain::resetHeading, m_driveTrain));
        m_mainController.a().whileTrue(new StartEndCommand(m_driveTrain::togglePrecisionMode,
                m_driveTrain::togglePrecisionMode, m_driveTrain));
        m_mainController.y()
                .onTrue(new PrepAmpShot(m_elevator, m_shooter, m_shooterPivot).alongWith(setIsShootingAmp(true)));
        m_mainController.x()
                .onTrue(new PrepareShooter(m_shooterPivot, m_shooter, m_driveTrain, m_mainController.getHID(),
                        () -> m_mainController.getLeftY(),
                        () -> m_mainController.getLeftX()));
        m_mainController.b().onTrue(new Stow(m_elevator, m_shooterPivot, m_shooter));

        // need buttons for PrepareSpeakerShot, TriggerShot and PrepareAmpShot
        
        JoystickButton farm1 = new JoystickButton(m_farm, 1);
        farm1.onTrue(new SetElevatorLength(m_elevator, Elevator.ONSTAGE_RAISE_ELEVATOR));

        JoystickButton farm2 = new JoystickButton(m_farm, 2);
        farm2.onTrue(new SetElevatorLength(m_elevator, Elevator.ONSTAGE_LOWER_ELEVATOR));

        JoystickButton farm3 = new JoystickButton(m_farm, 3);
        farm3.onTrue(new SetElevatorLength(m_elevator,
                () -> SmartDashboard.getNumber("elevator/testGoalLength", 0)));

        JoystickButton farm10 = new JoystickButton(m_farm, 10);
        farm10.onTrue(new TestShootSpeed(m_shooter,
            () -> SmartDashboard.getNumber("shooter/test_left_rpm", 0),
            () -> SmartDashboard.getNumber("shooter/test_right_rpm", 0)));
            
        JoystickButton farm11 = new JoystickButton(m_farm, 11);
        farm11.onTrue(new TestShoot(m_shooter,
            ()->SmartDashboard.getNumber("shooter/test_left_rpm", 0),
            ()->SmartDashboard.getNumber("shooter/test_right_rpm", 0)));


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
        // m_chosenAuto.setDefaultOption("GetNoteC1", new GetNoteC1(m_driveTrain, m_noteVision, m_shooter, m_intake));
        // m_chosenAuto.addOption("GetNoteC2", new GetNoteC2(m_driveTrain, m_noteVision, m_shooter, m_intake));

        m_chosenAuto.setDefaultOption("GetNoteX (C1)", new GetNoteX(FieldConstants.NOTE_C_1, m_driveTrain, m_noteVision, m_shooter, m_intake));
        m_chosenAuto.addOption("GetNoteX (C2)", new GetNoteX(FieldConstants.NOTE_C_2, m_driveTrain, m_noteVision, m_shooter, m_intake));

        m_chosenAuto.addOption("GetNoteX (S1)", new GetNoteX(FieldConstants.BLUE_NOTE_S_1, m_driveTrain, m_noteVision, m_shooter, m_intake));
        m_chosenAuto.addOption("GetNoteX (S2)", new GetNoteX(FieldConstants.BLUE_NOTE_S_2, m_driveTrain, m_noteVision, m_shooter, m_intake));
        m_chosenAuto.addOption("GetNoteX (S3)", new GetNoteX(FieldConstants.BLUE_NOTE_S_3, m_driveTrain, m_noteVision, m_shooter, m_intake));

        Translation2d[] noteList = new Translation2d[]{FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_2};
        m_chosenAuto.addOption("C1-C2", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.NOTE_C_2, FieldConstants.NOTE_C_1};
        m_chosenAuto.addOption("C2-C1", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.BLUE_NOTE_S_1, FieldConstants.BLUE_NOTE_S_2};
        m_chosenAuto.addOption("S1-S2", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2};
        m_chosenAuto.addOption("S3-S2", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.BLUE_NOTE_S_1, FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_3};
        m_chosenAuto.addOption("S1-S2-S3", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_1};
        m_chosenAuto.addOption("S3-S2-S1", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_1};
        m_chosenAuto.addOption("S2-S1", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[]{FieldConstants.BLUE_NOTE_S_1, FieldConstants.NOTE_C_1};
        m_chosenAuto.addOption("S1-C1", new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        m_chosenAuto.addOption("Test Auto", new NoteAuto(m_driveTrain));
        SmartDashboard.putData("Chosen Auto", m_chosenAuto);
    }

    public AutoCommandInterface getAutonomousCommand() {
        return m_chosenAuto.getSelected();
    }

    public Command setIsShootingAmp(boolean isShootingAmp) {
        m_isShootingAmp = isShootingAmp;
        return null;
    }

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_mainController.getLeftY()),
                () -> -modifyAxis(m_mainController.getLeftX()),
                () -> -modifyAxis(m_mainController.getRightX()));
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
