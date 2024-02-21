// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final Joystick m_farm = new Joystick(2);

    private final NoteVision m_noteVision = new NoteVision();
    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain(m_aprilTagVision, m_noteVision);
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    // Java problem: the encoder needs to be created outside the constructor
    private final ShooterPivot m_shooterPivot = new ShooterPivot(new DutyCycleEncoder(0));
    private final Elevator m_elevator = new Elevator();

    private final SendableChooser<Command> m_chosenAuto = new SendableChooser<>();
    private final SendableChooser<Pose2d> m_startLocation = new SendableChooser<>();
    private Command m_prevAutoCommand = null;
    private Pose2d m_prevInitialPose = new Pose2d();

    public RobotContainer() {
        configureBindings();
        configureAutos();

        m_driveTrain.setDefaultCommand(getDriveCommand());
        m_elevator.setDefaultCommand(getElevatorOverrideCommand());
        m_shooterPivot.setDefaultCommand(getShooterPivotOverrideCommand());
    }

    private void configureBindings() {
        // Intake
        // m_controller.leftBumper().whileTrue(new StartEndCommand(m_intake::intake, m_intake::stop, m_intake));

        // run the intake as long as the bumper is held.
        // When release, shut off the intake and back up the note a little bit
        m_driverController.leftTrigger(0.5).whileTrue(new StartIntake(m_intake, m_shooter, m_elevator, m_shooterPivot))
                .onFalse(new InstantCommand(m_intake::stop, m_intake).andThen(new BackupFeed(m_shooter)));
        m_driverController.leftBumper().whileTrue(new StartEndCommand(m_intake::outtake, m_intake::stop, m_intake));

        m_driverController.rightTrigger(0.5).onTrue(new TriggerShot(m_shooter));

        m_driverController.x().onTrue(new Stow(m_shooter, m_shooterPivot, m_elevator));

        m_driverController.a().whileTrue(new StartEndCommand(m_driveTrain::togglePrecisionMode,
                m_driveTrain::togglePrecisionMode, m_driveTrain));

        m_driverController.b().onTrue(new PrepareAmpShot(m_elevator, m_shooterPivot, m_shooter));

        m_driverController.a()
                .onTrue(new PrepareSpeakerShot(m_driveTrain, m_shooter, m_shooterPivot, m_driverController.getHID(),
                        () -> -modifyAxis(m_driverController.getLeftY()),
                        () -> -modifyAxis(m_driverController.getLeftX())));

        m_driverController.start().onTrue(new InstantCommand(m_driveTrain::lockWheels, m_driveTrain));
        m_driverController.back().onTrue(new InstantCommand(m_driveTrain::resetHeading, m_driveTrain));

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
                () -> SmartDashboard.getNumber("shooter/test_left_rpm", 0),
                () -> SmartDashboard.getNumber("shooter/test_right_rpm", 0)));

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
        // List of start locations
        m_startLocation.setDefaultOption("NotAmp Side", FieldConstants.ROBOT_START_1);
        m_startLocation.addOption("Center", FieldConstants.ROBOT_START_2);
        m_startLocation.addOption("Amp Side", FieldConstants.ROBOT_START_3);
        SmartDashboard.putData("Start Location", m_startLocation);

        // Initialize the list of available Autonomous routines
        // m_chosenAuto.setDefaultOption("GetNoteC1", new GetNoteC1(m_driveTrain,
        // m_noteVision, m_shooter, m_intake));
        // m_chosenAuto.addOption("GetNoteC2", new GetNoteC2(m_driveTrain, m_noteVision,
        // m_shooter, m_intake));

        // m_chosenAuto.setDefaultOption("GetNoteX (C1)", new
        // GetNoteX(FieldConstants.NOTE_C_1, m_driveTrain, m_noteVision, m_shooter,
        // m_intake));
        // m_chosenAuto.addOption("GetNoteX (C2)", new GetNoteX(FieldConstants.NOTE_C_2,
        // m_driveTrain, m_noteVision, m_shooter, m_intake));

        // m_chosenAuto.addOption("GetNoteX (S1)", new
        // GetNoteX(FieldConstants.BLUE_NOTE_S_1, m_driveTrain, m_noteVision, m_shooter,
        // m_intake));
        // m_chosenAuto.addOption("GetNoteX (S2)", new
        // GetNoteX(FieldConstants.BLUE_NOTE_S_2, m_driveTrain, m_noteVision, m_shooter,
        // m_intake));
        // m_chosenAuto.addOption("GetNoteX (S3)", new
        // GetNoteX(FieldConstants.BLUE_NOTE_S_3, m_driveTrain, m_noteVision, m_shooter,
        // m_intake));

        Translation2d[] noteList = new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_2 };
        m_chosenAuto.setDefaultOption("C1-C2",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.NOTE_C_2, FieldConstants.NOTE_C_1 };
        m_chosenAuto.addOption("C2-C1",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.BLUE_NOTE_S_1, FieldConstants.BLUE_NOTE_S_2 };
        m_chosenAuto.addOption("S1-S2",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2 };
        m_chosenAuto.addOption("S3-S2",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.BLUE_NOTE_S_1, FieldConstants.BLUE_NOTE_S_2,
                FieldConstants.BLUE_NOTE_S_3 };
        m_chosenAuto.addOption("S1-S2-S3",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2,
                FieldConstants.BLUE_NOTE_S_1 };
        m_chosenAuto.addOption("S3-S2-S1",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_1 };
        m_chosenAuto.addOption("S2-S1",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        noteList = new Translation2d[] { FieldConstants.BLUE_NOTE_S_1, FieldConstants.NOTE_C_1 };
        m_chosenAuto.addOption("S1-C1",
                new GetMultiNoteGeneric(noteList, m_driveTrain, m_noteVision, m_shooter, m_intake));

        m_chosenAuto.addOption("Test Auto", new NoteAuto(m_driveTrain));
        SmartDashboard.putData("Chosen Auto", m_chosenAuto);
    }

    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_startLocation.getSelected());
    }

    public Command getAutonomousCommand() {
        return m_chosenAuto.getSelected();
    }

    public boolean autoHasChanged() {
        Command autoCommand = getAutonomousCommand();
        Pose2d pose = getInitialPose();
        boolean changed = pose != m_prevInitialPose || (autoCommand != null && autoCommand != m_prevAutoCommand);
        m_prevAutoCommand = autoCommand;
        m_prevInitialPose = pose;
        return changed;
    }

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_driverController.getLeftY()),
                () -> -modifyAxis(m_driverController.getLeftX()),
                () -> -modifyAxis(m_driverController.getRightX()));
    }

    public Command getElevatorOverrideCommand() {
        return new OverrideElevator(
                m_elevator,
                () -> -modifyAxis(m_operatorController.getLeftY()));
    }

    public Command getShooterPivotOverrideCommand() {
        return new OverrideShooterPivot(
            m_shooterPivot, 
            () -> -modifyAxis(m_operatorController.getRightY())
        );
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
