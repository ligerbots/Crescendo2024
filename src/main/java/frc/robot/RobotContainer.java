// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
        private final CommandXboxController m_driverController = new CommandXboxController(0);
        // private final CommandXboxController m_operatorController = new
        // CommandXboxController(1);
        private final Joystick m_farm = new Joystick(1);

        private final NoteVision m_noteVision = new NoteVision();
        private final AprilTagVision m_aprilTagVision = new AprilTagVision();
        private final DriveTrain m_driveTrain = new DriveTrain(m_aprilTagVision, m_noteVision);
        private final Intake m_intake = new Intake();
        private final Shooter m_shooter = new Shooter();
        private final ShooterPivot m_shooterPivot = new ShooterPivot();
        private final Elevator m_elevator = new Elevator();

        private final Climber m_climber = new Climber(m_driveTrain);

        private final SendableChooser<Command> m_chosenAuto = new SendableChooser<>();
        private final SendableChooser<Pose2d> m_startLocation = new SendableChooser<>();
        private Command m_prevAutoCommand = null;
        private Pose2d m_prevInitialPose = new Pose2d();

        public RobotContainer() {
                configureBindings();
                configureAutos();

                m_driveTrain.setDefaultCommand(getDriveCommand());
        }

    private void configureBindings() {
        // run the intake as long as the bumper is held.
        // When release, shut off the intake and feeder
        m_driverController.leftTrigger()
                .onTrue(new RumbleOnIntake(m_intake, m_driverController.getHID())
                        .alongWith(new StartIntake(m_intake, m_shooter, m_shooterPivot, m_elevator)))
                .onFalse(new InstantCommand(m_intake::stop, m_intake)
                        .alongWith(new InstantCommand(m_shooter::turnOffShooter, m_shooter)));

        // m_driverController.leftBumper().whileTrue(new StartEndCommand(m_intake::outtake, m_intake::stop, m_intake));

        m_driverController.leftBumper().whileTrue(new InstantCommand(m_intake::intake).alongWith(new InstantCommand(()-> m_shooter.setShooterSpeeds(3000,3000))));

        m_driverController.rightTrigger().onTrue(
            new TriggerShot(m_shooter).alongWith(new InstantCommand(m_intake::clearHasNote))
            .andThen(new Stow(m_shooter, m_shooterPivot, m_elevator)).onlyIf(() -> !m_driverController.leftBumper().getAsBoolean())
            .alongWith(new InstantCommand(() -> m_driveTrain.getDefaultCommand().schedule()))
        );
        
        m_driverController.y().onTrue(new Stow(m_shooter, m_shooterPivot, m_elevator));

        // don't require the Drivetrain. Otherwise you cannot drive.
        m_driverController.b().whileTrue(new StartEndCommand(() -> m_driveTrain.setPrecisionMode(true),
                () -> m_driveTrain.setPrecisionMode(false)));

        m_driverController.a().onTrue(new PrepareAmpShot(m_driveTrain, m_elevator, m_shooterPivot, m_shooter));
                // .onlyIf(() -> m_driveTrain.getAmpDistance() < DriveTrain.AMP_DRIVE_MAX_METERS));

        m_driverController.x()
                .onTrue(new PrepareSpeakerShot(m_driveTrain, m_shooter, m_shooterPivot, m_driverController.getHID(),
                        () -> -modifyAxis(m_driverController.getLeftY()),
                        () -> -modifyAxis(m_driverController.getLeftX()), 
                        () -> -modifyAxis(m_driverController.getRightX())));
        // Bind the header control separately from the other parts of PrepSpeakerShot
        // This allows us to kill the heading command without killing the rest of it.
        // TODO: if this works, remove joysticks from above
        m_driverController.x().onTrue(new ActiveTurnToHeadingWithDriving(m_driveTrain, m_driveTrain::headingToSpeaker,
                        () -> -modifyAxis(m_driverController.getLeftY()),
                        () -> -modifyAxis(m_driverController.getLeftX()),
                        () -> -modifyAxis(m_driverController.getRightX())));
                        
        m_driverController.start().onTrue(new InstantCommand(m_driveTrain::lockWheels, m_driveTrain));
        m_driverController.back().onTrue(new InstantCommand(m_driveTrain::resetHeading, m_driveTrain));

        // Climber Commands
        JoystickButton farm1 = new JoystickButton(m_farm, 1);
        farm1.onTrue(new InstantCommand(m_climber::extendHooks, m_climber));

        JoystickButton farm2 = new JoystickButton(m_farm, 2);
        farm2.onTrue(new InstantCommand(m_climber::retractHooks, m_climber)
                        .alongWith(new InstantCommand(() -> m_shooterPivot.setAngle(ShooterPivot.CLIMB_ANGLE_RADIANS, false))));

        JoystickButton farm3 = new JoystickButton(m_farm, 3);
        farm3.onTrue(new InstantCommand(m_climber::holdHooks, m_climber));

        JoystickButton farm6 = new JoystickButton(m_farm, 6);
        farm6.onTrue(new InstantCommand(() -> m_climber.run(Climber.WINCH_MANUAL_SPEED, 0), m_climber))
                .onFalse(new InstantCommand(m_climber::holdHooks, m_climber));

        JoystickButton farm7 = new JoystickButton(m_farm, 7);
        farm7.onTrue(new InstantCommand(() -> m_climber.run(Climber.WINCH_MANUAL_SPEED, Climber.WINCH_MANUAL_SPEED), m_climber))
                .onFalse(new InstantCommand(m_climber::holdHooks, m_climber));

        JoystickButton farm8 = new JoystickButton(m_farm, 8);
        farm8.onTrue(new InstantCommand(() -> m_climber.run(0, Climber.WINCH_MANUAL_SPEED), m_climber))
                .onFalse(new InstantCommand(m_climber::holdHooks, m_climber));

        JoystickButton farm11 = new JoystickButton(m_farm, 11);
        farm11.onTrue(new DropNote(m_shooter)).onFalse(new Stow(m_shooter, m_shooterPivot, m_elevator));

        // Elevator adjust up/down
        JoystickButton farm4 = new JoystickButton(m_farm, 4);
        farm4.onTrue(new InstantCommand(() -> m_elevator.adjustLength(true)));
        JoystickButton farm9 = new JoystickButton(m_farm, 9);
        farm9.onTrue(new InstantCommand(() -> m_elevator.adjustLength(false)));

        // Pivot adjust up/down
        JoystickButton farm5 = new JoystickButton(m_farm, 5);
        farm5.onTrue(new InstantCommand(() -> m_shooterPivot.adjustAngle(true)));
        JoystickButton farm10 = new JoystickButton(m_farm, 10);
        farm10.onTrue(new InstantCommand(() -> m_shooterPivot.adjustAngle(false)));

        // Adjust shoot heading angle, using DPad
        POVButton dpadLeft = new POVButton(m_driverController.getHID(), 270);
        dpadLeft.onTrue(new InstantCommand(() -> m_driveTrain.adjustHeading(false)));
        POVButton dpadRight = new POVButton(m_driverController.getHID(), 90);
        dpadRight.onTrue(new InstantCommand(() -> m_driveTrain.adjustHeading(true)));

        // schedule Drive command, which will cancel other control of Drivetrain, ie active heading
        JoystickButton farm12 = new JoystickButton(m_farm, 12);
        farm12.onTrue(new InstantCommand(() -> m_driveTrain.getDefaultCommand().schedule()));

        // reset zero of elevator
        JoystickButton farm15 = new JoystickButton(m_farm, 15);
        farm15.onTrue(new InstantCommand(m_elevator::zeroElevator));

        // fix camera mode
        JoystickButton farm16 = new JoystickButton(m_farm, 16);
        farm16.onTrue(new CameraMode(m_noteVision, m_aprilTagVision));

        // Test commands

        JoystickButton farm22 = new JoystickButton(m_farm, 22);
        farm22.onTrue(new SetElevatorLength(m_elevator,
                () -> Units.inchesToMeters(SmartDashboard.getNumber("elevator/testLength", 0)), false).withTimeout(5.0));

        JoystickButton farm23 = new JoystickButton(m_farm, 23);
        farm23.onTrue(new SetPivotAngle(m_shooterPivot,
                () -> Math.toRadians(SmartDashboard.getNumber("shooterPivot/testAngle", 0)), false).withTimeout(5.0));

        JoystickButton farm24 = new JoystickButton(m_farm, 24);
        farm24.onTrue(new TestShoot(m_driveTrain, m_shooter,
                () -> SmartDashboard.getNumber("shooter/testLeftRpm", 0),
                () -> SmartDashboard.getNumber("shooter/testRightRpm", 0)));

        // JoystickButton farm24 = new JoystickButton(m_farm, 24);
        // farm24.whileTrue(new ActiveTurnToHeadingWithDriving(m_driveTrain, m_driveTrain::headingToSpeaker,
        //                 () -> -modifyAxis(m_driverController.getLeftY()),
        //                 () -> -modifyAxis(m_driverController.getLeftX()),
        //                 () -> -modifyAxis(m_driverController.getRightX())));
        
        // JoystickButton farm15 = new JoystickButton(m_farm, 15);
        // farm15.onTrue(new TestShootSpeed(m_shooter,
        //         () -> SmartDashboard.getNumber("shooter/testLeftRpm", 0),
        //         () -> SmartDashboard.getNumber("shooter/testRightRpm", 0)));

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

                // autoName = "S2-S3-C5";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_3,
                // FieldConstants.NOTE_C_5 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));
                // autoName = "S2-S3-C4";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_3,
                // FieldConstants.NOTE_C_4 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                String autoName = "C1-C2";
                m_chosenAuto.setDefaultOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_2 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C1-C2-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_2,
                                                FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C1-C2-C1";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_2,
                                                FieldConstants.NOTE_C_1 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C2-C1";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_2, FieldConstants.NOTE_C_1 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C2-C1-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_2, FieldConstants.NOTE_C_1,
                                                FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C1-C3-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_3,
                                                FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C1-C3-C2";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_3,
                                                FieldConstants.NOTE_C_2 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C1-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C2-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_2, FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C3-C2";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_3, FieldConstants.NOTE_C_2 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C1-C2-S1";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_2,
                                                FieldConstants.BLUE_NOTE_S_1 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C4-C5";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_4, FieldConstants.NOTE_C_5 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C4-C5-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_4, FieldConstants.NOTE_C_5,
                                                FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C5-C4";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_5, FieldConstants.NOTE_C_4 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "C5-C4-C3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.NOTE_C_5, FieldConstants.NOTE_C_4,
                                                FieldConstants.NOTE_C_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "S3-C5-C4";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.BLUE_NOTE_S_3, FieldConstants.NOTE_C_5,
                                                FieldConstants.NOTE_C_4 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "S3-S2-S1";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2,
                                                FieldConstants.BLUE_NOTE_S_1 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "S3-S2-S1-C1";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2,
                                                FieldConstants.BLUE_NOTE_S_1, FieldConstants.NOTE_C_1 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "S1-S2-S3";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.BLUE_NOTE_S_1, FieldConstants.BLUE_NOTE_S_2,
                                                FieldConstants.BLUE_NOTE_S_3 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                autoName = "S1-S2-S3-C5";
                m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                                new Translation2d[] { FieldConstants.BLUE_NOTE_S_1, FieldConstants.BLUE_NOTE_S_2,
                                                FieldConstants.BLUE_NOTE_S_3, FieldConstants.NOTE_C_5 },
                                m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake, m_elevator));

                // autoName = "S1";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_1 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // autoName = "S2";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_2 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // autoName = "S3";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_3 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // autoName = "S3-S2";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_3, FieldConstants.BLUE_NOTE_S_2 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // autoName = "S2-S3";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(new Translation2d[]
                // { FieldConstants.BLUE_NOTE_S_2, FieldConstants.BLUE_NOTE_S_3 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // autoName = "C1-C3-C2";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                // new Translation2d[] { FieldConstants.NOTE_C_1, FieldConstants.NOTE_C_3,
                // FieldConstants.NOTE_C_2 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // autoName = "C3";
                // m_chosenAuto.addOption(autoName, new GetMultiNoteGeneric(
                // new Translation2d[] { FieldConstants.NOTE_C_3 },
                // m_driveTrain, m_noteVision, m_shooter, m_shooterPivot, m_intake,
                // m_elevator));

                // List<String> autonamesDropdown = Arrays.asList("S1-S2", "S3-S2", "S1-S2-S3",
                // "S3-S2-S1", "S2-S1", "S1-C1", "C4", "C5", "S3-C4-C5" );

                // for (String autoNm : autonamesDropdown) {
                // m_chosenAuto.addOption(autoNm, new GetMultiNoteGeneric(autoNm, m_driveTrain,
                // m_noteVision, m_shooter, m_intake));
                // }

                // m_chosenAuto.addOption("Test Auto", new NoteAuto(m_driveTrain));
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
                // warning do not compare poses with "==". That compares object IDs, not value.
                boolean changed = !pose.equals(m_prevInitialPose)
                                || (m_prevAutoCommand != null && autoCommand != m_prevAutoCommand);
                m_prevAutoCommand = autoCommand;
                m_prevInitialPose = pose;
                return changed;
        }

        public Command getDriveCommand() {
                // The controls are for field-oriented driving:
                // Left stick Y axis -> forward and backwards movement
                // Left stick X axis -> left and right movement
                // Right stick X axis -> rotation
                // note: "rightBumper()"" is a Trigger which is a BooleanSupplier
                return new Drive(
                                m_driveTrain,
                                () -> -modifyAxis(m_driverController.getLeftY()),
                                () -> -modifyAxis(m_driverController.getLeftX()),
                                () -> -modifyAxis(m_driverController.getRightX()),
                                m_driverController.rightBumper());
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

        public ShooterPivot getShooterPivot() {
                return m_shooterPivot;
        }

        public Intake getIntake() {
                return m_intake;
        }
}
