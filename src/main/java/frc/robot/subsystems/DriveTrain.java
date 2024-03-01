// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.swerve.*;

public class DriveTrain extends SubsystemBase {
    // The left-to-right distance between the drivetrain wheels
    // Should be measured from center to center.
    private static final double TRACKWIDTH_METERS = Units.inchesToMeters(21.75);

    // The front-to-back distance between the drivetrain wheels.
    // Should be measured from center to center.
    private static final double WHEELBASE_METERS = Units.inchesToMeters(17.75);

    public static final double ROBOT_SWERVE_OFFSET_X_INCHES = -3.0;
    private static final Translation2d ROTATION_CENTER_OFFSET = new Translation2d(Units.inchesToMeters(ROBOT_SWERVE_OFFSET_X_INCHES), 0 );

    private static final double DRIVE_BASE_RADIUS_METERS = 
            Math.sqrt(TRACKWIDTH_METERS * TRACKWIDTH_METERS + WHEELBASE_METERS * WHEELBASE_METERS) / 2.0;

    // used in lots of places, so create a local constant
    private static final double MAX_VELOCITY_METERS_PER_SECOND = FalconDriveController.MAX_VELOCITY_METERS_PER_SECOND;

    // TODO get correct values
    public static final double PATH_PLANNER_MAX_VELOCITY = 5.0;
    public static final double PATH_PLANNER_MAX_ACCELERATION = 5.0;
    public static final double PATH_PLANNER_MAX_ANGULAR_VELOCITY = 4.5;
    public static final double PATH_PLANNER_MAX_ANGULAR_ACCELERATION = 4.5;

    public static final double ANGLE_TOLERANCE_DEGREES = 5;

    // P constants for controllin during trajectory following
    private static final double X_PID_CONTROLLER_P = 3.0;
    private static final double Y_PID_CONTROLLER_P = 3.0;
    // private static final double THETA_PID_CONTROLLER_P = 4.0;

    // the max velocity and acceleration for drivetrain
    // adjusted when in precision driving mode
    private double m_maxVelocity = MAX_VELOCITY_METERS_PER_SECOND;
    private double m_maxAngularVelocity = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // if true, then robot is in field centric mode
    private boolean m_fieldCentric = true;

    // if true, then robot is in precision mode
    private boolean m_precisionMode = true;

    // store status of whether we are on goal for turning while driving
    private boolean m_onGoalForActiveTurn;

    // limit the acceleration from 0 to full power to take 1/3 second.
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(10);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(10);
    private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(10);

    private static final double MAX_VELOCITY_PRECISION_MODE = MAX_VELOCITY_METERS_PER_SECOND / 6.0;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
            MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_PRECISION_MODE = 
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 6.0;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    // NavX connected over MXP
    private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200);

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule[] m_swerveModules = new SwerveModule[4];

    // the odometry class to keep track of where the robot is on the field
    private final SwerveDrivePoseEstimator m_odometry;

    private final AprilTagVision m_aprilTagVision;

    // this is needed for the simulation
    private final NoteVision m_noteVision;

    // simulation variables
    private static final double SIM_LOOP_TIME = 0.020;
    private Pose2d m_simPose = new Pose2d();
    // remember the current chassis speeds so we can update the robot position
    private ChassisSpeeds m_simChassisSpeeds = new ChassisSpeeds();

    private final Field2d m_field = new Field2d();

    // // PID controller for swerve
    // private final PIDController m_xController = new PIDController(X_PID_CONTROLLER_P, 0, 0);
    // private final PIDController m_yController = new PIDController(Y_PID_CONTROLLER_P, 0, 0);
    // private final ProfiledPIDController m_thetaController = new ProfiledPIDController(THETA_PID_CONTROLLER_P,
    //         0, 0,
    //         new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI));

    // TODO: test if using MAX_VELOCITY_METERS_PER_SECOND of whole robot for Max
    // Module Speed per module is appropriate
    private final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(X_PID_CONTROLLER_P), new PIDConstants(Y_PID_CONTROLLER_P), MAX_VELOCITY_METERS_PER_SECOND,
            DRIVE_BASE_RADIUS_METERS, new ReplanningConfig());
            
    public DriveTrain(AprilTagVision apriltagVision, NoteVision noteVision) {
        m_swerveModules[0] = new SwerveModule("frontLeft",
                new FalconDriveController(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR),
                new NeoSteerController(Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET));

        m_swerveModules[1] = new SwerveModule("frontRight",
                new FalconDriveController(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR),
                new NeoSteerController(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET));

        m_swerveModules[2] = new SwerveModule("backLeft",
                new FalconDriveController(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR),
                new NeoSteerController(Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                        Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET));

        m_swerveModules[3] = new SwerveModule("backRight",
                new FalconDriveController(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR),
                new NeoSteerController(Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET));

        // initialize the odometry class
        // needs to be done after the Modules are created and initialized
        // TODO add in the uncertainty matrices for encoders vs vision measurements
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), getModulePositions(), new Pose2d());

        m_aprilTagVision = apriltagVision;
        m_noteVision = noteVision;

        // as late as possible, re-sync the swerve angle encoders
        for (SwerveModule module : m_swerveModules) {
            module.syncAngleEncoders(true);
        }

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("driveTrain/redFlip", 0);

    }

    // sets the heading to zero with the existing pose
    public void resetHeading() {
        Pose2d pose = getPose();
        Pose2d newPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0));
        setPose(newPose);
    }

    public Pose2d getPose() {
        if (Robot.isSimulation()) {
            return m_simPose;
        }
        return m_odometry.getEstimatedPosition();
    }

    // sets the odometry to the specified pose
    public void setPose(Pose2d pose) {
        if (Robot.isSimulation()) {
            m_simPose = pose;
            // this is called on setting a new auto. Clear the chassis speeds
            m_simChassisSpeeds = new ChassisSpeeds();
        }

        m_odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        if (Robot.isSimulation()) {
            return m_simPose.getRotation();
        }

        return m_odometry.getEstimatedPosition().getRotation();
    }

    // the gyro reading should be private.
    // Everyone else who wants the robot angle should call getHeading()
    private Rotation2d getGyroscopeRotation() {
        if (Robot.isSimulation()) {
            return m_simPose.getRotation();
        }

        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(360.0 - m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    // know the robot heading and get pitch and roll
    private Translation3d getNormalVector3d() {
        Rotation3d tilt = new Rotation3d(Units.degreesToRadians(m_navx.getRoll()),
                Units.degreesToRadians(m_navx.getPitch()), 0);
        return new Translation3d(0, 0, 1).rotateBy(tilt);
    }

    // know how much it tilted, so we know if it's balance on the ramp
    public double getTiltDegrees() {
        return Math.toDegrees(Math.acos(getNormalVector3d().getZ()));
    }

    public Rotation2d getTiltDirection() {
        return new Rotation2d(getNormalVector3d().getX(), getNormalVector3d().getY());
    }

    public void joystickDrive(double inputX, double inputY, double inputRotation) {
        // SmartDashboard.putNumber("drivetrain/joystickX", inputX);
        // SmartDashboard.putNumber("drivetrain/joystickY", inputY);
        SmartDashboard.putNumber("drivetrain/joystickR", inputRotation);

        // apply SlewLimiters to the joystick values to control acceleration
        double newInputX = m_xLimiter.calculate(inputX);
        double newInputY = m_yLimiter.calculate(inputY);
        double newInputRotation = m_rotationLimiter.calculate(inputRotation);

        // prevents a drive call with parameters of 0 0 0
        if (Math.abs(newInputX) < 0.01 && Math.abs(newInputY) < 0.01 && Math.abs(newInputRotation) < 0.01) {
            stop();
            return;
        }

        ChassisSpeeds chassisSpeeds;
        // when in field-relative mode
        if (m_fieldCentric) {
            // if we are Red, field-cenric points the other way in absolute coordinates
            // this is equivalent to flipping the X and Y joysticks
            double redFlip = FieldConstants.isRedAlliance() ? -1.0 : 1.0;
            SmartDashboard.putNumber("driveTrain/redFlip", redFlip);

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    redFlip * newInputX * m_maxVelocity,
                    redFlip * newInputY * m_maxVelocity,
                    newInputRotation * m_maxAngularVelocity,
                    getHeading());
        }
        // when in robot-centric mode
        else {
            chassisSpeeds = new ChassisSpeeds(
                    newInputX * m_maxVelocity,
                    newInputY * m_maxVelocity,
                    newInputRotation * m_maxAngularVelocity);
        }

        drive(chassisSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // // for debugging
        // SmartDashboard.putNumber("drivetrain/chassisX",
        // chassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("drivetrain/chassisY",
        // chassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("drivetrain/chassisAngle",
        // Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds, ROTATION_CENTER_OFFSET);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < 4; i++) {
            m_swerveModules[i].set(
                    states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
                    states[i].angle.getRadians());
        }

        // simulation
        m_simChassisSpeeds = m_kinematics.toChassisSpeeds(states);
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            m_swerveModules[i].stopWheel();
        }

        // zero speeds in the sim
        m_simChassisSpeeds = new ChassisSpeeds();
    }

    // for the beginning of auto rountines
    public void resetDrivingModes() {
        setFieldCentricMode(true);
        setPrecisionMode(false);
    }

    // toggle whether driving is field-centric
    public void toggleFieldCentric() {
        setFieldCentricMode(!m_fieldCentric);
    }

    // toggle precision mode for driving
    public void togglePrecisionMode() {
        setPrecisionMode(!m_precisionMode);
    }

    public void setFieldCentricMode(boolean fieldCentricMode) {
        m_fieldCentric = fieldCentricMode;
    }

    public void setPrecisionMode(boolean precisionMode) {
        m_precisionMode = precisionMode;
        m_maxVelocity = m_precisionMode ? MAX_VELOCITY_PRECISION_MODE : MAX_VELOCITY_METERS_PER_SECOND;
        m_maxAngularVelocity = m_precisionMode ? MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_PRECISION_MODE
                : MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    // lock wheels in x position to resist pushing
    public void lockWheels() {
        double lockRadians = Math.toRadians(45);
        m_swerveModules[0].set(0.0, lockRadians);
        m_swerveModules[1].set(0.0, -lockRadians);
        m_swerveModules[2].set(0.0, -lockRadians);
        m_swerveModules[3].set(0.0, lockRadians);
    }

    public Rotation2d getPitch() {
        // gets pitch of robot
        return Rotation2d.fromDegrees(m_navx.getPitch());
    }

    public Rotation2d getYaw() {
        // gets yaw of robot
        return Rotation2d.fromDegrees(m_navx.getYaw());
    }

    public Rotation2d getRoll() {
        // gets roll of robot
        return Rotation2d.fromDegrees(m_navx.getRoll());
    }

    public Field2d getField2d() {
        return m_field;
    }

    // get the swerveModuleState manually
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] state = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            state[i] = m_swerveModules[i].getSwerveModulePosition();
        }
        return state;
    }

    public void syncSwerveAngleEncoders() {
        // check if we can sync the swerve angle encoders
        // this will only trigger if the chassis is idle for 10 seconds
        for (SwerveModule module : m_swerveModules) {
            module.syncAngleEncoders(false);
        }
    }

    // return robot relative chassis speeds
    private ChassisSpeeds getChassisSpeeds() {
        return m_simChassisSpeeds;
    }

    public Command followPath(PathPlannerPath path) {

        return new FollowPathHolonomic(path, this::getPose, this::getChassisSpeeds, this::drive, PATH_FOLLOWER_CONFIG,
                () -> FieldConstants.isRedAlliance(), this);
    }

    // probably useless. This does not delay the choice of the path, since "path.get()" is called immediately
    // public Command followPath(Supplier<PathPlannerPath> path) {

    //     return new FollowPathHolonomic(path.get(), this::getPose, this::getChassisSpeeds, this::drive,
    //             PATH_FOLLOWER_CONFIG, () -> FieldConstants.isRedAlliance(), this);
    // }

    public static PathPlannerPath loadPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(String.format("Unable to load PP path %s", pathName), true);
        }
        return null;
    }

    @Override
    public void periodic() {
        m_odometry.update(getGyroscopeRotation(), getModulePositions());

        // Have the vision system update based on the Apriltags, if seen
        // need to add the pipeline result
        m_aprilTagVision.updateOdometry(m_odometry, m_field);

        // log the pose into the Field2d object
        m_field.setRobotPose(m_odometry.getEstimatedPosition());
        // also get the gyro, just in case
        SmartDashboard.putNumber("drivetrain/gyro", getGyroscopeRotation().getDegrees());

        // SmartDashboard.putNumber("drivetrain/pitch", getPitch().getDegrees());
        // SmartDashboard.putNumber("drivetrain/roll", getRoll().getDegrees());
        // SmartDashboard.putNumber("drivetrain/yaw", getYaw().getDegrees());

        SmartDashboard.putBoolean("drivetrain/precisionMode", m_precisionMode);

        for (SwerveModule mod : m_swerveModules) {
            mod.updateSmartDashboard();
        }
    }

    @Override
    public void simulationPeriodic() {
        Rotation2d head = m_simPose.getRotation();
        double newX = m_simPose.getX() + SIM_LOOP_TIME * (head.getCos() * m_simChassisSpeeds.vxMetersPerSecond
                - head.getSin() * m_simChassisSpeeds.vyMetersPerSecond);
        double newY = m_simPose.getY() + SIM_LOOP_TIME * (head.getSin() * m_simChassisSpeeds.vxMetersPerSecond
                + head.getCos() * m_simChassisSpeeds.vyMetersPerSecond);
        double newHeading = m_simPose.getRotation().getRadians()
                + m_simChassisSpeeds.omegaRadiansPerSecond * SIM_LOOP_TIME;

        // set the sim variable, and force the odometry to match
        setPose(new Pose2d(newX, newY, Rotation2d.fromRadians(newHeading)));
        m_field.setRobotPose(m_simPose);

        SmartDashboard.putNumber("drivetrain/simX", newX);
        SmartDashboard.putNumber("drivetrain/simY", newY);
        SmartDashboard.putNumber("drivetrain/simHeading", Math.toDegrees(newHeading));

        // update the Apriltag sim. Needs the robot pose
        Pose2d robotPose = getPose();
        m_aprilTagVision.updateSimulation(robotPose);
        m_noteVision.updateSimulation(robotPose);
    }

    public boolean getOnGoalForActiveTurn() {
        return m_onGoalForActiveTurn;
    }

    public void setOnGoalForActiveTurn(boolean value) {
        m_onGoalForActiveTurn = value;
    }
}