/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Map;
import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Used for the SysId data collection
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.Constants;
import frc.robot.FieldConstants;

public class Shooter extends SubsystemBase {
    
    static final double FEEDER_SPEED = 0.5;

    // AMP shot, backwards out input end
    static final double AMP_SHOOT_SPEED = -0.8;
    
    // This is negative to push the note back slowly
    public static final double BACKUP_FEED_SPEED = -0.25;
    public static final double BACKUP_SHOOTER_SPEED = -0.2;

    public static final double BACKUP_FEED_TIME = 0.5;  // seconds

    public static final double RPM_TOLERANCE = 200;
    public static final double FEEDER_RPM_TOLERANCE = 100; 

    // manually tuned kFF and guessed kP
    static final double K_P_LEFT = 1e-4;
    static final double K_P_RIGHT = K_P_LEFT;
    static final double K_I = 0.0;
    static final double K_D = 0.0;
    static final double K_FF_LEFT = 0.00024;
    static final double K_FF_RIGHT = 0.00022;

    CANSparkMax m_feederMotor;
    RelativeEncoder m_feederMotorEncoder;
    SparkPIDController m_feederPidController;

    CANSparkMax m_leftShooterMotor, m_rightShooterMotor;
    SparkPIDController m_leftPidController, m_rightPidController;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    // SysId data collection
    private SysIdRoutine m_sysIdRoutine = null;
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(edu.wpi.first.units.Units.Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(edu.wpi.first.units.Units.Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(edu.wpi.first.units.Units.RotationsPerSecond.of(0));

    // Used for is on target
    private double m_leftGoalRPM;
    private double m_rightGoalRPM;

    private boolean m_speakerShootMode = true;
    
    // lookup table for upper hub speeds
    public static class ShooterValues {
        public double leftRPM, rightRPM, shootAngle;
        
        public ShooterValues(double leftRPM, double rightRPM, double shootAngle) {
            this.leftRPM = leftRPM;
            this.rightRPM = rightRPM;
            this.shootAngle = shootAngle;
        }

        public ShooterValues interpolate(ShooterValues other, double ratio) {
            return new ShooterValues(
                    leftRPM + (other.leftRPM - leftRPM) * ratio,
                    rightRPM + (other.rightRPM - rightRPM) * ratio,
                    shootAngle + (other.shootAngle - shootAngle) * ratio);
        }
    }

    static final TreeMap<Double, ShooterValues> shooterSpeeds = new TreeMap<>(Map.ofEntries(
            Map.entry(Units.inchesToMeters(52), new ShooterValues(3000.0, 3000.0, Math.toRadians(55.0))),
            Map.entry(Units.inchesToMeters(104.3), new ShooterValues(3000.0, 3000.0, Math.toRadians(37.0))),
            Map.entry(Units.inchesToMeters(137.4), new ShooterValues(3100.0, 3100.0, Math.toRadians(30.5))),
            Map.entry(Units.inchesToMeters(146.5), new ShooterValues(3400.0, 3300.0, Math.toRadians(29.5))),
            Map.entry(Units.inchesToMeters(168.0), new ShooterValues(3850.0, 3700.0, Math.toRadians(26.0))),
            Map.entry(Units.inchesToMeters(224.0), new ShooterValues(4600.0, 4500.0, Math.toRadians(22.25))),
            Map.entry(5.8, new ShooterValues(4600.0, 4500.0, Math.toRadians(22.25))),

            // Pass shots from Center zone
            // close to Stage
            Map.entry(6.0, new ShooterValues(2000.0, 2000.0, Math.toRadians(50))),
            // more in center area
            Map.entry(FieldConstants.FIELD_LENGTH/2, new ShooterValues(2500.0, 2500.0, Math.toRadians(40)))
            ));

    // Shooter class constructor, initialize arrays for motors controllers,
    // encoders, and SmartDashboard data
    public Shooter() {
        m_feederMotor = new CANSparkMax(Constants.FEEDER_CAN_ID, MotorType.kBrushless);
        m_feederMotor.restoreFactoryDefaults();
        m_feederMotor.setInverted(true);
        m_feederMotor.setIdleMode(IdleMode.kBrake);
        m_feederMotor.setSmartCurrentLimit(30);
        m_feederMotorEncoder = m_feederMotor.getEncoder();
        
        m_feederPidController = m_feederMotor.getPIDController();
        setPidController(m_feederPidController, 1e-4, 0);

        m_leftShooterMotor = new CANSparkMax(Constants.LEFT_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_leftShooterMotor.restoreFactoryDefaults();
        m_leftShooterMotor.setInverted(true);

        m_leftPidController = m_leftShooterMotor.getPIDController();
        setPidController(m_leftPidController, K_P_LEFT, K_FF_LEFT);
        m_leftEncoder = m_leftShooterMotor.getEncoder();

        m_rightShooterMotor = new CANSparkMax(Constants.RIGHT_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_rightShooterMotor.restoreFactoryDefaults();

        m_rightPidController = m_rightShooterMotor.getPIDController();
        setPidController(m_rightPidController, K_P_RIGHT, K_FF_RIGHT);
        m_rightEncoder = m_rightShooterMotor.getEncoder();

        // RPMs for testing
        SmartDashboard.putNumber("shooter/testLeftRpm", 0);
        SmartDashboard.putNumber("shooter/testRightRpm", 0);
        SmartDashboard.putNumber("shooter/leftRpmTarget", 0);
        SmartDashboard.putNumber("shooter/rightRpmTarget", 0);

        SmartDashboard.putNumber("shooter/shotDistanceInches", 0);
    }

    private void setPidController(SparkPIDController pidController, double kP, double kFF) {
        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(K_I);
        pidController.setD(K_D);
        pidController.setIZone(0);
        pidController.setFF(kFF);
        pidController.setOutputRange(-1.0, 1.0);
    }

    public static ShooterValues calculateShooterSpeeds(double distance) {
        Map.Entry<Double, ShooterValues> before = shooterSpeeds.floorEntry(distance);
        Map.Entry<Double, ShooterValues> after = shooterSpeeds.ceilingEntry(distance);
        if (before == null) {
            if (after == null) {
                return null; // this should never happen b/c shooterSpeeds should have at least 1 element
            }
            return after.getValue();
        }
        if (after == null)
            return before.getValue();
            
        double denom = after.getKey() - before.getKey();
        if (Math.abs(denom) < 0.1) {
            // distance must have exactly matched a key
            return before.getValue();
        }

        double ratio = (distance - before.getKey()) / denom;
        ShooterValues res = before.getValue().interpolate(after.getValue(), ratio);

        // for tuning. Leave in for diagnostics??
        SmartDashboard.putNumber("shooter/shotDistanceInches", Units.metersToInches(distance));
        SmartDashboard.putNumber("shooter/shotLeftRPM", res.leftRPM);
        SmartDashboard.putNumber("shooter/shotRightRPM", res.rightRPM);
        SmartDashboard.putNumber("shooter/shotAngle", Math.toDegrees(res.shootAngle));

        return res;
    }

    // periodically update the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/leftRpm", getLeftRpm());
        SmartDashboard.putNumber("shooter/rightRpm", getRightRpm());
        SmartDashboard.putNumber("shooter/feederRpm", getFeederRpm());

        SmartDashboard.putNumber("shooter/leftCurrent", m_leftShooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("shooter/rightCurrent", m_rightShooterMotor.getOutputCurrent());
        SmartDashboard.putNumber("shooter/feederSpeed", m_feederMotor.get());
        SmartDashboard.putNumber("shooter/feederCurrent", m_feederMotor.getOutputCurrent());
    }

    public double getLeftRpm() {
        return m_leftEncoder.getVelocity();
    }

    public double getRightRpm() {
        return m_rightEncoder.getVelocity();
    }

    public double getFeederRpm() {
        return m_feederMotorEncoder.getVelocity();
    }

    public double getFeederRotations() {
        return m_feederMotorEncoder.getPosition();
    }

    // set speeds -1 -> 1
    public void setShooterSpeeds(double leftSpeed, double rightSpeed) {
        m_leftShooterMotor.set(leftSpeed);
        m_rightShooterMotor.set(rightSpeed);
    }

    // set shooter RPMs, under PID control
    public void setShooterRpms(double leftRpm, double rightRpm) {
        SmartDashboard.putNumber("shooter/leftRpmTarget", leftRpm);
        SmartDashboard.putNumber("shooter/righRpmTarget", rightRpm);

        //Used in isWithinTolerenceFunc
        m_leftGoalRPM = leftRpm;
        m_rightGoalRPM = rightRpm;

        m_leftPidController.setReference(leftRpm, CANSparkMax.ControlType.kVelocity);
        m_rightPidController.setReference(rightRpm, CANSparkMax.ControlType.kVelocity);
    }

    public boolean rpmWithinTolerance() {
        return m_leftGoalRPM > 1000.0
                && Math.abs(m_leftGoalRPM - getLeftRpm()) < RPM_TOLERANCE
                && Math.abs(m_rightGoalRPM - getRightRpm()) < RPM_TOLERANCE;
    }

    public void startForIntake() {
        setShooterSpeeds(BACKUP_SHOOTER_SPEED, BACKUP_SHOOTER_SPEED);
        setFeederSpeed(FEEDER_SPEED);
    }

    public void speakerShot() {
        setFeederSpeed(FEEDER_SPEED);
    }

    public void ampShot() {
        setFeederSpeed(AMP_SHOOT_SPEED);
    }

    public void turnOffShooter() {
        turnOffShooterWheels();
        turnOffFeeder();
    }

    public void setFeederSpeed(double speed) {
        m_feederMotor.set(-speed);
    }

    public void turnOffShooterWheels() {
        SmartDashboard.putNumber("shooter/leftRpmTarget", 0);
        SmartDashboard.putNumber("shooter/righRpmTarget", 0);

        setShooterSpeeds(0, 0);
    }

    public void turnOffFeeder() {
        // try using PID to get the feeder stopped as quickly as possible
        m_feederPidController.setReference(0, CANSparkMax.ControlType.kVelocity);
        // setFeederSpeed(0);
    }

    public void setSpeakerShootMode(boolean mode) {
        m_speakerShootMode = mode;
    }

    public boolean getSpeakerShootMode() {
        return m_speakerShootMode;
    }

    // Data collection for SysId
    private void setSysIdRoutine() {
        m_sysIdRoutine = new SysIdRoutine(new Config(), new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) -> m_leftShooterMotor.set(voltage.in(edu.wpi.first.units.Units.Volts) / Constants.MAX_VOLTAGE),
            log -> {
                log.motor("left motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftShooterMotor.get() * Constants.MAX_VOLTAGE, edu.wpi.first.units.Units.Volts))
                    .angularPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), edu.wpi.first.units.Units.Rotations))
                    .angularVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity(), edu.wpi.first.units.Units.RPM));
            }, this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        if (m_sysIdRoutine == null)
            setSysIdRoutine();

        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if (m_sysIdRoutine == null)
            setSysIdRoutine();

        return m_sysIdRoutine.dynamic(direction);
    }
}
