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
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Used for the SysId data collection
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    static final double FEEDER_SPEED = 0.3;

    // constants for side shooter, from SysId
    // Not right. There is a units problem!
    static final double K_P = 5.5e-4;//2.0766E-06;
    static final double K_I = 0.0;
    static final double K_D = 0.0;
    static final double K_FF = 0.0001575; //0.1111 / 60.0 / 12.0;

    CANSparkMax m_feederMotor;
    CANSparkMax m_leftShooterMotor, m_rightShooterMotor;
    SparkPIDController m_leftPidController, m_rightPidController;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    // SysId data collection
    private SysIdRoutine m_sysIdRoutine = null;
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Units.Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Units.Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(Units.RotationsPerSecond.of(0));

    // lookup table for upper hub speeds
    public static class ShooterSpeeds {
        public double top, bottom, chute;
        
        public ShooterSpeeds(double top, double bottom, double chute) {
            this.top = top;
            this.bottom = bottom;
            this.chute = chute;
        }

        public ShooterSpeeds interpolate(ShooterSpeeds other, double ratio) {
            return new ShooterSpeeds(
                    top + (other.top - top) * ratio,
                    bottom + (other.bottom - bottom) * ratio,
                    chute + (other.chute - chute) * ratio);
        }
    }

    static final TreeMap<Double, ShooterSpeeds> shooterSpeeds = new TreeMap<>(Map.ofEntries(
            Map.entry(0.0, new ShooterSpeeds(900.0, 900.0, FEEDER_SPEED)),      // actually lower hub, but safer to include
            Map.entry(235.0, new ShooterSpeeds(2120.0, 2300.0, FEEDER_SPEED)))); // Revere
            
    // Shooter class constructor, initialize arrays for motors controllers,
    // encoders, and SmartDashboard data
    public Shooter() {
        m_feederMotor = new CANSparkMax(Constants.FEEDER_CAN_ID, MotorType.kBrushless);

        m_leftShooterMotor = new CANSparkMax(Constants.LEFT_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(Constants.RIGHT_SHOOTER_CAN_ID, MotorType.kBrushless);

        m_leftPidController = m_leftShooterMotor.getPIDController();
        setPidController(m_leftPidController);
        m_rightPidController = m_rightShooterMotor.getPIDController();
        setPidController(m_rightPidController);
        m_leftEncoder = m_leftShooterMotor.getEncoder();
        m_rightEncoder = m_rightShooterMotor.getEncoder();

        // RPMs for testing
        SmartDashboard.putNumber("shooter/test_left_rpm", 0);
        SmartDashboard.putNumber("shooter/test_right_rpm", 0);
        SmartDashboard.putNumber("shooter/left_rpm_target", 0);
        SmartDashboard.putNumber("shooter/right_rpm_target", 0);
    }

    private void setPidController(SparkPIDController pidController) {
        // set PID coefficients
        pidController.setP(K_P);
        pidController.setI(K_I);
        pidController.setD(K_D);
        pidController.setIZone(0);
        pidController.setFF(K_FF);
        pidController.setOutputRange(-1.0, 1.0);
    }

    public static ShooterSpeeds calculateShooterSpeeds(double distance, boolean upperHub) {
        Map.Entry<Double, ShooterSpeeds> before = shooterSpeeds.floorEntry(distance);
        Map.Entry<Double, ShooterSpeeds> after = shooterSpeeds.ceilingEntry(distance);
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
        return before.getValue().interpolate(after.getValue(), ratio);
    }

    // periodically update the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/left_rpm", getLeftRpm());
        SmartDashboard.putNumber("shooter/right_rpm", getRightRpm());
    }

    public double getLeftRpm() {
        return m_leftEncoder.getVelocity();
    }

    public double getRightRpm() {
        return m_rightEncoder.getVelocity();
    }

    // set speeds -1 -> 1
    public void setShooterSpeeds(double leftSpeed, double rightSpeed) {
        m_leftShooterMotor.set(leftSpeed);
        m_rightShooterMotor.set(rightSpeed);
    }

    // set shooter RPMs, under PID control
    public void setShooterRpms(double leftRpm, double rightRpm) {
        SmartDashboard.putNumber("shooter/left_rpm_target", leftRpm);
        SmartDashboard.putNumber("shooter/right_rpm_target", rightRpm);

        m_leftPidController.setReference(leftRpm, CANSparkMax.ControlType.kVelocity);
        m_rightPidController.setReference(rightRpm, CANSparkMax.ControlType.kVelocity);
    }

    public void turnOnFeeder() {
        setFeederSpeed(FEEDER_SPEED);
    }

    public void turnOffShooter() {
        turnOffShooterWheels();
        turnOffFeeder();
    }

    public void setFeederSpeed(double chute) {
        m_feederMotor.set(-chute);
    }

    public void turnOffShooterWheels() {
        setShooterSpeeds(0, 0);
    }

    public void turnOffFeeder() {
        setFeederSpeed(0);
    }

    // Data collection for SysId
    private void setSysIdRoutine() {
        m_sysIdRoutine = new SysIdRoutine(new Config(), new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) -> m_leftShooterMotor.set(voltage.in(Units.Volts) / Constants.MAX_VOLTAGE),
            log -> {
                log.motor("left motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftShooterMotor.get() * Constants.MAX_VOLTAGE, Units.Volts))
                    .angularPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity(), Units.RPM));
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
