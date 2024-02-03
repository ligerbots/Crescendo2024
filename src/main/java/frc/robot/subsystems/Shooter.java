/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    static final double FEEDER_SPEED = 0.3;

    static final double KP = 1.0e-3;
    static final double KI = 0.0;
    static final double KD = 0.0;
    static final double KF = 0.0;

    CANSparkMax m_feederMotor;
    CANSparkMax m_leftShooterMotor, m_rightShooterMotor;
    SparkPIDController m_leftPidController, m_rightPidController;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    // SysId data collection
    private SysIdRoutine m_sysIdRoutine;
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

        setSysIdRoutine();

        // RPMs for testing
        SmartDashboard.putNumber("shooter/test_left_rpm", 0);
        SmartDashboard.putNumber("shooter/test_right_rpm", 0);
    }

    private void setPidController(SparkPIDController pidController) {
        // set PID coefficients
        pidController.setP(KP);
        pidController.setI(KI);
        pidController.setD(KD);
        // pidController.setIZone(kIz);
        // pidController.setFF(kFF);
        // pidController.setOutputRange(kMinOutput, kMaxOutput);
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

    public void resetShooterRpms() {
        setShooterRpms(0, 0);
    }
    public void setShooterRpms(double leftRpm, double rightRpm) {
        m_leftPidController.setReference(leftRpm, CANSparkMax.ControlType.kVelocity);
        m_rightPidController.setReference(rightRpm, CANSparkMax.ControlType.kVelocity);
    }

    public void resetFeederSpeed() {
        setFeederSpeed(0);
    }
    public void setFeederSpeed(double chute) {
        m_feederMotor.set(-chute);
    }

    public void resetShooter() {
        resetShooterRpms();
        resetFeederSpeed();
    }

    public void turnOnFeeder() {
        setFeederSpeed(FEEDER_SPEED);
    }

    public void setSysIdRoutine() {
        m_sysIdRoutine = new SysIdRoutine(new Config(), new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) -> m_leftShooterMotor.set(voltage.in(Units.Volts) / RobotController.getBatteryVoltage()),
            log -> {
                log.motor("left motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_leftShooterMotor.get() * RobotController.getBatteryVoltage(), Units.Volts))
                    .angularPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity(), Units.RPM));
            }, this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
