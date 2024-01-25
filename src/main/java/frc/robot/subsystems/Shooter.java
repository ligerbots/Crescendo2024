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
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    static final double FEEDER_SPEED = 0.3;

    static final double KP = 0.1;
    static final double KI = 0.0;
    static final double KD = 0.0;
    static final double KF = 0.0;

    CANSparkMax m_feederMotor;
    CANSparkMax m_leftShooterMotor, m_rightShooterMotor;

    // lookup table for upper hub speeds
    static final TreeMap<Double, ShooterSpeeds> shooterSpeeds = new TreeMap<>(Map.ofEntries(
            Map.entry(0.0, new ShooterSpeeds(900.0, 900.0, FEEDER_SPEED)),      // actually lower hub, but safer to include
            Map.entry(235.0, new ShooterSpeeds(2120.0, 2300.0, FEEDER_SPEED)))); // Revere
            
    // values for lowerHub
    static final ShooterSpeeds lowHubSpeeds = new ShooterSpeeds(900.0, 900.0, 0.3);

    // Shooter class constructor, initialize arrays for motors controllers,
    // encoders, and SmartDashboard data
    public Shooter() {
        m_feederMotor = new CANSparkMax(Constants.FEEDER_CAN_ID, MotorType.kBrushless);

        m_leftShooterMotor = new CANSparkMax(Constants.LEFT_SHOOTER_CAN_ID, MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(Constants.RIGHT_SHOOTER_CAN_ID, MotorType.kBrushless);

        // // Config the Velocity closed loop gains in slot0
        // m_leftShooterMotor.config_kP(0, KP);
        // m_leftShooterMotor.config_kI(0, KI);
        // m_leftShooterMotor.config_kD(0, KD);
        // m_leftShooterMotor.config_kF(0, KF);

        // m_rightShooterMotor.config_kP(0, Constants.SHOOTER_KP);
        // m_rightShooterMotor.config_kI(0, Constants.SHOOTER_KI);
        // m_rightShooterMotor.config_kD(0, Constants.SHOOTER_KD);
        // m_rightShooterMotor.config_kF(0, Constants.SHOOTER_KF);
    }

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

    public static ShooterSpeeds calculateShooterSpeeds(double distance, boolean upperHub) {
        if (upperHub == false) {
            // if shooting to lowerHub, then return shooterSpeed with values for lowerHub
            return lowHubSpeeds;
        }

        Map.Entry<Double, ShooterSpeeds> before = shooterSpeeds.floorEntry(distance);
        Map.Entry<Double, ShooterSpeeds> after = shooterSpeeds.ceilingEntry(distance);
        if (before == null) {
            if (after == null) {
                return lowHubSpeeds; // this should never happen b/c shooterSpeeds should have at least 1 element
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
        // SmartDashboard.putNumber("shooter/bottom_rpm", getBottomRpm());
        // SmartDashboard.putNumber("shooter/top_rpm", getTopRpm());
    }

    // public double getTopRpm() {
    //     return m_leftShooterMotor.getSelectedSensorVelocity() / Constants.FALCON_UNITS_PER_RPM;
    // }

    // public double getBottomRpm() {
    //     return m_rightShooterMotor.getSelectedSensorVelocity() / Constants.FALCON_UNITS_PER_RPM;
    // }

    // public void setShooterRpms(double topRpm, double bottomRpm) {
    //     // double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0; //RPM -> Native units
    //     // double targetVelocity_UnitsPer100ms = leftYstick * 2000.0 * 2048.0 / 600.0;
    //     double falconTop = topRpm * Constants.FALCON_UNITS_PER_RPM;
    //     double falconBottom = bottomRpm * Constants.FALCON_UNITS_PER_RPM;
    //     System.out.println("setting shooter motor signals " + falconTop + " " + falconBottom);
    //     m_leftShooterMotor.set(ControlMode.Velocity, falconTop);
    //     m_rightShooterMotor.set(TalonFXControlMode.Velocity, falconBottom);
    // }

    public void setFeederSpeed(double chute) {
        m_feederMotor.set(-chute);
    }
}
