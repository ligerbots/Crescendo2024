// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants;

public class ShooterPivot extends TrapezoidProfileSubsystem {
    // NOTE: the 0 angle should be with the shooter horizontal
    // The offset for the absolute encoder should be taken care off
    //   in the SparkMax settings (firmware).

    private static final double TWO_PI = 2.0 * Math.PI;

    private static final double MIN_ANGLE = Math.toRadians(0); //TODO set for 2024
    private static final double MAX_ANGLE = Math.toRadians(60.0); //TODO set for 2024

    private static final double ANGLE_TOLERANCE_RADIAN = Math.toRadians(2.0); //TODO: set for 2024

    private static final int CURRENT_LIMIT = 10;

    // position constants for commands
    public static final double STOW_ANGLE_RADIANS = Math.toRadians(58.0);
    public static final double AMP_SCORE_ANGLE_RADIANS = Math.toRadians(50);

    // All units are MKS with angles in Radians
    // Constants to limit the shooterPivot rotation speed
    private static final double MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(40); //TODO: set for 2024
    private static final double MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(40); //TODO: set for 2024

    // Angle Offset should be taken care off in the SparkMax
    // private static final double OFFSET_ROTATION = 0.0/360.0; //TODO: set for 2024

    // Constants for the shooterPivot PID controller
    private static final double K_P = 0.01;  // TODO: set for 2024
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;  // TODO: set for 2024

    private final RelativeEncoder m_absoluteEncoder;
    private final CANSparkMax m_motor;
    private final SparkPIDController m_pidController;

    // Used for checking if on goal
    private double m_goalRadians;

    // Construct a new shooterPivot subsystem
    public ShooterPivot(DutyCycleEncoder absEncoder) {
        super(new TrapezoidProfile.Constraints(MAX_VEL_RADIAN_PER_SEC, MAX_ACC_RADIAN_PER_SEC_SQ));
       
        // NOTE: currently using a mini CIM on the pivot. This is a BRUSHED motor.
        m_motor = new CANSparkMax(Constants.SHOOTER_PIVOT_CAN_ID, CANSparkMax.MotorType.kBrushed);
        // don't reset to factory; might wipe important settings
        // m_motor.restoreFactoryDefaults();
       
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        // Rev through bore
        m_absoluteEncoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        // offset
        //m_absoluteEncoder.setPosition(m_absoluteEncoder.getPosition() - OFFSET_ROTATION);

        m_pidController = m_motor.getPIDController();
        m_pidController.setFeedbackDevice(m_absoluteEncoder);

        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);
        m_pidController.setFF(K_FF);
        m_pidController.setOutputRange(-1.0, 1.0);

        SmartDashboard.putNumber("shooterPivot/testAngle", 0);

        SmartDashboard.putBoolean("shooterPivot/coastMode", false);
        setCoastMode();
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("shooterPivot/encoder", Math.toDegrees(getAngleRadians()));
        SmartDashboard.putNumber("shooterPivot/current", m_motor.getOutputCurrent());

        setCoastMode();

        // Execute the super class periodic method
        super.periodic();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Remember - encoder is in rotations

        m_pidController.setReference(setPoint.position / TWO_PI, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("shooterPivot/setPoint", Math.toDegrees(setPoint.position));
    }

    // get the current pivot angle in radians
    public double getAngleRadians() {
        double angle = TWO_PI * m_absoluteEncoder.getPosition();
        // we want the range to be -PI -> PI. Makes the control around 0 easier.
        if (angle > Math.PI) angle -= TWO_PI;
        return angle;
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitPivotAngle(double angle) {
        return MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    }

    // set shooterPivot, angle in radians
    public void setAngle(double angle) {
        m_goalRadians = limitPivotAngle(angle);
        super.setGoal(m_goalRadians);
        SmartDashboard.putNumber("shooterPivot/goal", Math.toDegrees(m_goalRadians));
    }

    public boolean angleWithinTolerance() {
        return Math.abs(m_goalRadians - getAngleRadians()) < ANGLE_TOLERANCE_RADIAN;
    }

    public void resetGoal() {
        setAngle(getAngleRadians());
    }

    public void setCoastMode() {
        boolean coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", false);
        if (coastMode) {
            m_motor.setIdleMode(IdleMode.kCoast);
            m_motor.stopMotor();
        } else
            m_motor.setIdleMode(IdleMode.kBrake);
    }
}