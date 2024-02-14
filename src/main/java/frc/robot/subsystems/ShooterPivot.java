// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants;

public class ShooterPivot extends TrapezoidProfileSubsystem {
    private static final double MAX_ANGLE = Math.toRadians(30.0); //TODO set for 2024
    private static final double MIN_ANGLE = Math.toRadians(-65.0); //TODO set for 2024
    //NOTE: All constants were taken from the 2023 arm 
    public static final double ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0); //TODO: set for 2024

    private static final int CURRENT_LIMIT = 10;

    // All units are MKS with angles in Radians
      
    // Constants to limit the shooterPivot rotation speed
    private static final double MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(40); //TODO: set for 2024
    private static final double MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(40); //TODO: set for 2024

    private static final double POSITION_OFFSET = 62.0/360.0; //TODO: set for 2024
    private static final double OFFSET_RADIAN = POSITION_OFFSET * 2 * Math.PI;

    private static final double GEAR_RATIO = 1.0 / 22.5;  // TODO: set for 2024

    // Constants for the shooterPivot PID controller
    private static final double K_P = 0.01;  // TODO: set for 2024
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;  // TODO: set for 2024

    // Used in conversion factor
    private static final double RADIANS_PER_MOTOR_ROTATION = 2 * Math.PI * GEAR_RATIO;

    private final DutyCycleEncoder m_absoluteEncoder;  
    private final CANSparkMax m_motor;
    private final SparkPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    // Construct a new shooterPivot subsystem
    public ShooterPivot(DutyCycleEncoder absEncoder) {
        super(new TrapezoidProfile.Constraints(MAX_VEL_RADIAN_PER_SEC, MAX_ACC_RADIAN_PER_SEC_SQ),
                OFFSET_RADIAN - absEncoder.getDistance() * 2 * Math.PI);
       
        m_motor = new CANSparkMax(Constants.SHOOTER_PIVOT_CAN_ID, CANSparkMax.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
       
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        m_pidController = m_motor.getPIDController();
        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);
        m_pidController.setFF(K_FF);
       
        // Absolute encoder - set calibration to use radians
        m_absoluteEncoder = absEncoder;
        m_absoluteEncoder.setDistancePerRotation(2 * Math.PI);
        m_absoluteEncoder.setPositionOffset(POSITION_OFFSET);

        // motor encoder - set calibration and offset to match absolute encoder
        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(RADIANS_PER_MOTOR_ROTATION);
        updateMotorEncoderOffset();

        SmartDashboard.putBoolean("shooterPivot/coastMode", false);
        setCoastMode();
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("shooterPivot/encoder", Math.toDegrees(getAngleRadians()));
        SmartDashboard.putNumber("shooterPivot/absoluteEncoder", Math.toDegrees(getAbsEncoderAngleRadians()));
        SmartDashboard.putNumber("shooterPivot/current", m_motor.getOutputCurrent());

        setCoastMode();

        // Execute the super class periodic method
        super.periodic();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Remember that the encoder was already set to account for the gear ratios.

        m_pidController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("shooterPivot/setPoint", Math.toDegrees(setPoint.position));
    }

    // get the current pivot angle in radians
    public double getAngleRadians() {
        return m_encoder.getPosition();
    }

    // get the angle from the absolute encoder
    public double getAbsEncoderAngleRadians() {
        return -m_absoluteEncoder.getDistance();
    }

    // update the motor encoder offset to match the absolute encoder
    public void updateMotorEncoderOffset() {
        m_encoder.setPosition(getAbsEncoderAngleRadians());
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitPivotAngle(double angle) {
        return Math.min(MAX_ANGLE, Math.max(MIN_ANGLE, angle));
    }

    // set shooterPivot angle in radians
    public void setAngle(double angle) {
        double goal = limitPivotAngle(angle);
        super.setGoal(goal);
        SmartDashboard.putNumber("shooterPivot/goal", Math.toDegrees(goal));
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