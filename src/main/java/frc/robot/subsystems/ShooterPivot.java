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
    //Note: Current values for limits are refrenced with the shooter being flat facing fowards as zero.
    //As of writing the above note we still may want to change the limits
    private static final double MAX_ANGLE = Math.toRadians(90.0); //TODO update, need to talk to CAD
    private static final double MIN_ANGLE = Math.toRadians(11.430453); //TODO update, need to talk to CAD
    //NOTE: All constants were taken from the 2023 arm 
    public static final double ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0); //TODO: tune

    private static final int CURRENT_LIMIT = 10;

    // All units are MKS with angles in Radians
      
    // Constants to limit the shooterPivot rotation speed
    private static final double MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(40); //TODO: tune
    private static final double MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(40); //TODO: tune

    private static final double POSITION_OFFSET = 62.0/360.0; //TODO: Check if value good
    private static final double OFFSET_RADIAN = POSITION_OFFSET * 2 * Math.PI;

    private static final double GEAR_RATIO = 1.0 / 22.5;  // TODO: check to see if changed before run on robot

    // PID Constants for the shooterPivot PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double K_P = 0.01;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;

    //Used in conversion factor
    private static final double RADIAN_PER_ROTATION = 2 * Math.PI * GEAR_RATIO; //TODO:Check if valid

    private DutyCycleEncoder m_absoluteEncoder; //TODO:Check if using same encoder, if not need to update   
    private final CANSparkMax m_motor;
    private final SparkPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    private boolean m_coastMode = false;

    //Used for checking if on goal
    private double m_goalRAD;

    // Construct a new shooterPivot subsystem
    public ShooterPivot(DutyCycleEncoder dutyCycleEncoder) {
        super(new TrapezoidProfile.Constraints(MAX_VEL_RADIAN_PER_SEC, MAX_ACC_RADIAN_PER_SEC_SQ),
                OFFSET_RADIAN - dutyCycleEncoder.getDistance() * 2 * Math.PI);
       
        m_motor = new CANSparkMax(Constants.SHOOTER_PIVOT_CAN_ID, CANSparkMax.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
       
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        m_pidController = m_motor.getPIDController();
        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);
        m_pidController.setFF(K_FF);
       
        // Absolute encoder stuff:
        m_absoluteEncoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_absoluteEncoder.setDistancePerRotation(2 * Math.PI);
        m_absoluteEncoder.setPositionOffset(POSITION_OFFSET);
        double initialAngle = getAbsEncoderAngleRadians();
        // SmartDashboard.putNumber("shooterPivot/initAngle", Units.radiansToDegrees(initialAngle));

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(RADIAN_PER_ROTATION);
        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_encoder.setPosition(initialAngle);

        setCoastMode(m_coastMode);
        SmartDashboard.putBoolean("shooterPivot/coastMode", m_coastMode);
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // Add some extra numbers to diagnose the load on the motors
        SmartDashboard.putNumber("shooterPivot/encoder", Math.toDegrees(getAngleRadians()));
        SmartDashboard.putNumber("shooterPivot/absoluteEncoder", Math.toDegrees(getAbsEncoderAngleRadians()));
        // SmartDashboard.putBoolean("shooterPivot/m_resetPivotPos", m_resetPivotPos)

        // m_coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", m_coastMode);

        // if (m_coastMode)
        //     setCoastMode(m_coastMode);

        // Execute the super class periodic method
        super.periodic();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Remember that the encoder was already set to account for the gear ratios.

        m_pidController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("shooterPivotPivot/setPoint", Math.toDegrees(setPoint.position));
    }

    // get the current pivot angle in radians
    public double getAngleRadians() {
        return m_encoder.getPosition();
    }

    // get the angle from the absolute encoder
    public double getAbsEncoderAngleRadians() {
        return -m_absoluteEncoder.getDistance();
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitPivotAngle(double angle){
        return Math.min(MAX_ANGLE, Math.max(MIN_ANGLE, angle));
    }

    // set shooterPivot angle in radians
    public void setAngle(double angle) {
        m_goalRAD = limitPivotAngle(angle);
        super.setGoal(m_goalRAD);
        SmartDashboard.putNumber("shooterPivot/goal", Math.toDegrees(m_goalRAD));
    }
    public void resetGoal(){
        setAngle(getAngleRadians());
        // super.disable();
        // m_superEnabled = false;
    }

    public void setCoastMode(boolean coastMode){
        if (coastMode) {
            m_motor.setIdleMode(IdleMode.kCoast);
            m_motor.stopMotor();
        } else
            m_motor.setIdleMode(IdleMode.kBrake);
    }

    public boolean isWithinTolerence() {
        return Math.abs(m_goalRAD-getAngleRadians()) < ANGLE_TOLERANCE_RADIAN;
    }
}