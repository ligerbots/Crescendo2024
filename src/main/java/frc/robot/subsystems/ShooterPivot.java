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
    private static final double SHOOTER_PIVOT_MAX_ANGLE = Math.toRadians(30.0); //TODO update, need to talk to CAD
    private static final double SHOOTER_PIVOT_MIN_ANGLE = Math.toRadians(-65.0); //TODO update, need to talk to CAD
    //NOTE: All constants were taken from the 2023 arm 
    public static final double SHOOTER_PIVOT_ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0); //TODO: tune

    private static final int CURRENT_LIMIT = 10;

    // All units are MKS with angles in Radians
      
    // Constants to limit the shooterPivot rotation speed
    private static final double SHOOTER_PIVOT_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(40); //TODO: tune
    private static final double SHOOTER_PIVOT_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(40); //TODO: tune

    private static final double SHOOTER_PIVOT_POSITION_OFFSET = 62.0/360.0; //TODO: Check if value good
    private static final double SHOOTER_PIVOT_OFFSET_RADIAN = SHOOTER_PIVOT_POSITION_OFFSET * 2 * Math.PI;

    private static final double SHOOTER_PIVOT_GEAR_RATIO = 1/22.5;//TODO: check to see if changed before run on robot

    // PID Constants for the shooterPivot PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOOTER_PIVOT_K_P = 0.15;
    private static final double SHOOTER_PIVOT_K_I = 0.0;
    private static final double SHOOTER_PIVOT_K_D = 0.0;
    private static final double SHOOTER_PIVOT_K_FF = 0.0;

    //Used in conversion factor
    private static final double SHOOTER_PIVOT_RADIAN_PER_ROTATION = 2 * Math.PI * SHOOTER_PIVOT_GEAR_RATIO; //TODO:Check if valid

    DutyCycleEncoder m_dutyEncoder; //TODO:Check if using same encoder, if not need to update
    
    private boolean m_coastMode = false;

    // current goal in radians
    private double m_goal;

    private final CANSparkMax m_motor;
    private final SparkPIDController m_PIDController;
    private final RelativeEncoder m_encoder;

    // Construct a new shooterPivot subsystem
    public ShooterPivot(DutyCycleEncoder dutyCycleEncoder) {
        super(new TrapezoidProfile.Constraints(SHOOTER_PIVOT_MAX_VEL_RADIAN_PER_SEC, SHOOTER_PIVOT_MAX_ACC_RADIAN_PER_SEC_SQ),(SHOOTER_PIVOT_OFFSET_RADIAN-dutyCycleEncoder.getDistance() * 2 * Math.PI));
       
        m_motor = new CANSparkMax(Constants.SHOOTER_PIVOT_ID, CANSparkMax.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
       
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(SHOOTER_PIVOT_K_P);
        m_PIDController.setI(SHOOTER_PIVOT_K_I);
        m_PIDController.setD(SHOOTER_PIVOT_K_D);
        m_PIDController.setFF(SHOOTER_PIVOT_K_FF);
       
        //Absolute encoder stuff:
        m_dutyEncoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_dutyEncoder.setDistancePerRotation(2 * Math.PI);
        m_dutyEncoder.setPositionOffset(SHOOTER_PIVOT_POSITION_OFFSET);
        double initialAngle = -m_dutyEncoder.getDistance();
        // SmartDashboard.putNumber("shooterPivot/initAngle", Units.radiansToDegrees(initialAngle));

        m_encoder = m_motor.getEncoder();
        m_encoder.setPositionConversionFactor(SHOOTER_PIVOT_RADIAN_PER_ROTATION);
        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_encoder.setPosition(initialAngle);

        setCoastMode(false);
        SmartDashboard.putBoolean("shooterPivot/coastMode", m_coastMode);
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // Add some extra numbers to diagnose the load on the motors
        SmartDashboard.putNumber("shooterPivot/encoder", Math.toDegrees(getPivotAngle()));
        SmartDashboard.putNumber("shooterPivot/goal", Math.toDegrees(m_goal));
        SmartDashboard.putNumber("shooterPivot/absoluteEncoder", Math.toDegrees(-m_dutyEncoder.getDistance()));
        // SmartDashboard.putBoolean("shooterPivot/m_resetPivotPos", m_resetPivotPos)

        m_coastMode = SmartDashboard.getBoolean("shooterPivot/coastMode", m_coastMode);

        if (m_coastMode)
            setCoastMode(m_coastMode);

        // Execute the super class periodic method
        super.periodic();
    }

    @Override
        protected void useState(TrapezoidProfile.State setPoint) {
        // Remember that the encoder was already set to account for the gear ratios.

        m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition); //(setPoint.position, ControlType.kPosition, 0); // , feedforward / 12.0);
        SmartDashboard.putNumber("shooterPivotPivot/setPoint", Math.toDegrees(setPoint.position));
    }

    // return current pivot angle in radians
    public double getPivotAngle() {
        return m_encoder.getPosition();
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitPivotAngle(double angle){
        return Math.min(SHOOTER_PIVOT_MAX_ANGLE, Math.max(SHOOTER_PIVOT_MIN_ANGLE, angle));
    }

    // set shooterPivot angle in radians
    public void setAngle(double angle) {
        m_goal = limitPivotAngle(angle);
        super.setGoal(m_goal);
    }
    public void resetGoal(){
        setAngle(getPivotAngle());
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
}