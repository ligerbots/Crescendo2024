// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants;

public class ShooterPivot extends TrapezoidProfileSubsystem {
    private static final double TWO_PI = 2.0 * Math.PI;
    
    public static final double MIN_ANGLE = Math.toRadians(0.0);
    public static final double MAX_ANGLE = Math.toRadians(60.0);
    // NOTE: All constants were taken from the 2023 arm 
    // Note: Current values for limits are refrenced with the shooter being flat
    // facing fowards as zero.
    // As of writing the above note we still may want to change the limits
    public static final double ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0); //TODO: set for 2024

    private static final int CURRENT_LIMIT = 30;

    // position constants for commands
    public static final double STOW_ANGLE_RADIANS = Math.toRadians(58.0);
    public static final double AMP_SCORE_ANGLE_RADIANS = Math.toRadians(45);
    
    // All units are MKS with angles in Radians
      
    // Constants to limit the shooterPivot rotation speed
    private static final double MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(40); //TODO: set for 2024
    private static final double MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(40); //TODO: set for 2024

    private static final double POSITION_OFFSET = 59.5/360.0; 
    // private static final double OFFSET_RADIAN = POSITION_OFFSET * 2 * Math.PI;

    // 15:1 planetary plus 48:32 sprockets
    private static final double GEAR_RATIO = (1.0 / 15.0) * (32.0 / 48.0);

    // Constants for the shooterPivot PID controller
    private static final double K_P = 1.0;  // TODO: set for 2024
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;
    private static final double K_FF = 0.0;  // TODO: set for 2024

    // Used in conversion factor
    // private static final double RADIANS_PER_MOTOR_ROTATION = 2 * Math.PI * GEAR_RATIO;

    private final DutyCycleEncoder m_absoluteEncoder;  
    private final CANSparkMax m_motor;
    private final SparkPIDController m_pidController;
    private final RelativeEncoder m_encoder;

    // Used for checking if on goal
    private double m_goalRadians = 0;

    // Construct a new shooterPivot subsystem
    public ShooterPivot(DutyCycleEncoder absEncoder) {
        super(new TrapezoidProfile.Constraints(MAX_VEL_RADIAN_PER_SEC, MAX_ACC_RADIAN_PER_SEC_SQ));
       
        m_motor = new CANSparkMax(Constants.SHOOTER_PIVOT_CAN_ID, CANSparkMax.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);

        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);
    
        m_pidController = m_motor.getPIDController();
        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);
        m_pidController.setFF(K_FF);
        m_pidController.setOutputRange(-1, 1);

        // Absolute encoder - work in rotations
        m_absoluteEncoder = absEncoder;
        // m_absoluteEncoder.setDistancePerRotation(2 * Math.PI);
        m_absoluteEncoder.setPositionOffset(POSITION_OFFSET);

        // motor encoder - set calibration and offset to match absolute encoder
        m_encoder = m_motor.getEncoder();
        // m_encoder.setPositionConversionFactor(RADIANS_PER_MOTOR_ROTATION);
        m_encoder.setPositionConversionFactor(GEAR_RATIO);
        updateMotorEncoderOffset();
        // resetGoal();

        SmartDashboard.putBoolean("shooterPivot/coastMode", false);
        setCoastMode();

        SmartDashboard.putNumber("shooterPivot/testAngle", 0);

        if (Constants.SIMULATION_SUPPORT) {
            // Simulation
            REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
        }
    }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // This also gets logged to the log file on the Rio and aids in replaying a match
        SmartDashboard.putNumber("shooterPivot/encoder", Math.toDegrees(getAngleRadians()));
        SmartDashboard.putNumber("shooterPivot/absoluteEncoder", Math.toDegrees(getAbsEncoderAngleRadians()));
        SmartDashboard.putNumber("shooterPivot/current", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("shooterPivot/onGoal", angleWithinTolerance());

        setCoastMode();

        // Execute the super class periodic method
        super.periodic();
    }


    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Remember that the encoder was already set to account for the gear ratios.
        // System.out.println("Pivot setpoint = " + setPoint.position);

        m_pidController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("shooterPivot/setPoint", Math.toDegrees(TWO_PI * setPoint.position));
    }

    // get the current pivot angle in radians
    public double getAngleRadians() {
        return TWO_PI * m_encoder.getPosition();
    }

    // get the angle from the absolute encoder
    public double getAbsEncoderAngleRadians() {
        return TWO_PI * m_absoluteEncoder.getDistance();
    }

    // update the motor encoder offset to match the absolute encoder
    public void updateMotorEncoderOffset() {
        m_encoder.setPosition(m_absoluteEncoder.getDistance());
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitPivotAngle(double angle) {
        return MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    }

    // set shooterPivot angle in radians
    public void setAngle(double angle) {
        m_goalRadians = limitPivotAngle(angle);

        // System.out.println("Pivot set goal = " + Math.toDegrees(m_goalRadians));
        super.setGoal(m_goalRadians / TWO_PI);
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

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();
    }
}