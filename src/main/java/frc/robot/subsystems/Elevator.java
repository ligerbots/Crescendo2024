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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Elevator extends TrapezoidProfileSubsystem {

    private static final double ELEVATOR_MAX_LENGTH = Units.inchesToMeters(22.0);
    private static final double ELEVATOR_MIN_LENGTH = Units.inchesToMeters(0.0);

    public static final double ELEVATOR_OFFSET_TOLERANCE_METERS = Units.inchesToMeters(1.0); //TODO: Find tolorence

    // For initial testing, these should be very slow.
    // We can update them as we get more confidence.
    private static final double ELEVATOR_MAX_VEL_METER_PER_SEC = Units.inchesToMeters(100.0);

    // private static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(50.0);

    private static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(100.0); //TODO: Find actual value

    private static final double ELEVATOR_METER_PER_REVOLUTION = Units.inchesToMeters(0.7); //TODO: find actual value


    // PID Constants for the reacher PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double ELEVATOR_K_P = 0.1; //TODO: Need to tune
    // private static final double ELEVATOR_K_P1 = 100;
    private static final double ELEVATOR_K_I = 0.0;
    private static final double ELEVATOR_K_D = 0.0;
    private static final double ELEVATOR_K_FF = 0.0;

    // Define the motor and encoders
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;

    //initializing Potentiometer
    private final int POTENTIOMETER_CHANNEL = 2; //TODO: Update with actual value
    private final double POTENTIOMETER_RANGE = -2.625; // meters, the string potentiometer on takes in range in integers TODO: update to correct value
    private final double POTENTIOMETER_OFFSET = 2.51; //TODO: Find inital value and update
    private final AnalogPotentiometer m_stringPotentiometer;

    private final SparkPIDController m_PIDController;

    private double m_kPElevator;
    private boolean m_coastMode = false;
    private double m_goal = 0;

    /** Creates a new Elevator. */
    public Elevator() {
        super(new TrapezoidProfile.Constraints(ELEVATOR_MAX_VEL_METER_PER_SEC, ELEVATOR_MAX_ACC_METER_PER_SEC_SQ));

        m_kPElevator = ELEVATOR_K_P;

        // Create the motor, PID Controller and encoder.
        m_motor = new CANSparkMax(Constants.ELEVATOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        
        //set currentLimit for reacher to 35 amps
        m_motor.setSmartCurrentLimit(35);

        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(ELEVATOR_K_P);
        m_PIDController.setI(ELEVATOR_K_I);
        m_PIDController.setD(ELEVATOR_K_D);
        m_PIDController.setFF(ELEVATOR_K_FF);
        // m_motor.setInverted(true);

        m_encoder = m_motor.getEncoder();
        // Set the position conversion factor.
        m_encoder.setPositionConversionFactor(ELEVATOR_METER_PER_REVOLUTION);

        m_stringPotentiometer = new AnalogPotentiometer(POTENTIOMETER_CHANNEL, POTENTIOMETER_RANGE, POTENTIOMETER_OFFSET);
        // m_encoder.setPosition(ELEVATOR_OFFSET_METER);
        m_encoder.setPosition(getPotentiometerReadingMeters());

        setCoastMode(false);
        SmartDashboard.putBoolean("Elevator/coastMode", m_coastMode);

        SmartDashboard.putNumber("Elevator/P Gain", m_kPElevator);
    }

    @Override
    public void periodic() {
        double encoderValue = m_encoder.getPosition();
        SmartDashboard.putNumber("Elevator/encoder", Units.metersToInches(encoderValue));
        SmartDashboard.putNumber("Elevator/encoderMeter", encoderValue);
        SmartDashboard.putNumber("Elevator/goal", Units.metersToInches(m_goal));
        // SmartDashboard.putBoolean("Elevator/mesetElevatorPos", m_resetElevatorPos);
        
        SmartDashboard.putNumber("Elevator/stringPotMeter", getPotentiometerReadingMeters());

        //Toggle to turn on and off cost mode
        m_coastMode = SmartDashboard.getBoolean("Elevator/coastMode", m_coastMode);
        if (m_coastMode) {
            setCoastMode(m_coastMode);
        }
        super.periodic();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Remember that the encoder was already set to account for the gear ratios.

        m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition); //(setPoint.position, ControlType.kPosition, 0); // , feedforward / 12.0);
        SmartDashboard.putNumber("Elevator/setPoint", Units.metersToInches(setPoint.position));
    }

    public double getLength() {
        return m_encoder.getPosition();
    }

    public double getPotentiometerReadingMeters(){
        return m_stringPotentiometer.get();
    }

    // this needs to be public so that commands can get the restricted distance. (safety, limits of to high)
    public static double limitElevatorLength(double length){
        return Math.min(ELEVATOR_MAX_LENGTH, Math.max(ELEVATOR_MIN_LENGTH, length));
    }

    // set reacher length in inches
    public void setLength(double goal) {
        m_goal = limitElevatorLength(goal);
        super.setGoal(m_goal);
    }

    public void setCoastMode(boolean coastMode){
        if (coastMode) {
            m_motor.setIdleMode(IdleMode.kCoast);
            m_motor.stopMotor();
        } else
            m_motor.setIdleMode(IdleMode.kBrake);
    }
}