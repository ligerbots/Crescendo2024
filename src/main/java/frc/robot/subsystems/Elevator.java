package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Elevator extends TrapezoidProfileSubsystem {

    private static final double ELEVATOR_MAX_VELOCITY_METER_PER_SEC = 12.0; 
    private static final double ELEVATOR_MAX_ACCELLERATION_METER_PER_SEC_SQ = 12.0; 

    private static final double ELEVATOR_MIN_HEIGHT = 12.0;
    private static final double ELEVATOR_MAX_HEIGHT = 48.0;

    ELEVATOR_MIN_HEIGHT

    private static final double ELEVATOR_METER_PER_REVOLUTION = 0; //TODO this needs to be set up 
    private static final double ELEVATOR_OFFSET_METER = Units.inchesToMeters(0.0);

    // Feedforward constants for the elevator
    private static final double ELEVATOR_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/linear
    private static final double ELEVATOR_KG = 0.16; // Volts
    private static final double ELEVATOR_KV = 3.07; // V*sec/meter
    private static final double ELEVATOR_KA = 0.03; // V*sec^2/meter

    // PID Constants for the elevator PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double ELEVATOR_K_P = 10.0;
    // private static final double ELEVATOR_K_P1 = 100;
    private static final double ELEVATOR_K_I = 0.0;
    private static final double ELEVATOR_K_D = 0.0;
    private static final double ELEVATOR_K_FF = 0.0;

    // Define the motor and encoders
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;

    //initializing Potentiometer

    private final SparkPIDController m_PIDController;

    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ELEVATOR_KS,
                    ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

    private double m_kPReacher;
    private boolean m_resetReacherPos = false;
    private boolean m_coastMode = false;
    private double m_goal = 0;
    


    public Elevator() {
        super(new TrapezoidProfile.Constraints(ELEVATOR_MAX_VELOCITY_METER_PER_SEC, ELEVATOR_MAX_ACCELLERATION_METER_PER_SEC_SQ));
        
        m_kPReacher = ELEVATOR_K_P;

        // Create the motor, PID Controller and encoder.
        m_motor = new CANSparkMax(Constants.ELEVATOR_CAN_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        
        //set currentLimit for elevator to 35 amps
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

        setCoastMode(false);
        SmartDashboard.putBoolean("Reacher/coastMode", m_coastMode);

        SmartDashboard.putNumber("Reacher/P Gain", m_kPReacher);
 

    }

    
    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Calculate the feedforward from the setPoint
        // double feedforward = m_feedForward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // Remember that the encoder was already set to account for the gear ratios.

        if (m_resetReacherPos) {
            setPoint.position = m_encoder.getPosition();
            m_resetReacherPos = false;
        }
        m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0); // , feedforward / 12.0);
        SmartDashboard.putNumber("Elevator/setPoint", Units.metersToInches(setPoint.position));
    }

    public double getLength() {
        return m_encoder.getPosition();
    }

    public static double limitElevatorLength(double length){
        return Math.min(ELEVATOR_MAX_HEIGHT, Math.max(ELEVATOR_MIN_HEIGHT, length));
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
