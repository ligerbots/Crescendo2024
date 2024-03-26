package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    final static double INTAKE_SPEED = 0.5;
    final static double INTAKE_CENTERING_SPEED = 0.5;
    final static double INTAKE_CENTERING_CURRENT_THRESHOLD = 15;
    final static double OUTTAKE_SPEED = -0.3;
    final static double OUTTAKE_CENTERING_SPEED = -0.3;
    
    // Variables and constants to detect a NOTE 
    private enum IntakeState {IDLE, WAITING_FOR_NOTE, HAS_NOTE};
    private IntakeState m_noteIntakeState = IntakeState.IDLE;

    // median filter to filter the feeder current, to signal holding a note
    private final MedianFilter m_medianFilter = new MedianFilter(5);
    protected double m_medianCenteringCurrent = 0.0;
    private final LinearFilter m_aveFilter = LinearFilter.movingAverage(5);
    protected double m_aveCenteringCurrent = 0.0;

    final CANSparkMax m_intakeMotor;
    final CANSparkMax m_centeringMotor;

    private boolean m_prevState;
    private boolean m_pastFirst;

    public Intake(){
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        m_intakeMotor.setInverted(true);

        m_centeringMotor = new CANSparkMax(Constants.CENTERING_MOTOR_CAN_ID, MotorType.kBrushless);
        m_centeringMotor.setInverted(false);
        // increase update frequency of Status Frame 1 to get faster current updates
        m_centeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/intakeSpeed", m_intakeMotor.get());
        SmartDashboard.putNumber("intake/centeringSpeed", m_centeringMotor.get());
        SmartDashboard.putNumber("intake/intakeCurrent", m_intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake/centeringCurrent", m_centeringMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake/medianCenterCurrent", m_medianCenteringCurrent);
        SmartDashboard.putNumber("intake/aveCenterCurrent", m_aveCenteringCurrent);

        boolean currState = noteInCentering();
        SmartDashboard.putBoolean("intake/noteInCentering", currState);
        SmartDashboard.putBoolean("intake/hasNote", hasNote());
        SmartDashboard.putString("intake/intakeState", m_noteIntakeState.toString());

        if (m_noteIntakeState == IntakeState.WAITING_FOR_NOTE) {
            if (m_prevState && !currState) {
                if (!m_pastFirst) {
                    m_pastFirst = true;
                } else {
                    // note has passed through the Centering Wheels
                    // start rumbling
                    m_noteIntakeState = IntakeState.HAS_NOTE;
                }
            }

            m_prevState = currState;
        }
    }

    public boolean hasNote() {
        return m_noteIntakeState == IntakeState.HAS_NOTE;
    }

    public void clearHasNote() {
        m_noteIntakeState = IntakeState.IDLE;
    }

    public void run(double rollerSpeed, double centeringWheelSpeed) {
        m_intakeMotor.set(rollerSpeed);
        m_centeringMotor.set(centeringWheelSpeed);
    }

    public void intake() {
        m_noteIntakeState = IntakeState.WAITING_FOR_NOTE;
        m_prevState = false;
        m_pastFirst = false;
        // System.err.println("*** RUNNING INTAKE");
        run(INTAKE_SPEED, INTAKE_CENTERING_SPEED);
    }

    public void outtake() {
        run(OUTTAKE_SPEED, OUTTAKE_CENTERING_SPEED);
    }

    public void stop() {
        run(0, 0);
    }

    public Runnable updateCenteringCurrentReadingPeriodic(){
        return () -> {
            m_medianCenteringCurrent = m_medianFilter.calculate(m_centeringMotor.getOutputCurrent());
            m_aveCenteringCurrent = m_aveFilter.calculate(m_centeringMotor.getOutputCurrent());
        };
    }

    public boolean noteInCentering(){
        return m_medianCenteringCurrent > INTAKE_CENTERING_CURRENT_THRESHOLD;
    }
}