package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
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
    private static final double INTAKE_BASE_MAX_CURRENT = 18.0;
    private static final double INTAKE_NOTE_MIN_CURRENT = 23.0;
    private static final double INTAKE_NOTE_OUT_MAX_CURRENT = 10.0;
    private enum IntakeState {IDLE, MOTOR_START, WAITING, NOTE_ENTERING, NOTE_PAST_INTAKE, HAS_NOTE};
    private IntakeState m_noteIntakeState = IntakeState.IDLE;

    // median filter to filter the feeder current, to signal holding a note
    private final MedianFilter m_medianFilter = new MedianFilter(10);
    protected double m_curCenteringMotorCurrent = 0.0;

    CANSparkMax m_intakeMotor;
    CANSparkMax m_centeringMotor;

    Timer m_timer = new Timer();
    boolean m_prevHasNote = false;

    public Intake(){
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        m_centeringMotor = new CANSparkMax(Constants.CENTERING_MOTOR_CAN_ID, MotorType.kBrushless);
        m_intakeMotor.setInverted(true);
        m_centeringMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/intakeSpeed", m_intakeMotor.get());
        SmartDashboard.putNumber("intake/centeringSpeed", m_centeringMotor.get());
        SmartDashboard.putNumber("intake/intakeCurrent", m_intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake/centeringCurrent", m_centeringMotor.getOutputCurrent());
        SmartDashboard.putBoolean("intake/noteInCentering", noteInCentering());
        SmartDashboard.putBoolean("intake/hasNote", hasNote());
        
        // look for the Note by checking the Intake current
        if (m_noteIntakeState == IntakeState.MOTOR_START) {
            // feeder current spikes when the motors start
            if (m_intakeMotor.getOutputCurrent() < INTAKE_BASE_MAX_CURRENT) {
                m_noteIntakeState = IntakeState.WAITING;
                m_prevHasNote = false;
            }
        } else if (m_noteIntakeState == IntakeState.WAITING) {
            boolean noteState = noteInCentering();
            if (m_prevHasNote && !noteState) {
                m_noteIntakeState = IntakeState.NOTE_PAST_INTAKE;
                m_timer.restart();
            }
            m_prevHasNote = noteState;
            
        //     if (m_intakeMotor.getOutputCurrent() > INTAKE_NOTE_MIN_CURRENT) {
        //         m_noteIntakeState = IntakeState.NOTE_ENTERING;
        //     }
        // } else if (m_noteIntakeState == IntakeState.NOTE_ENTERING) {
        //     if (m_intakeMotor.getOutputCurrent() < INTAKE_NOTE_OUT_MAX_CURRENT) {
        //         m_noteIntakeState = IntakeState.NOTE_PAST_INTAKE;
        //         m_timer.restart();
        //     }
        } else if (m_noteIntakeState == IntakeState.NOTE_PAST_INTAKE) {
            if (m_timer.hasElapsed(0.1)) {
                m_noteIntakeState = IntakeState.HAS_NOTE;
            }
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
        m_noteIntakeState = IntakeState.MOTOR_START;
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
            m_curCenteringMotorCurrent = m_medianFilter.calculate(m_centeringMotor.getOutputCurrent());
        };
    }

    public boolean noteInCentering(){
        return m_curCenteringMotorCurrent > INTAKE_CENTERING_CURRENT_THRESHOLD;
    }
}