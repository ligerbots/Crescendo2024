package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    // private static final double INTAKE_BASE_MAX_CURRENT = 18.0;
    // private static final double INTAKE_NOTE_MIN_CURRENT = 23.0;
    // private static final double INTAKE_NOTE_OUT_MAX_CURRENT = 10.0;
    // private enum IntakeState {IDLE, MOTOR_START, WAITING, NOTE_ENTERING, NOTE_PAST_INTAKE, HAS_NOTE};
    private enum IntakeState {IDLE, WAITING_FOR_NOTE, HAS_NOTE};
    private IntakeState m_noteIntakeState = IntakeState.IDLE;

    // median filter to filter the feeder current, to signal holding a note
    private final MedianFilter m_medianFilter = new MedianFilter(15);
    protected double m_curCenteringMotorCurrent = 0.0;
    private final LinearFilter m_aveFilter = LinearFilter.movingAverage(1);
    protected double m_aveCenteringMotorCurrent = 0.0;

    CANSparkMax m_intakeMotor;
    CANSparkMax m_centeringMotor;

    // Timer m_timer = new Timer();
    private boolean m_prevState;
    private boolean m_pastFirst;

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
        SmartDashboard.putNumber("intake/filterCenterCurrent", m_curCenteringMotorCurrent);
        SmartDashboard.putNumber("intake/aveCenterCurrent", m_aveCenteringMotorCurrent);

        boolean currState = noteInCentering();
        SmartDashboard.putBoolean("intake/noteInCentering", currState);
        SmartDashboard.putBoolean("intake/hasNote", hasNote());

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

        // // look for the Note by checking the Intake current
        // if (m_noteIntakeState == IntakeState.MOTOR_START) {
        //     // feeder current spikes when the motors start
        //     if (m_intakmeMotor.getOutputCurrent() < INTAKE_BASE_MAX_CURRENT) {
        //         m_noteIntakeState = IntakeState.WAITING;
        //         m_prevHasNote = false;
        //     }
        // } else if (m_noteIntakeState == IntakeState.WAITING) {
        //     if (m_prevHasNote && !noteInCentering) {
        //         m_noteIntakeState = IntakeState.NOTE_PAST_INTAKE;
        //         m_timer.restart();
        //     }
            
        // //     if (m_intakeMotor.getOutputCurrent() > INTAKE_NOTE_MIN_CURRENT) {
        // //         m_noteIntakeState = IntakeState.NOTE_ENTERING;
        // //     }
        // // } else if (m_noteIntakeState == IntakeState.NOTE_ENTERING) {
        // //     if (m_intakeMotor.getOutputCurrent() < INTAKE_NOTE_OUT_MAX_CURRENT) {
        // //         m_noteIntakeState = IntakeState.NOTE_PAST_INTAKE;
        // //         m_timer.restart();
        // //     }
        // } else if (m_noteIntakeState == IntakeState.NOTE_PAST_INTAKE) {
        //     if (m_timer.hasElapsed(0.1)) {
        //         m_noteIntakeState = IntakeState.HAS_NOTE;
        //     }
        // }
        //     m_prevHasNote = noteInCentering;

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
            m_curCenteringMotorCurrent = m_medianFilter.calculate(m_centeringMotor.getOutputCurrent());
            m_aveCenteringMotorCurrent = m_aveFilter.calculate(m_centeringMotor.getOutputCurrent());
        };
    }

    public boolean noteInCentering(){
        return m_curCenteringMotorCurrent > INTAKE_CENTERING_CURRENT_THRESHOLD;
    }
}