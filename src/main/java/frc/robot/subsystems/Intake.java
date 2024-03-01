package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    final static double INTAKE_SPEED = 0.5;
    final static double INTAKE_CENTERING_SPEED = 0.5;
    final static double OUTTAKE_SPEED = -0.3;
    final static double OUTTAKE_CENTERING_SPEED = -0.3;

    CANSparkMax m_intakeMotor;
    CANSparkMax m_centeringMotor;

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
    }

    public void run(double rollerSpeed, double centeringWheelSpeed) {
        m_intakeMotor.set(rollerSpeed);
        m_centeringMotor.set(centeringWheelSpeed);
    }

    public void intake() {
        run(INTAKE_SPEED, INTAKE_CENTERING_SPEED);
    }

    public void outtake() {
        run(OUTTAKE_SPEED, OUTTAKE_CENTERING_SPEED);
    }

    public void stop() {
        run(0, 0);
    }
}