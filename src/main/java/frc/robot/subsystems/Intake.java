package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    final static double INTAKE_SPEED = 0.1;
    final static double INTAKE_CENTERING_SPEED = 0.1;
    final static double OUTTAKE_SPEED = -0.1;
    final static double OUTTAKE_CENTERING_SPEED = -0.1;
    CANSparkMax m_intakeMotor;
    CANSparkMax m_centeringMotor;

    public Intake(){
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        m_centeringMotor = new CANSparkMax(Constants.CENTERING_MOTOR_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double rollerSpeed, double centeringWheelSpeed) {
        m_intakeMotor.set(rollerSpeed); //TODO: Find if posotive or negitive
        m_centeringMotor.set(centeringWheelSpeed); //TODO: Find if posotive or negitive
    }

    public void intake() {
        run(INTAKE_SPEED, INTAKE_CENTERING_SPEED);
    }

    public void outtake() {
        run(OUTTAKE_SPEED, OUTTAKE_CENTERING_SPEED);
    }

    public void stop() {
        run(0,0);
    }

}