package frc.robot.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FalconDriveController implements DriveController {
    private final TalonFX m_motor;

    // private static final double CURRENT_LIMIT = 9999;
    private static final boolean MOTOR_INVERTED = true;

    // public static final double DRIVE_REDUCTION = 0;
    // public static final double WHEEL_DIAMETER = 1;

    public FalconDriveController(int id) {
        m_motor = new TalonFX(id);
        m_motor.setInverted(MOTOR_INVERTED);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setReferenceVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public double getStateVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getWheelDistance() {
        return m_motor.getPosition().getValueAsDouble();
    }

}
