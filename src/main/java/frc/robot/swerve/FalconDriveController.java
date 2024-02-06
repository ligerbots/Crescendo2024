package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.DriveTrain;

public class FalconDriveController implements DriveController {
    private final TalonFX m_motor;
    

    private static final double FALCON_DISTANCE_PER_UNIT = (Math.PI * DriveTrain.WHEEL_DIAMETER) / 6.75 / 2048.0 ;
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
        return m_motor.getVelocity().getValueAsDouble() * FALCON_DISTANCE_PER_UNIT;
    }

    @Override
    public double getWheelDistance() {
        return m_motor.getPosition().getValueAsDouble() * FALCON_DISTANCE_PER_UNIT;
    }

}
