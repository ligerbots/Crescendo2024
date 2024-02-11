package frc.robot.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class FalconDriveController implements DriveController {
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    // This is the L2 gearing
    private static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    private static final double FALCON_DISTANCE_PER_UNIT = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION / 2048.0 ;
    
    private static final double CURRENT_LIMIT = 35.0;  // Amps
    private static final double CURRENT_LIMIT_TIME = 0.25;  // seconds
    
    private static final boolean MOTOR_INVERTED = true;

    private final TalonFX m_motor;

    public FalconDriveController(int id) {
        m_motor = new TalonFX(id);
        m_motor.setInverted(MOTOR_INVERTED);
    
        CurrentLimitsConfigs motorCurrentConfigs = new CurrentLimitsConfigs();
        motorCurrentConfigs.withSupplyCurrentThreshold(CURRENT_LIMIT);
        motorCurrentConfigs.withSupplyTimeThreshold(CURRENT_LIMIT_TIME);
        motorCurrentConfigs.withSupplyCurrentLimitEnable(true);
        m_motor.getConfigurator().apply(motorCurrentConfigs);
    
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
