package frc.robot.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconDriveController implements DriveController {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    // This is the L2 gearing
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 
            6380.0 / 60.0 * DRIVE_REDUCTION * (Math.PI * WHEEL_DIAMETER);
    
    // Note: Phoenix 6 library uses rotations, so no need for tick count
    private static final double FALCON_DISTANCE_PER_UNIT = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;
    
    private static final double THRESHOLD_CURRENT_LIMIT = 80.0; // Amps 
    private static final double CURRENT_LIMIT = 40.0;  // Amps
    private static final double CURRENT_LIMIT_TIME = 18;  // seconds. taken from the data sheet 
    
    private static final boolean MOTOR_INVERTED = true;


    private final TalonFX m_motor;

    public FalconDriveController(int id) {
        m_motor = new TalonFX(id);
        m_motor.setInverted(MOTOR_INVERTED);
    
        CurrentLimitsConfigs motorCurrentConfigs = new CurrentLimitsConfigs();

        motorCurrentConfigs.withSupplyCurrentThreshold(THRESHOLD_CURRENT_LIMIT);// this is the peak current it will take
        motorCurrentConfigs.withSupplyTimeThreshold(CURRENT_LIMIT_TIME);// if the peak current is held for this long it drops to the current limit 
        motorCurrentConfigs.withSupplyCurrentLimit(CURRENT_LIMIT);// regular current limit 
        motorCurrentConfigs.withSupplyCurrentLimitEnable(true);
        motorCurrentConfigs.withStatorCurrentLimitEnable(false);
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

    @Override
    public void updateSmartDashboard(String sdPrefix) {
        SmartDashboard.putNumber(sdPrefix + "/speed", getStateVelocity());
        SmartDashboard.putNumber(sdPrefix + "/position", getWheelDistance());
        SmartDashboard.putNumber(sdPrefix + "/current", m_motor.getSupplyCurrent().getValueAsDouble());
    }
}
