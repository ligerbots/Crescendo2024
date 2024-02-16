package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;

// LigerBots DriveController for Swerve

public class NeoDriveController implements DriveController {
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;

    //currentLimit for driving 35 amps
    private static final double CURRENT_LIMIT = 35.0;
    private static final boolean MOTOR_INVERTED = true;

    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double WHEEL_DIAMETER = 0.10033;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 
            5880.0 / 60.0 * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
    
    static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }

    public NeoDriveController(int motorCanId) {
        m_motor = new CANSparkMax(motorCanId, CANSparkLowLevel.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(MOTOR_INVERTED);

        // Setup voltage compensation
        checkNeoError(m_motor.enableVoltageCompensation(Constants.MAX_VOLTAGE), "Failed to enable voltage compensation");
        // set current limit
        checkNeoError(m_motor.setSmartCurrentLimit((int) CURRENT_LIMIT), "Failed to set current limit for NEO");

        // adjust CANBus update periods 
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100),
                "Failed to set periodic status frame 0 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20),
                "Failed to set periodic status frame 1 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20),
                "Failed to set periodic status frame 2 rate");

        // Set neutral mode to brake
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Setup encoder
        m_encoder = m_motor.getEncoder();
        double positionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;
        m_encoder.setPositionConversionFactor(positionConversionFactor);
        m_encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
    }

    // set the drive voltage
    @Override
    public void setReferenceVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    // get the drive velocity
    @Override
    public double getStateVelocity() {
        return m_encoder.getVelocity();
    }

    //get wheel distance
    @Override
    public double getWheelDistance(){
        return m_encoder.getPosition();
    }

}