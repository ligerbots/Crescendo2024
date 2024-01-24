package frc.robot.swerve;

import com.revrobotics.CANSparkMax;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;

// LigerBots DriveController for Swerve

public class FalconDriveController {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_configuration;
    // private final RelativeEncoder m_encoder;

    //currentLimit for driving 35 amps
    private static final double CURRENT_LIMIT = 35.0;
    private static final boolean MOTOR_INVERTED = true;

    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double WHEEL_DIAMETER = 0.10033;

    static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }

    public FalconDriveController(int motorCanId) {
        m_motor = new TalonFX(motorCanId);
        m_configuration = new TalonFXConfiguration();
        var m_talonFXConfigurator = m_motor.getConfigurator();
        var m_motorConfigs = new MotorOutputConfigs();
        var m_currentLimitsConfigs = new CurrentLimitsConfigs();

        m_motor.getConfigurator().apply(new TalonFXConfiguration());//this factory resets
        
        
        m_motor.setInverted(MOTOR_INVERTED);

        // Setup voltage compensation
        //idk if there is an equivilant for the falcons 
        checkNeoError(m_motor.enableVoltageCompensation(Constants.MAX_VOLTAGE), "Failed to enable voltage compensation");
        // set current limit
        // checkNeoError(m_motor.setSmartCurrentLimit((int) CURRENT_LIMIT), "Failed to set current limit for NEO");
        m_currentLimitsConfigs.withStatorCurrentLimitEnable(true);
        m_currentLimitsConfigs.withStatorCurrentLimit(CURRENT_LIMIT);
        // Set neutral mode to brake
        m_motorConfigs.withNeutralMode(Brake);
        
       
       
        // adjust CANBus update periods 
        checkNeoError(m_motor.setUpdateFrequency(CANSparkLowLevel.PeriodicFrame.kStatus0, 100),
                "Failed to set periodic status frame 0 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20),
                "Failed to set periodic status frame 1 rate");
        checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20),
                "Failed to set periodic status frame 2 rate");

        

        // Setup encoder
        
        double positionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;
        m_motor.setConversionFactor(positionConversionFactor);
        m_encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
    }

    // set the drive voltage
    public void setReferenceVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    // get the drive velocity
    public StatusSignal<Double> getStateVelocity() {
        return m_motor.getVelocity();
    }

    //get wheel distance        
    public StatusSignal <Double> getWheelDistance(){
        return m_motor.getPosition();//i think these need to be converted but idk what value to use 
    }

}
