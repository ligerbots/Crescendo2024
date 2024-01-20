package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;

// A wrapper around the CANCoder absolute angle sensor

public class CanCoderWrapper {
    private static final double UPDATE_FREQUENCY_HZ = 10.0;
    private static final boolean ROTATION_CLOCKWISE = false;

    private final CANcoder m_encoder;

    // remember the offsetAngle to simplify recalibration of the offset
    private final double m_offsetAngleRadians;

    public static void checkCtreError(StatusCode errorCode, String message) {
        if (errorCode != StatusCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
            // System.out.println("** ERROR in config of CANCoder: " + errorCode.toString());
        }
    }

    public CanCoderWrapper(int canId, double offsetRadians) {
        m_encoder = new CANcoder(canId);
        m_offsetAngleRadians = offsetRadians;

        CANcoderConfigurator config = m_encoder.getConfigurator();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        magConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magConfig.MagnetOffset = offsetRadians / (2*Math.PI);
        if (ROTATION_CLOCKWISE)
            magConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        else
            magConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        checkCtreError(config.apply(magConfig), "Failed to configure CANCoder");

        // set the update period and report any errors
        checkCtreError(m_encoder.getAbsolutePosition().setUpdateFrequency(UPDATE_FREQUENCY_HZ, 250),
                "Failed to configure CANCoder update rate");
    };

    public double getOffsetAngleRadians() {
        return m_offsetAngleRadians;
    }

    // get the absolute angle, in radians
    public double getAbsoluteAngleRadians() {
        double angle = 2 * Math.PI * m_encoder.getAbsolutePosition().getValue();
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}
