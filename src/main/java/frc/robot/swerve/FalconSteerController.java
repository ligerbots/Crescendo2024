package frc.robot.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconSteerController implements SteerController {
    private static final double TWO_PI = 2 * Math.PI;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private final TalonFX m_motor;
    private final CanCoderWrapper m_encoder;

    private double m_referenceAngleRadians = 0;
    private double m_resetIteration = 0;

    public FalconSteerController(int id, int canCoderId, double angleOffset) {
        m_motor = new TalonFX(id);
        m_encoder = new CanCoderWrapper(canCoderId, angleOffset);
    }

    @Override
    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }
    @Override
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_motor.getPosition().getValueAsDouble();

        // force into 0 -> 2*PI
        double currentAngleRadiansMod = currentAngleRadians % TWO_PI;
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += TWO_PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= TWO_PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += TWO_PI;
        }

        m_referenceAngleRadians = referenceAngleRadians;

        setReference(adjustedReferenceAngleRadians);
    }

    @Override
    public void syncAngleEncoders(boolean dontCheckTimer) {
        if (dontCheckTimer) {
            // System.out.println("** Synchronizing swerve angle encoders");
            m_motor.setPosition(m_encoder.getAbsoluteAngleRadians());
            m_resetIteration = 0;
            return;
        }

        if (m_motor.getVelocity().getValueAsDouble() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++m_resetIteration >= ENCODER_RESET_ITERATIONS) {
                // System.out.println("** Synchronizing swerve angle encoders");
                m_motor.setPosition(m_encoder.getAbsoluteAngleRadians());
                m_resetIteration = 0;
            }
        } else {
            m_resetIteration = 0;
        }
    }
    @Override
    public Rotation2d getStateAngle() {
        double motorAngleRadians = m_motor.getPosition().getValueAsDouble();
        motorAngleRadians %= TWO_PI;
        if (motorAngleRadians < 0.0) motorAngleRadians += TWO_PI;
        return Rotation2d.fromRadians(motorAngleRadians);
    }

    @Override
    public void updateSmartDashboard(String sdPrefix) {
        SmartDashboard.putNumber(sdPrefix + "/angle", getStateAngle().getDegrees());
        SmartDashboard.putNumber(sdPrefix + "/cancoder", Math.toDegrees(m_encoder.getAbsoluteAngleRadians()));

        // Compute the calibration angle for this module
        // Only use the value if the wheels are physically aligned forward, with bevel gear on the left
        // NOTE: we want a negative angle, -2PI -> 0
        double calibAngle = m_encoder.getOffsetAngleRadians() - m_encoder.getAbsoluteAngleRadians();
        if (calibAngle > 0.0) calibAngle -= TWO_PI;
        if (calibAngle < -TWO_PI) calibAngle += TWO_PI;
        SmartDashboard.putNumber(sdPrefix + "/calibrationAngle", Math.toDegrees(calibAngle));

        double offset = getStateAngle().getRadians() - m_encoder.getAbsoluteAngleRadians();
        if (offset > Math.PI) offset -= TWO_PI;
        if (offset < -Math.PI) offset += TWO_PI;
        SmartDashboard.putNumber(sdPrefix + "/cancoder_offset", Math.toDegrees(offset));
    }

    private void setReference(double angle) {
        // TODO Figure this out, if possible
        // Should set the reference angle for some kind of PID controller - something like SparkPIDController.setReference
    }
}
