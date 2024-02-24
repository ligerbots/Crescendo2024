package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class AbsoluteEncoder_EXAMPLE_CODE_ {
    final CANSparkMax m_motor;
    final AbsoluteEncoder m_absoluteEncoder;
    final RelativeEncoder m_relativeEncoder;

    public AbsoluteEncoder_EXAMPLE_CODE_(CANSparkMax motor) {
        super();
        m_motor = motor;
        // A lot of functions are the same for both types
        m_absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        m_relativeEncoder = motor.getEncoder();
    }
}
