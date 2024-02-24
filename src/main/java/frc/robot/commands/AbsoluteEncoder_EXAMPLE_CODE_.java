package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class AbsoluteEncoder_EXAMPLE_CODE_ {
    final CANSparkMax m_motor;
    final AbsoluteEncoder m_encoder;

    public AbsoluteEncoder_EXAMPLE_CODE_(CANSparkMax motor) {
        super();
        m_motor = motor;
        m_encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    }
}
