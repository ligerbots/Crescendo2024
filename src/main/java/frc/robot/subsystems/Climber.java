package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {

    CANSparkMax m_rightWinch, m_leftWinch;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private static final double WINCH_GEAR_RATIO = 1.0;  // TODO find correct numbers
    private static final double MAX_WINCH_POSITION = 1.0;

    public Climber() {
        m_rightWinch = new CANSparkMax(Constants.CLIMBER_RIGHT_CAN_ID, MotorType.kBrushless);
        m_leftWinch = new CANSparkMax(Constants.CLIMBER_LEFT_CAN_ID, MotorType.kBrushless);

        m_leftEncoder = m_leftWinch.getEncoder();
        m_rightEncoder = m_rightWinch.getEncoder();

        m_leftEncoder.setPositionConversionFactor(WINCH_GEAR_RATIO);
        m_rightEncoder.setPositionConversionFactor(WINCH_GEAR_RATIO);

        m_leftWinch.restoreFactoryDefaults();
        // TODO only one will be inverted, not sure which
        m_leftWinch.setInverted(true);
        m_leftWinch.setIdleMode(IdleMode.kBrake);

        m_rightWinch.restoreFactoryDefaults();
        m_rightWinch.setInverted(true);
        m_rightWinch.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/leftSpeed", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("climber/rightSpeed", m_rightEncoder.getVelocity());

        SmartDashboard.putNumber("climber/leftPosition", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("climber/rightPosition", m_rightEncoder.getPosition());
    }

    private double limitWinch(RelativeEncoder encoder, double speed) {
        if (encoder.getPosition() >= MAX_WINCH_POSITION) {
            return 0.0;
        } else
            return speed;
    }

    public void run(double leftSpeed, double rightSpeed) {
        m_leftWinch.set(limitWinch(m_leftEncoder, leftSpeed));
        m_rightWinch.set(limitWinch(m_leftEncoder, rightSpeed));    
    }

    public double getRightPosition(){
        return m_leftEncoder.getPosition();
    }

    public double getLeftPosition(){
        return m_leftEncoder.getPosition();
    }

    public double getRightVelocity(){
        return m_rightEncoder.getVelocity();
    }

    public double getLeftVelocity(){
        return m_leftEncoder.getVelocity();
    }
}
