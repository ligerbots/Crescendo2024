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

    private static final double WINCH_GEAR_RATIO = 1.0;

    public Climber() {
        m_rightWinch = new CANSparkMax(Constants.CLIMBER_RIGHT_CAN_ID, MotorType.kBrushless);
        m_leftWinch = new CANSparkMax(Constants.CLIMBER_LEFT_CAN_ID, MotorType.kBrushless);

        m_leftEncoder = m_leftWinch.getEncoder();
        m_rightEncoder = m_rightWinch.getEncoder();

        m_leftEncoder.setPositionConversionFactor(WINCH_GEAR_RATIO);
        m_rightEncoder.setPositionConversionFactor(WINCH_GEAR_RATIO);


        m_leftWinch.restoreFactoryDefaults();
        m_leftWinch.setInverted(true);
        m_leftWinch.setIdleMode(IdleMode.kBrake);
        m_leftWinch.getEncoder();

        m_rightWinch.restoreFactoryDefaults();
        m_rightWinch.setInverted(true);
        m_rightWinch.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Winch speed", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Winch speed", m_rightEncoder.getVelocity());

        SmartDashboard.putNumber("Left Winch position", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Winch position", m_rightEncoder.getPosition());


    }

    public void run(double rightSpeed, double leftSpeed) {
        m_rightWinch.set(rightSpeed);
        m_leftWinch.set(leftSpeed);
    }

    public void runLeft(double speed) {
        m_leftWinch.set(speed);
    }

    public void runRight(double speed) {
        m_rightWinch.set(speed);
    }
}
