package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {

    CANSparkMax m_rightWinch;
    CANSparkMax m_leftWinch;

    public Climber() {
        m_rightWinch = new CANSparkMax(Constants.CLIMBER_RIGHT_CAN_ID, MotorType.kBrushless);
        m_leftWinch = new CANSparkMax(Constants.CLIMBER_RIGHT_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
    }

    public void run(double rightSpeed, double leftSpeed) {
        m_rightWinch.set(rightSpeed);
        m_leftWinch.set(leftSpeed);
    }

    public void runLeft(double speed) {
        m_leftWinch.set(speed);
    }

    public void runRight(double speed) {
        m_leftWinch.set(speed);
    }
}
