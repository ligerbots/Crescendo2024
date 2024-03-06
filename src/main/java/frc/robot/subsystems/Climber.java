package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive;

public class Climber extends SubsystemBase {

    CANSparkMax m_rightWinch, m_leftWinch;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;
    private double m_leftPosition;
    private double m_rightPosition;
    private double m_leftVelocity;
    private double m_rightVelocity;
    private double m_leftEngagedPosition;
    private double m_rightEngagedPosition;
    private boolean m_leftHookReadyToEngage = false;
    private boolean m_rightHookReadyToEngage = false;
    private boolean m_leftHookEngaged = false;
    private boolean m_rightHookEngaged = false;
    private boolean m_leftHookComplete = false;
    private boolean m_rightHookComplete = false;
    private double m_rollAngle;

    // Constants to be used in this class
    private static final double WINCH_GEAR_RATIO = 1.0/15.0;
    private static final double NOMINAL_WINCH_DIAMETER = Units.inchesToMeters(0.75);
    private static final double NOMINAL_INCHES_PER_ROTATION = Math.PI * NOMINAL_WINCH_DIAMETER * WINCH_GEAR_RATIO;
    private static final double HOOK_DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION = 20.0 / NOMINAL_INCHES_PER_ROTATION;
    private static final double CLIMB_ROTATIONS_ABOVE_FLOOR = 6.0 / NOMINAL_INCHES_PER_ROTATION;
    private static final double MAX_WINCH_ROTATIONS_ALLOWED = HOOK_DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION + CLIMB_ROTATIONS_ABOVE_FLOOR;

    // Winch motor speed values
    private static final double IDLE_MOTOR_SPEED = -0.01;
    private static final double WINCH_EXTEND_SPEED = 0.5;
    private static final double WINCH_RETRACT_SPEED = 0.5;
    private static final double WINCH_CLIMB_SPEED = 0.5;
    private static final double WINCH_CLIMB_ADJUST_SPEED = 0.2;
    private static final double ROLL_ANGLE_TOLERANCE = Units.degreesToRadians(2.0);
    private static final double ROLL_ANGLE_EMERGENCY_STOP = Units.degreesToRadians(5.0);
    
    // State definitions:
    // IDLE - Winches holding hooks in place, Should be used for entire match until End Game.
    // EXTENDING_HOOKS - Winches unwinding ropes to allow hooks to raise.
    // WAITING - Hooks at max height. Waiting dor robot to get close enough to chain to lower hooks.
    // RETRACTING_HOOKS - Winches start winding up the rope to lower the hooks onto the chain.
    // CLIMBING - Hooks have engaged with chain. As the winches retract further, the robot goes up.
    //      Since the robot is no longer onthe ground, we need to adjust the climb speeds to keep the robot level.
    // HOLDING - Normally, this would mean the the robot is now off the floor and the ratchets are engaged.
    //      In an emergency, if the robot rools too far, we will stop the motors and let the ratchets hold the robot.
    //      In this state, the runWinch() operation can allow further adjustments as needed.
    private enum ClimberState {IDLE, EXTENDING_HOOKS, WAITING, RETRACTING_HOOKS, CLIMBING, HOLDING};
    private ClimberState m_climberState = ClimberState.IDLE;

    private RobotContainer m_robotContainer;
    private DriveTrain m_driveTrain;

    
    public Climber() {
        m_rightWinch = new CANSparkMax(Constants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);
        m_leftWinch = new CANSparkMax(Constants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);

        m_leftEncoder = m_leftWinch.getEncoder();
        m_rightEncoder = m_rightWinch.getEncoder();

        // Let's keep everything in rotations
        // m_leftEncoder.setPositionConversionFactor(WINCH_GEAR_RATIO*Math.PI*NOMINAL_WINCH_DIAMETER);
        // m_rightEncoder.setPositionConversionFactor(WINCH_GEAR_RATIO*Math.PI*NOMINAL_WINCH_DIAMETER);

        m_leftWinch.restoreFactoryDefaults();
        // TODO only one will be inverted, not sure which
        m_leftWinch.setInverted(true);
        m_leftWinch.setIdleMode(IdleMode.kBrake);
        // Reset position to 0
        m_leftEncoder.setPosition(0.0);

        m_rightWinch.restoreFactoryDefaults();
        m_rightWinch.setInverted(false);
        m_rightWinch.setIdleMode(IdleMode.kBrake);
        // Reset position to 0
        m_rightEncoder.setPosition(0.0);

        // Need the DriveTrain to get the roll angle of the robot.
        m_driveTrain = m_robotContainer.getDriveTrain();
    }

    @Override
    public void periodic() {
        m_leftPosition = m_leftEncoder.getPosition();
        m_rightPosition = m_rightEncoder.getPosition();
        m_leftVelocity = m_leftEncoder.getVelocity();
        m_rightVelocity = m_rightEncoder.getVelocity();
        m_rollAngle = m_driveTrain.getRoll().getRadians();

        SmartDashboard.putNumber("climber/leftSpeed", m_leftVelocity);
        SmartDashboard.putNumber("climber/rightSpeed", m_rightVelocity);

        SmartDashboard.putNumber("climber/leftPosition", m_leftPosition);
        SmartDashboard.putNumber("climber/rightPosition", m_leftVelocity);

        // Always check if we're too far off balance, but only while climbing.
        // If the robot tips while driving, don't worry about it.
        if (m_climberState != ClimberState.IDLE && Math.abs(m_rollAngle) > ROLL_ANGLE_EMERGENCY_STOP) {
            m_climberState = ClimberState.HOLDING;
            m_leftWinch.set(0.0);
            m_rightWinch.set(0.0);
        }

        // Wile idle, we want a small volatage applied to hold the hooks in place.
        if (m_climberState == ClimberState.IDLE) {
            m_leftWinch.set(IDLE_MOTOR_SPEED);
            m_rightWinch.set(IDLE_MOTOR_SPEED);
            // Note that the EXTENDING_HOOKS state is entered via a command, so here we just stay IDLE
        }
        else if (m_climberState == ClimberState.EXTENDING_HOOKS) {
            // If the left hook is all the way out...
            if (m_leftPosition > HOOK_DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION) {
                // Stop left winch
                m_leftWinch.set(0.0);
                m_leftHookReadyToEngage = true;
            }
            // If the right hook is all the way out...
            if (m_rightPosition > HOOK_DEPLOYED_ROTATIONS_ABOVE_INITIAL_POSITION) {
                // Stop right winch
                m_rightWinch.set(0.0);
                m_rightHookReadyToEngage = true;
            }
            // If both hooks are all the way up, wait to retract
            if (m_leftHookReadyToEngage && m_rightHookReadyToEngage) {
                m_climberState = ClimberState.WAITING;
            }
        }
        else if (m_climberState == ClimberState.WAITING) {
            // Nothing to do here since both winches are stopped and the hooks are as high as they can go.
            // The RETRACTING_HOOKS state is only entered via a command
        }
        else if (m_climberState == ClimberState.RETRACTING_HOOKS) {
            // Here we retract both hooks. When one hook grabs the chain before the other, the robot will start to roll
            // Roll angle is positive if the left side is higher than the right side.
            // Once one side goes high, we need to stop that winch until the other hook "catches up"
            // Note that when we enetered this state, both winches started retracting. We only need to stop them one at a time until
            // the robot is level again and then we will go to the CLIMBING state
            if (m_rollAngle > ROLL_ANGLE_TOLERANCE) {
                // The left side is high, so the left hook is engaged.
                   // Stop the left motor. The ratchet wrench will hold it.
                   m_leftWinch.set(0.0);
                   m_leftHookEngaged = true;
                   m_leftEngagedPosition = m_leftPosition;
            }
            else if (m_rollAngle < -ROLL_ANGLE_TOLERANCE) {
                // The right side is high, so the right hook is engaged.
                   // Stop the right motor. The ratchet wrench will hold it.
                   m_rightWinch.set(0.0);
                   m_rightHookEngaged = true;
                   m_rightEngagedPosition = m_rightPosition;
            }
            // If both hooks are engaged, then we start climbing
            if (m_leftHookEngaged && m_rightHookEngaged) {
                m_climberState = ClimberState.CLIMBING;
                m_leftWinch.set(WINCH_CLIMB_SPEED);
                m_rightWinch.set(WINCH_CLIMB_SPEED);
                
            }
        }
        else if (m_climberState == ClimberState.CLIMBING) {
            // If we climbed far enough, stop the winches and let the ratchets hold the robot.
            if (m_leftPosition - m_leftEngagedPosition > CLIMB_ROTATIONS_ABOVE_FLOOR) {
                m_leftWinch.set(0.0);
                m_leftHookEngaged = true;
            }
            if (m_rightPosition - m_rightEngagedPosition > CLIMB_ROTATIONS_ABOVE_FLOOR) {
                m_rightWinch.set(0.0);
                m_rightHookEngaged = true;
            }
            // If both hooks are in enough, just hold where we are.
            if (m_leftHookEngaged && m_rightHookEngaged) {
                m_climberState = ClimberState.HOLDING;
            }
            else {
                // Need to keep the robot level. We defiend a voltage for a decent climbing rate.
                // Keep the left side at that rate and adjust the right side based on the roll angle
                if (Math.abs(m_rollAngle) > ROLL_ANGLE_TOLERANCE) {
                    m_rightWinch.set(WINCH_CLIMB_SPEED + 
                        Math.signum(m_rollAngle) *
                        Math.min(Math.abs(m_rollAngle)/ROLL_ANGLE_TOLERANCE, 1.0) * WINCH_CLIMB_ADJUST_SPEED);
                }
            }
        }
    }

    private double limitWinch(RelativeEncoder encoder, double speed) {
        if (encoder.getPosition() >= MAX_WINCH_ROTATIONS_ALLOWED) {
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

    public void extendHooks() {
        m_climberState = ClimberState.EXTENDING_HOOKS;
        m_leftWinch.set(WINCH_EXTEND_SPEED);
        m_rightWinch.set(WINCH_EXTEND_SPEED);
    }

    public void retractHooks() {
        m_climberState = ClimberState.RETRACTING_HOOKS;
        m_leftWinch.set(WINCH_RETRACT_SPEED);
        m_rightWinch.set(WINCH_RETRACT_SPEED);
    }

    public void holdHooks() {
        m_climberState = ClimberState.HOLDING;
        m_leftWinch.set(0.0);
        m_rightWinch.set(0.0);
    }
}
