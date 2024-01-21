// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//this was taken from the 2023 shoulder code 

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Shooter extends TrapezoidProfileSubsystem {
    private static final double SHOOTER_SHOULDER_MAX_ANGLE = Math.toRadians(30.0);
    private static final double SHOOTER_SHOULDER_MIN_ANGLE = Math.toRadians(-65.0);
    //TODO all constants were taken from the 2023 arm 
    public static final double SHOOTER_SHOULDER_ANGLE_TOLERANCE_RADIAN = Math.toRadians(3.0);

    private static final double CURRENT_LIMIT = 40.0;

    // TODO: The following constants came from the 2022 robot.
    // These need to be set for this robot.

    // All units are MKS with angles in Radians

    // Feedforward constants for the shoulder
    private static final double SHOOTER_SHOULDER_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    private static final double SHOOTER_SHOULDER_KG = 0.09; // V
    private static final double SHOOTER_SHOULDER_KV = 6.60; // V*sec/rad
    private static final double SHOOTER_SHOULDER_KA = 0.01; // V*sec^2/rad
    
  
    // Constants to limit the shooter rotation speed
    private static final double SHOOTER_SHOULDER_MAX_VEL_RADIAN_PER_SEC = Units.degreesToRadians(300.0); // 120 deg/sec
    private static final double SHOOTER_SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ = Units.degreesToRadians(450.0); // 120 deg/sec^2

    private static final double SHOOTER_SHOULDER_POSITION_OFFSET = 62.0/360.0;
    private static final double SHOOTER_SHOULDER_OFFSET_RADIAN = SHOOTER_SHOULDER_POSITION_OFFSET * 2 * Math.PI;

    // The Shoulder gear ratio is 288, but let's get it exactly.
    // private static final double SHOOTER_SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (84.0 / 26.0) * (60.0 / 22.0);
    private static final double SHOOTER_SHOULDER_GEAR_RATIO = (84.0 /12.0) * (84.0 / 18.0) * (70.0 / 40.0) * (60.0 / 22.0);//TODO ask what the ratio is 

    // PID Constants for the shooter PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    private static final double SHOOTER_SHOULDER_K_P = 0.15;
    private static final double SHOOTER_SHOULDER_K_I = 0.0;
    private static final double SHOOTER_SHOULDER_K_D = 0.0;
    private static final double SHOOTER_SHOULDER_K_FF = 0.0;
    private static final int kPIDLoopIdx = 0;
    private static final int kTimeoutMs = 0;

    // The TalonFX, the integrated motor controller for the Falcon, uses ticks as it's noative unit.
    // There are 2048 ticks per revolution. Need to account for the gear ratio.
    private static final double SHOOTER_SHOULDER_RADIAN_PER_UNIT = 2 * Math.PI / (2048.0 * SHOOTER_SHOULDER_GEAR_RATIO);
    private static final String SHOOTER_SHOULDER_CURRENT_LIMIT = null;

    // private final ArmFeedforward m_feedForward = new ArmFeedforward(SHOOTER_SHOULDER_KS, SHOOTER_SHOULDER_KG, SHOOTER_SHOULDER_KV, SHOOTER_SHOULDER_KA);

    // Define the motor and encoders
    private final TalonFX m_rightMotor;
    private final TalonFX m_leftMotor;
    private final TalonFX m_shoulder;
    private final CANSparkMax m_feeder;//this is for the bottom two wheels if it was not obvious s

    DutyCycleEncoder m_dutyEncoder;
    
    // The P gain for the PID controller that drives this shooter.
    private double m_kPShoulder = SHOOTER_SHOULDER_K_P;
    private double m_kFeedForward = SHOOTER_SHOULDER_K_FF;

    private boolean m_resetShoulderPos = false;
    private boolean m_coastMode = false;

    // current goal in radians
    private double m_goal;

    // *********************Simulation Stuff************************
    // private final EncoderSim m_encoderSim;

    // // The arm gearbox represents a gearbox containing two Falcon motors.
    // private final DCMotor m_shooterGearbox = DCMotor.getFalcon500(2);

    // // Simulation classes help us simulate what's going on, including gravity.
    // private static final double SHOOTER_SHOULDER_MASS = 10; // Kilograms
    // private static final double SHOOTER_SHOULDER_LENGTH = Units.inchesToMeters(30);

    // private TalonFXSimCollection m_motorSim;

    // private final SingleJointedArmSim m_shooterSim = new SingleJointedArmSim(m_shooterGearbox, SHOOTER_SHOULDER_GEAR_RATIO,
    //         SingleJointedArmSim.estimateMOI(SHOOTER_SHOULDER_LENGTH, SHOOTER_SHOULDER_MASS), SHOOTER_SHOULDER_LENGTH,
    //         Units.degreesToRadians(-75), Units.degreesToRadians(120), true);

    // // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    // // private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    // private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    // private final MechanismRoot2d m_shooterPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    // private final MechanismLigament2d m_shooterTower = m_shooterPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    // private final MechanismLigament2d m_shooter = m_shooterPivot.append(new MechanismLigament2d("Arm", 30,
    //         Units.radiansToDegrees(m_shooterSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));

    // Construct a new shooter subsystem
    public Shooter(DutyCycleEncoder dutyCycleEncoder) {
        super(new TrapezoidProfile.Constraints(SHOOTER_SHOULDER_MAX_VEL_RADIAN_PER_SEC / SHOOTER_SHOULDER_RADIAN_PER_UNIT,
                SHOOTER_SHOULDER_MAX_ACC_RADIAN_PER_SEC_SQ / SHOOTER_SHOULDER_RADIAN_PER_UNIT),
                (SHOOTER_SHOULDER_OFFSET_RADIAN-dutyCycleEncoder.getDistance() * 2 * Math.PI) / SHOOTER_SHOULDER_RADIAN_PER_UNIT);
       
        TalonFXConfiguration m_configs = new TalonFXConfiguration();
       
        m_leftMotor = new TalonFX(Constants.LEFT_SHOOTER_ID);
        m_rightMotor = new TalonFX(Constants.RIGHT_SHOOTER_ID);
        m_feeder = new CANSparkMax(Constants.SHOOTER_FEEDER_ID, CANSparkLowLevel.MotorType.kBrushless);
        m_shoulder = new TalonFX(Constants.SHOOTER_SHOULDER_ID);
    
        CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
        m_currentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        
        m_configs.CurrentLimits = m_currentLimits;


        m_configs.kF = SHOOTER_SHOULDER_K_FF;
		    m_configs.Slot0.kP = SHOOTER_SHOULDER_K_P;
		    m_configs.Slot0.kI = SHOOTER_SHOULDER_K_I;
		    m_configs.Slot0.kD = SHOOTER_SHOULDER_K_D;

        m_shoulder.getConfigurator().apply(m_configs, kTimeoutMs);

        // limits for motor leader and folower
        // always limit current to the values. Trigger limit = 0 so that it is always enforced.
       

        m_dutyEncoder = dutyCycleEncoder;

        // Encoder distance is in radians
        m_dutyEncoder.setDistancePerRotation(2 * Math.PI);
        m_dutyEncoder.setPositionOffset(SHOOTER_SHOULDER_POSITION_OFFSET);

        double initialAngle = -m_dutyEncoder.getDistance();
        // SmartDashboard.putNumber("shooter/initAngle", Units.radiansToDegrees(initialAngle));

        // Set the motor encoder and Position setpoint to the initialAngle from the absolute encoder
        m_encoder.setIntegratedSensorPosition(initialAngle / SHOOTER_SHOULDER_RADIAN_PER_UNIT, 0);

        // m_shoulder.setSelectedSensorPosition(-m_Duty_Encoder.getDistance());
        // m_shoulder.setSelectedSensorPosition(m_encoder.getIntegratedSensorPosition());
        // m_shoulder.setSelectedSensorPosition(initialAngle / SHOOTER_SHOULDER_RADIAN_PER_UNIT);
        // SmartDashboard.putNumber("shooter/motorLeaderIntegSensPos", m_shoulder.getSelectedSensorPosition());

        // m_Duty_Encoder.setPositionOffset(SHOOTER_SHOULDER_OFFSET_RADIAN);
        // m_motorSim = new TalonFXSimCollection(m_shoulder);
        // m_encoderSim = new TalonFXSimCollection(m_encoder);
        // SmartDashboard.putNumber("shooter/absoluteEncoder", Math.toDegrees(-m_dutyEncoder.getDistance()));
        // SmartDashboard.putNumber("shooter/P Gain", m_kPShoulder);
        // SmartDashboard.putData("shooter Sim", m_mech2d);

        setCoastMode(false);
        SmartDashboard.putBoolean("shooter/coastMode", m_coastMode);
        // SmartDashboard.putNumber("shooter/kFeedForward", m_kFeedForward);

        // m_shoulder.set(ControlMode.Position, )
        // m_shoulder.set(ControlMode.Position, m_encoder.getIntegratedSensorPosition(), DemandType.ArbitraryFeedForward, 0.0);//feedforward/12.0);
        // setAngle(m_encoder.getIntegratedSensorPosition() * SHOOTER_SHOULDER_RADIAN_PER_UNIT);
    }

    // public void simulationPeriodic() {
    //     // First, we set our "inputs" (voltages)
    //     m_shooterSim.setInput(m_shoulder.get() * RobotController.getBatteryVoltage());
    //     // Next, we update it. The standard loop time is 20ms.
    //     m_shooterSim.update(0.020);

    //     // Finally, we set our simulated encoder's readings and simulated battery
    //     // voltage
    //     // m_motorSim.setIntegratedSensorRawPosition(m_armSim.getAngleRads());
    //     // SimBattery estimates loaded battery voltages
    //     RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_shooterSim.getCurrentDrawAmps()));

    //     // Update the Mechanism Arm angle based on the simulated arm angle
    //     m_shooter.setAngle(Units.radiansToDegrees(m_shooterSim.getAngleRads()));
    //     SmartDashboard.putNumber("Simulated Shoulder Angle", Units.radiansToDegrees(m_shooterSim.getAngleRads()));
    //     SmartDashboard.putNumber("Sim Battery voltage", RobotController.getBatteryVoltage());
    // }

    @Override
    public void periodic() {
        // Display current values on the SmartDashboard
        // Add some extra numbers to diagnose the load on the motors
        SmartDashboard.putNumber("shooter/leaderOutput", m_shoulder.get());
        SmartDashboard.putNumber("shooter/encoder", Math.toDegrees(getShoulderAngle()));
        SmartDashboard.putNumber("shooter/encoderSpeed", Math.toDegrees(getSpeed()));
        SmartDashboard.putNumber("shooter/goal", Math.toDegrees(m_goal));
        SmartDashboard.putNumber("shooter/absoluteEncoder", Math.toDegrees(-m_dutyEncoder.getDistance()));
        // SmartDashboard.putBoolean("shooter/m_resetShoulderPos", m_resetShoulderPos)

        m_coastMode = SmartDashboard.getBoolean("shooter/coastMode", m_coastMode);
        if (m_coastMode)
            setCoastMode(m_coastMode);
        
        // Jack: I dont think we need this check because we are only going to set to coast mode during disabled and the motor won't be moved by super.periodic() or useState() anyway
        // And we want the setPoint to follow the current encoder reading in disabled mode
        // if (m_coastMode)
        //     return;

        // Execute the super class periodic method
        super.periodic();

        // Here we can check the SmartDashboard for any updates to the PIC constants.
        // Note that since this is Trapezoidal control, we only need to set P.
        // Each increment will only change the set point position a little bit.
        // checkPIDVal();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Calculate the feedforward from the setPoint
        // double feedforward = m_feedForward.calculate(setPoint.position, setPoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        // The ArmFeedForward computes in radians. We need to convert back to degrees.
        // Remember that the encoder was already set to account for the gear ratios.

        // TODO: if the "12.0" is volts, should use RobotController.getBatteryVoltage()
        if (m_resetShoulderPos) {
            setPoint.position = m_shoulder.getPosition();
            m_resetShoulderPos = false;
        }

        m_kFeedForward = SmartDashboard.getNumber("shooter/kFeadForward", m_kFeedForward);

        m_shoulder.set(ControlMode.Position, setPoint.position, DemandType.ArbitraryFeedForward, m_kFeedForward); //, feedforward / 12.0);
        // SmartDashboard.putNumber("shooter/feedforward", feedforward);
        SmartDashboard.putNumber("shooter/setPoint", Math.toDegrees(setPoint.position * SHOOTER_SHOULDER_RADIAN_PER_UNIT));
        // SmartDashboard.putNumber("shooter/velocity", Units.metersToInches(setPoint.velocity));
    }

    private void checkPIDVal() {
        double p = SmartDashboard.getNumber("shooter/P Gain", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != m_kPShoulder)) {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            configs.Slot0.kP = p;
            m_shoulder.getConfigurator().apply(configs, kTimeoutMs);
            m_kPShoulder = p;
        }
    }

    // return current shoulder angle in radians
    public double getShoulderAngle() {
        // return m_encoder.getIntegratedSensorAbsolutePosition() * SHOOTER_SHOULDER_RADIAN_PER_UNIT;
        return m_shoulder.getPosition() * SHOOTER_SHOULDER_RADIAN_PER_UNIT;
    }

    // return current shooter angular speed in radians/sec
    public double getShoulderSpeed() {
        return m_shoulder.getVelocity() * SHOOTER_SHOULDER_RADIAN_PER_UNIT;
    }
    
    public void setShooterSpeeds(double leftSpeed, double rightSpeed, double feederSpeed){
        m_rightMotor.set(rightSpeed);
        m_leftMotor.set(leftSpeed);
        m_feeder.set(feederSpeed);
    }

    public double[] getShooterSpeeds(){
      double rightSpeed = m_rightMotor.get();
      double leftSpeed = m_leftMotor.get();
      double feederSpeed = m_feeder.get();
      //idk if we do this but this could be checked to make sure the speeds != null
      return new double[] {rightSpeed, leftSpeed, feederSpeed};
    }

    public void resetShoulderPos() {
        setAngle(getShoulderAngle());
        m_resetShoulderPos = true;
    }

    // needs to be public so that commands can get the restricted angle
    public static double limitShoulderAngle(double angle){
        return Math.min(SHOOTER_SHOULDER_MAX_ANGLE, Math.max(SHOOTER_SHOULDER_MIN_ANGLE, angle));
    }

    // set shooter angle in radians
    public void setAngle(double angle) {
        m_goal = limitShoulderAngle(angle);
        super.setGoal(m_goal / SHOOTER_SHOULDER_RADIAN_PER_UNIT);
    }
    public void resetGoal(){
        // m_encoder.setIntegratedSensorPosition(getAngle()/SHOOTER_SHOULDER_RADIAN_PER_UNIT, kTimeoutMs);
        setAngle(getShoulderAngle());
        // super.disable();
        // m_superEnabled = false;
    }

    public void setCoastMode(boolean coastMode){
        if (coastMode) {
            m_shoulder.setNeutralMode(NeutralModeValue.Coast);
            m_shoulder.stopMotor();
        } else
            m_shoulder.setNeutralMode(NeutralModeValue.Brake);
    }
}