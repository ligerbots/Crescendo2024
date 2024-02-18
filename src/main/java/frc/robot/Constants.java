package frc.robot;

public class Constants {
    public static double MAX_VOLTAGE = 12.0;

    // Feature flag: enable simulation in the classes
    // Can turn this off for competition to save a tiny bit of speed
    public static final boolean SIMULATION_SUPPORT = true;

    // Reminder: all CAN IDs of the same device/motor type need to be unique.
    // You don't actually need SparkMax IDs to be different from Falcon IDs

    // CAN IDs and swerve angle offsets for the drivetrain
    // These controllers are all SparkMaxes, so need to be unique
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(208.4);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(91.2);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(358.12);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(232.0);

    // Elevator
    public static final int ELEVATOR_CAN_ID = 18; //TODO: REPLACE WITH REAL CAN ID

    public static final int INTAKE_MOTOR_CAN_ID = 5;
    public static final int CENTERING_MOTOR_CAN_ID = 6; 

    public static final int SHOOTER_PIVOT_CAN_ID = 17; //TODO: REPLACE WITH REAL ID

    public static final int LEFT_SHOOTER_CAN_ID = 20;
    public static final int RIGHT_SHOOTER_CAN_ID = 21;
    public static final int FEEDER_CAN_ID = 22;
}
