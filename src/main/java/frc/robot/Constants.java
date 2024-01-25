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
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(88.4);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(359.5);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(65.5);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(108.5);

    public static final int INTAKE_MOTOR_CAN_ID = 15; //TODO: 1 IS A PLACEHOLDER, REPLACE IT
    public static final int CENTERING_MOTOR_CAN_ID = 16; //TODO: 1 IS A PLACEHOLDER, REPLACE IT

    public static final int SHOOTER_PIVOT_CAN_ID = 17; //TODO: REPLACE WITH REAL ID

}
