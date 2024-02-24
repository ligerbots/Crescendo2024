package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

    public static final double FIELD_LENGTH = 16.54;
    public static final double FIELD_WIDTH = 8.21;

    public static final double BLUE_WHITE_LINE_X_METERS = 1.93294;
    public static final double BLUE_WING_LINE_X_METERS = 5.87248;

    // C = center
    // S = stage
    // M = Midline

    private static final double NOTE_C_X = FIELD_LENGTH / 2.0;
    private static final double NOTE_S_X = 2.89;

    public static final Translation2d NOTE_C_1 = new Translation2d(NOTE_C_X, 0.78);
    public static final Translation2d NOTE_C_2 = new Translation2d(NOTE_C_X, 2.44);
    public static final Translation2d NOTE_C_3 = new Translation2d(NOTE_C_X, 4.10);
    public static final Translation2d NOTE_C_4 = new Translation2d(NOTE_C_X, 5.77);
    public static final Translation2d NOTE_C_5 = new Translation2d(NOTE_C_X, 7.45);

    public static final Translation2d BLUE_NOTE_S_1 = new Translation2d(NOTE_S_X, 4.26);
    public static final Translation2d BLUE_NOTE_S_2 = new Translation2d(NOTE_S_X, 5.52);
    public static final Translation2d BLUE_NOTE_S_3 = new Translation2d(NOTE_S_X, 7.00);   
   
    // Needed for the Note simulation. 
    // For Autos, always use BLUE and flipTranslation
    public static final Translation2d RED_NOTE_S_1 = new Translation2d(FIELD_LENGTH - NOTE_S_X, 4.26);
    public static final Translation2d RED_NOTE_S_2 = new Translation2d(FIELD_LENGTH - NOTE_S_X, 5.52);
    public static final Translation2d RED_NOTE_S_3 = new Translation2d(FIELD_LENGTH - NOTE_S_X, 7.00);   
   

    public static final Translation2d SPEAKER = new Translation2d(0, 5.54);

    public static final Pose2d ROBOT_NOTE_C_1 = new Pose2d(7.85, 0.78, Rotation2d.fromDegrees(0));
    public static final Pose2d ROBOT_NOTE_C_2 = new Pose2d(7.85, 2.44, Rotation2d.fromDegrees(0));
    public static final Pose2d ROBOT_NOTE_C_3 = new Pose2d(7.85, 4.10, Rotation2d.fromDegrees(0));
    public static final Pose2d ROBOT_NOTE_C_4 = new Pose2d(7.85, 5.77, Rotation2d.fromDegrees(0));
    public static final Pose2d ROBOT_NOTE_C_5 = new Pose2d(7.85, 7.45, Rotation2d.fromDegrees(0));

    public static final Pose2d ROBOT_NOTE_S_1 = new Pose2d(2.49, 4.26, Rotation2d.fromDegrees(157.45));
    public static final Pose2d ROBOT_NOTE_S_2 = new Pose2d(2.43, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d ROBOT_NOTE_S_3 = new Pose2d(2.49, 6.81, Rotation2d.fromDegrees(161.94));

    // these are all placeholders and subject to change
    public static final Pose2d ROBOT_SHOOT_M_1 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d ROBOT_SHOOT_M_2 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static final Pose2d ROBOT_START_1 = new Pose2d(1.25, 3.85, Rotation2d.fromDegrees(126.5));
    public static final Pose2d ROBOT_START_2 = new Pose2d(1.3, 5.53, Rotation2d.fromDegrees(180));
    public static final Pose2d ROBOT_START_3 = new Pose2d(1.25,6.95, Rotation2d.fromDegrees(-131.6));  // same as 228.4

    public static boolean isCenterNote(Translation2d targetNote) {
        return Math.abs(NOTE_C_X - targetNote.getX()) < 0.1;
    }

    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static Pose2d flipPose(Pose2d pose) {
        // flip pose when red
        if (isRedAlliance()) {
            Rotation2d rot = pose.getRotation();
            // reflect the pose over center line, flip both the X and the rotation
            return new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), new Rotation2d(-rot.getCos(), rot.getSin()));
        }

        // Blue or we don't know; return the original pose
        return pose;
    }

    public static Translation2d flipTranslation(Translation2d position) {
        // flip when red
        if (isRedAlliance()) {
            // reflect the pose over center line, flip both the X
            return new Translation2d(FIELD_LENGTH - position.getX(), position.getY());
        }

        // Blue or we don't know; return the original position
        return position;
    }
}