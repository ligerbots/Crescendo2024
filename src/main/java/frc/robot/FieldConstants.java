package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    /*
    C = center
    S = stage
    M = Midline
     */
    private static final double NOTE_C_X = 7.85;
    private static final double NOTE_S_X = 2.45;
    private static final double NOTECHECK_X = NOTE_C_X - 1;


    public static final Pose2d NOTE_C_1 = new Pose2d(NOTE_C_X, 0.78, Rotation2d.fromDegrees(0));
    public static final Pose2d NOTE_C_2 = new Pose2d(NOTE_C_X, 2.44, Rotation2d.fromDegrees(0));
    public static final Pose2d NOTE_C_3 = new Pose2d(NOTE_C_X, 4.10, Rotation2d.fromDegrees(0));
    public static final Pose2d NOTE_C_4 = new Pose2d(NOTE_C_X, 5.77, Rotation2d.fromDegrees(0));
    public static final Pose2d NOTE_C_5 = new Pose2d(NOTE_C_X, 7.45, Rotation2d.fromDegrees(0));

    public static final Pose2d NOTE_S_1 = new Pose2d(2.49, 4.26, Rotation2d.fromDegrees(157.45));
    public static final Pose2d NOTE_S_2 = new Pose2d(2.43, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d NOTE_S_3 = new Pose2d(2.49, 6.81, Rotation2d.fromDegrees(161.94));

    
    //these are all placeholders and subject to change 
    public static final Pose2d SHOOT_M_1 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public static final Pose2d SHOOT_M_2 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));

    public static final Pose2d NOTECHECK_1 = new Pose2d(NOTECHECK_X, 0.78,Rotation2d.fromDegrees(0));
    public static final Pose2d NOTECHECK_2 = new Pose2d(NOTECHECK_X, 2.44,Rotation2d.fromDegrees(0));
    public static final Pose2d NOTECHECK_3 = new Pose2d(NOTECHECK_X, 5.77, Rotation2d.fromDegrees(0));
    public static final Pose2d NOTECHECK_4 = new Pose2d(NOTECHECK_X, 7.45, Rotation2d.fromDegrees(0));



    public static final Pose2d START_1 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public static final Pose2d START_2 = new Pose2d(1.3, 5.53, Rotation2d.fromDegrees(0));
    public static final Pose2d START_3 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));

}
