package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    /*
    C = center
    S = stage
    M = Midline
     */
    private final double NOTE_C_X = 8.28;
    private final double NOTE_S_X = 2.89;
    private final double NOTE_CHECK_X = 6.85;

    public final Pose2d NOTE_C_1 = new Pose2d(8.28, 0.78, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_2 = new Pose2d(8.28, 2.44, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_3 = new Pose2d(8.28, 4.10, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_4 = new Pose2d(8.28, 5.77, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_5 = new Pose2d(8.28, 7.45, Rotation2d.fromDegrees(0));

    public final Pose2d NOTE_S_1 = new Pose2d(2.89, 4.26, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_S_2 = new Pose2d(2.89, 5.52, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_S_3 = new Pose2d(2.89, 7.00, Rotation2d.fromDegrees(0));

   
   
   
   
    public final Pose2d ROBOT_NOTE_C_1 = new Pose2d(7.85, 0.78, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_C_2 = new Pose2d(7.85, 2.44, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_C_3 = new Pose2d(7.85, 4.10, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_C_4 = new Pose2d(7.85, 5.77, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_C_5 = new Pose2d(7.85, 7.45, Rotation2d.fromDegrees(0));

    public final Pose2d ROBOT_NOTE_S_1 = new Pose2d(2.49, 4.26, Rotation2d.fromDegrees(157.45));
    public final Pose2d ROBOT_NOTE_S_2 = new Pose2d(2.43, 5.55, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_S_3 = new Pose2d(2.49, 6.81, Rotation2d.fromDegrees(161.94));

    
    //these are all placeholders and subject to change 
    public final Pose2d ROBOT_SHOOT_M_1 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_SHOOT_M_2 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));

    public final Pose2d ROBOT_NOTE_CHECK_1 = new Pose2d(6.85, 0.78,Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_CHECK_2 = new Pose2d(6.85, 2.44,Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_CHECK_3 = new Pose2d(6.85, 5.77, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_NOTE_CHECK_4 = new Pose2d(6.85, 7.45, Rotation2d.fromDegrees(0));



    public final Pose2d ROBOT_START_1 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_START_2 = new Pose2d(1.3, 5.53, Rotation2d.fromDegrees(0));
    public final Pose2d ROBOT_START_3 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));

}
