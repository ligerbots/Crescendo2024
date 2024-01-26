package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    /*
    C = center
    S = stage
    M = Midline
     */
    private final double Note_C_X = 7.85;
    private final double Note_S_X = 2.45;
    private final double NoteCheck_X = Note_C_X - 1;


    public final Pose2d NOTE_C_1 = new Pose2d(Note_C_X, 0.78, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_2 = new Pose2d(Note_C_X, 2.44, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_3 = new Pose2d(Note_C_X, 4.10, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_4 = new Pose2d(Note_C_X, 5.77, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_C_5 = new Pose2d(Note_C_X, 7.45, Rotation2d.fromDegrees(0));

    public final Pose2d NOTE_S_1 = new Pose2d(Note_S_X, 4.11, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_S_2 = new Pose2d(Note_S_X, 5.57, Rotation2d.fromDegrees(0));
    public final Pose2d NOTE_S_3 = new Pose2d(Note_S_X, 7.00, Rotation2d.fromDegrees(0));

    //these are all placeholders and subject to change 
    public final Pose2d SHOOT_M_1 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public final Pose2d SHOOT_M_2 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));

    public final Pose2d NoteCheck_1 = new Pose2d(NoteCheck_X, 0.78,Rotation2d.fromDegrees(0));
    public final Pose2d NoteCheck_2 = new Pose2d(NoteCheck_X, 2.44,Rotation2d.fromDegrees(0));
    public final Pose2d NoteCheck_3 = new Pose2d(NoteCheck_X, 5.77, Rotation2d.fromDegrees(0));
    public final Pose2d NoteCheck_4 = new Pose2d(NoteCheck_X, 7.45, Rotation2d.fromDegrees(0));



    public final Pose2d START_1 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public final Pose2d START_2 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));
    public final Pose2d START_3 = new Pose2d(0, 0 ,Rotation2d.fromDegrees(0));

}
