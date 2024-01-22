package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public class PairReturn {
    private double returnedDouble;
    private ArrayList<Pose2d> reternedPose2d;

    public PairReturn(double returnedDouble, ArrayList<Pose2d> returnedPose2d){
        this.returnedDouble = returnedDouble;
        this.reternedPose2d = returnedPose2d;
    }

    public double getDouble(){
        return returnedDouble;
    }

    public ArrayList<Pose2d> getPose2d(){
        return reternedPose2d;
    }
}   
