package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class TripleReturn {
    private double returnedDouble;
    private ArrayList<Pose2d> reternedPose2d;
    private Optional<EstimatedRobotPose> returnedEstimatedRobotPose;

    public TripleReturn(double returnedDouble, ArrayList<Pose2d> returnedPose2d, Optional<EstimatedRobotPose> returnedEstimatedRobotPose){
        this.returnedDouble = returnedDouble;
        this.reternedPose2d = returnedPose2d;
        if (returnedEstimatedRobotPose.isPresent()){
            this.returnedEstimatedRobotPose = returnedEstimatedRobotPose;
        }
\
    }

    public double getDouble(){
        return returnedDouble;
    }

    public ArrayList<Pose2d> getPose2d(){
        return reternedPose2d;
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(){
        return returnedEstimatedRobotPose;
    }
}   
