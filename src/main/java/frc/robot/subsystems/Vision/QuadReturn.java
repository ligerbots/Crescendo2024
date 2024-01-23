package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;

public class QuadReturn {
    private double returnedDouble;
    private ArrayList<Pose2d> returnedPose2d;
    private Optional<EstimatedRobotPose> returnedEstimatedRobotPose;
    private PhotonPipelineResult returnedPhotonPipelineResult;

    public QuadReturn(double returnedDouble, ArrayList<Pose2d> returnedPose2d, Optional<EstimatedRobotPose> returnedEstimatedRobotPose, PhotonPipelineResult returnedPhotonPipelineResult){
        this.returnedDouble = returnedDouble;
        this.returnedPose2d = returnedPose2d;
        if (returnedEstimatedRobotPose.isPresent()){
            this.returnedEstimatedRobotPose = returnedEstimatedRobotPose;
        }
        this.returnedPhotonPipelineResult = returnedPhotonPipelineResult;
    }

    public double getTimestamp(){
        return returnedDouble;
    }

    public ArrayList<Pose2d> getTargets(){
        return returnedPose2d;
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(){
        return returnedEstimatedRobotPose;
    }

    public PhotonPipelineResult getPhotonPipeline(){
        return returnedPhotonPipelineResult;
    }
}   
