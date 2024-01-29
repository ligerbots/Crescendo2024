// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AprilTagVision {
    // variable to turn on/off our private tag layout
    // if this is false, the compiler should remove all the unused code.
    public static final boolean USE_PRIVATE_TAG_LAYOUT = false;

    // Use the multitag pose estimator
    public static final boolean USE_MULTITAG = true;

    // Plot vision solutions
    public static final boolean PLOT_TAG_SOLUTIONS = true;

    // constants for extra tags in the shed lengths in meters!!)
    static final double SHED_TAG_NODE_XOFFSET = 0.45;
    static final double SHED_TAG_NODE_ZOFFSET = 0.31;
    static final double SHED_TAG_SUBSTATION_YOFFSET = 1.19;

    private static final String CAMERA_NAME_FRONT = "ApriltagCameraFront";
    private static final String CAMERA_NAME_BACK = "ApriltagCameraBack";
    private final PhotonCamera m_aprilTagCameraFront = new PhotonCamera(CAMERA_NAME_FRONT);
    private final PhotonCamera m_aprilTagCameraBack = new PhotonCamera(CAMERA_NAME_BACK);

    private AprilTagFieldLayout m_aprilTagFieldLayout;

    // Forward B&W camera for Apriltags
    // relative position of the camera on the robot ot the robot center
    // these are the offsets for the camera and need to be changed based off of our
    // robot
    private final Transform3d m_robotToFrontAprilTagCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(3.5), -0.136, Units.inchesToMeters(24.75)),
            new Rotation3d(0.0, 0.0, 0.0));

    private final Transform3d m_robotToBackAprilTagCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), 0, Units.inchesToMeters(0)),
            new Rotation3d(0.0, 0.0, 0.0));

    private final PhotonPoseEstimator m_photonPoseEstimatorFront;
    private final PhotonPoseEstimator m_photonPoseEstimatorBack;

    private final Set<Integer> reportedErrors = new HashSet<>();

    // Simulation support
    private VisionSystemSim m_visionSim;

    public AprilTagVision() {
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Unable to load AprilTag layout " + e.getMessage());
            m_aprilTagFieldLayout = null;
        }

        if (Constants.SIMULATION_SUPPORT) {
            // initialize a simulated camera. Must be done after creating the tag layout
            initializeSimulation();
        }
        // if there is multitag, use the corresponding strategy with reference as back
        // up
        if (USE_MULTITAG) {
            m_photonPoseEstimatorFront = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    m_aprilTagCameraFront, m_robotToFrontAprilTagCam);
            m_photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

            m_photonPoseEstimatorBack = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    m_aprilTagCameraFront, m_robotToBackAprilTagCam);
            m_photonPoseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        } else {
            m_photonPoseEstimatorFront = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_aprilTagCameraFront, m_robotToFrontAprilTagCam);
            m_photonPoseEstimatorBack = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_aprilTagCameraBack, m_robotToBackAprilTagCam);
        }

        // set the driver mode to false
        m_aprilTagCameraFront.setDriverMode(false);
        m_aprilTagCameraBack.setDriverMode(false);
    }

    public void updateSimulation(Pose2d pose) {
        m_visionSim.update(pose);
    }

    private double calculateDifference(Pose3d x, Pose3d y) {
        return x.getTranslation().getDistance(y.getTranslation());
    }

    private void reportFiducialPoseError(int fiducialId) {
        if (!reportedErrors.contains(fiducialId)) {
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
            reportedErrors.add(fiducialId);
        }
    }

    public Optional<EstimatedRobotPose> getEstimateForCamera(Pose2d robotPose, PhotonPoseEstimator poseEstimator) {
        try {
            poseEstimator.setReferencePose(robotPose);
            return poseEstimator.update();
        } catch (Exception e) {
            // bad! log this and keep going
            DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
            return Optional.empty();
        }

    }

    // create a strategy based off closestToReferencePoseStrategy that returns all
    // possible robot positions
    private ArrayList<Pose3d> getAmbiguousPoses(PhotonPipelineResult result, Transform3d robotToCamera) {
        ArrayList<Pose3d> ambigiousPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : result.targets) {
            int targetFiducialId = target.getFiducialId();

            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (targetFiducialId == -1)
                continue;

            Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(target.getFiducialId());

            if (targetPosition.isEmpty()) {
                reportFiducialPoseError(targetFiducialId);
                continue;
            }

            // add all possible robot positions to the array that is returned
            ambigiousPoses.add(
                    targetPosition
                            .get()
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse()));

        }
        return ambigiousPoses;
    }

    public void updateOdometry(SwerveDrivePoseEstimator odometry, Field2d field, PhotonPipelineResult targetResult) {

        if (m_aprilTagFieldLayout == null) {
            return;
        }

        Optional<EstimatedRobotPose> frontEstimate = getEstimateForCamera(odometry.getEstimatedPosition(),
                m_photonPoseEstimatorFront);
        Optional<EstimatedRobotPose> backEstimate = getEstimateForCamera(odometry.getEstimatedPosition(),
                m_photonPoseEstimatorBack);

        List<Pose3d> frontOptions = new ArrayList<Pose3d>();
        List<Pose3d> backOptions = new ArrayList<Pose3d>();

        double frontTimestamp = m_aprilTagCameraFront.getLatestResult().getTimestampSeconds();
        double backTimestamp = m_aprilTagCameraBack.getLatestResult().getTimestampSeconds();

        // if the front camera has any estimates of robot position, add this to the list
        // if front estimate is not there just add back estimate 
        if (!frontEstimate.isPresent()) {
            if (backEstimate.isPresent()) {
                odometry.addVisionMeasurement(backEstimate.get().estimatedPose.toPose2d(), backTimestamp);
            }
            return;
        } 
        // if back estimate is not there add front estimate because we know it is there
        if (!backEstimate.isPresent()) {
            odometry.addVisionMeasurement(frontEstimate.get().estimatedPose.toPose2d(), frontTimestamp);
            return;
        }
        // if multitag is used, add robot pose to frontOptions
        if (frontEstimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            frontOptions.add(frontEstimate.get().estimatedPose);
        } else {
            // if only one tag is visible, add all possible poses to frontOptions
            frontOptions = getAmbiguousPoses(targetResult, m_robotToFrontAprilTagCam);
        }

        // if multitag is used, add robot pose to backOptions
        if (backEstimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            backOptions.add(backEstimate.get().estimatedPose);
        } else {
            // if only one tag is visible, add all possible poses to backOptions
            backOptions = getAmbiguousPoses(targetResult, m_robotToFrontAprilTagCam);
        }

        Pose2d bestBackPose2d = new Pose2d();
        Pose2d bestFrontPose2d = new Pose2d();
        double minDistance = 10e6;

        // compare all backposes and frontposes to each other to find correct robot pose
        for (Pose3d backPoses : backOptions) {
            for (Pose3d frontPoses : frontOptions) {

                double distance = calculateDifference(frontPoses, backPoses);

                Pose2d frontPose2d = frontPoses.toPose2d();
                Pose2d backPose2d = backPoses.toPose2d();
                //makes the smallest difference the measurement 
                if (distance < minDistance) {
                    bestBackPose2d = backPose2d;
                    bestFrontPose2d = frontPose2d;
                    minDistance = distance;

                }

            }

        }
        odometry.addVisionMeasurement(bestFrontPose2d, frontTimestamp);
        odometry.addVisionMeasurement(bestBackPose2d, backTimestamp);
        return;
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        try {
            m_photonPoseEstimatorFront.setReferencePose(prevEstimatedRobotPose);
            return m_photonPoseEstimatorFront.update();
        } catch (Exception e) {
            // bad! log this and keep going
            DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
            return Optional.empty();
        }
    }

    // get the tag ID closest to horizontal center of camera
    // we might want to use this to do fine adjustments on field element locations
    public int getCentralTagId() {
        // make sure camera connected
        if (!m_aprilTagCameraFront.isConnected() && !m_aprilTagCameraBack.isConnected())
            return -1;

        var FrontTargetResult = m_aprilTagCameraFront.getLatestResult();
        var BacktargetResult = m_aprilTagCameraBack.getLatestResult();
        // make a temp holder var for least Y translation, set to first tags translation
        double minY = 1.0e6; // big number
        int targetID = -1;
        for (PhotonTrackedTarget tag : FrontTargetResult.getTargets()) { // for every target in camera
            // find id for current tag we are focusing on
            int tempTagID = tag.getFiducialId();

            // if tag has an invalid ID then skip this tag
            if (tempTagID < 1 || tempTagID > 16) {
                continue;
            }

            // get transformation to target
            Transform3d tagTransform = tag.getBestCameraToTarget();
            // get abs translation to target from transformation
            double tagY = Math.abs(tagTransform.getY());

            // looking for smallest absolute relative to camera Y
            // if abs Y translation of new tag is less then holder tag, it becomes holder
            // tag
            if (tagY < minY) {
                minY = tagY;
                targetID = tempTagID; // remember targetID
            }
        }

        return targetID;
    }

    // get the pose for a tag.
    // will return null if the tag is not in the field map (eg -1)
    public Optional<Pose2d> getTagPose(int tagId) {
        // optional in case no target is found
        Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return Optional.empty(); // returns an empty optional
        }
        return Optional.of(tagPose.get().toPose2d());
    }

    // private static AprilTag constructTag(int id, double x, double y, double z,
    // double angle) {
    // return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0, 0,
    // Math.toRadians(angle))));
    // }

    // // add a new tag relative to another tag. Assume the orientation is the same
    // private static AprilTag constructTagRelative(int id, Pose3d basePose, double
    // x, double y, double z) {
    // return new AprilTag(id, new Pose3d(basePose.getX() + x, basePose.getY() + y,
    // basePose.getZ() + z, basePose.getRotation()));
    // }

    private void initializeSimulation() {
        m_visionSim = new VisionSystemSim("AprilTag");

        // roughly our Arducam camera
        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(800, 600, Rotation2d.fromDegrees(90.0));
        prop.setFPS(60);
        prop.setAvgLatencyMs(10.0);
        prop.setLatencyStdDevMs(3.0);

        // Note: NetworkTables does not update the timestamp of an entry if the value
        // does not change.
        // The timestamp is used by PVLib to know if there is a new frame, so in a
        // simulation
        // with no uncertainty, it thinks that it is not detecting a tag if the robot is
        // static.
        // So, always have a little bit of uncertainty.
        prop.setCalibError(0.1, 0.03);

        PhotonCameraSim cam = new PhotonCameraSim(m_aprilTagCameraFront, prop);
        cam.setMaxSightRange(Units.feetToMeters(20.0));
        m_visionSim.addCamera(cam, m_robotToFrontAprilTagCam);

        m_visionSim.addAprilTags(m_aprilTagFieldLayout);
    }

    // --- Routines to plot the vision solutions on a Field2d ---------

    private void clearTagSolutions(Field2d field) {
        if (field == null)
            return;
        field.getObject("tagSolutions").setPoses();
        field.getObject("visionPose").setPoses();
        field.getObject("visionAltPose").setPoses();
        field.getObject("visibleTagPoses").setPoses();
    }

    private void plotPose(Field2d field, String label, Pose2d pose) {
        if (field == null)
            return;
        if (pose == null)
            field.getObject(label).setPoses();
        else
            field.getObject(label).setPose(pose);
    }

    private void plotVisibleTags(Field2d field, PhotonPipelineResult result) {
        if (field == null)
            return;

        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (PhotonTrackedTarget target : result.getTargets()) {
            int targetFiducialId = target.getFiducialId();
            if (targetFiducialId == -1)
                continue;

            Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId);
            if (targetPosition.isEmpty())
                continue;

            poses.add(targetPosition.get().toPose2d());
        }

        field.getObject("visibleTagPoses").setPoses(poses);
    }

    private void plotAlternateSolution(Field2d field, PhotonPipelineResult targetResult) {
        // NOTE this is for PV 2024
        var pnpResults = VisionEstimation.estimateCamPosePNP(m_aprilTagCameraFront.getCameraMatrix().get(),
                m_aprilTagCameraFront.getDistCoeffs().get(), targetResult.getTargets(), m_aprilTagFieldLayout,
                TargetModel.kAprilTag36h11);
        Pose3d alt = new Pose3d().plus(pnpResults.alt).plus(m_robotToFrontAprilTagCam.inverse());
        plotPose(field, "visionAltPose", alt.toPose2d());
    }
}