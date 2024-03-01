// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

import frc.robot.Constants;

public class AprilTagVision {
    // variable to turn on/off our private tag layout
    // if this is false, the compiler should remove all the unused code.
    public static final boolean USE_PRIVATE_TAG_LAYOUT = false;

    // Use the multitag pose estimator
    public static final boolean USE_MULTITAG = true;

    // Plot vision solutions
    public static final boolean PLOT_VISIBLE_TAGS = true;
    public static final boolean PLOT_POSE_SOLUTIONS = true;
    public static final boolean PLOT_ALTERNATE_POSES = true;

    // constants for extra tags in the shed lengths in meters!!)
    static final double SHED_TAG_NODE_XOFFSET = 0.45;
    static final double SHED_TAG_NODE_ZOFFSET = 0.31;
    static final double SHED_TAG_SUBSTATION_YOFFSET = 1.19;

    private static final String CAMERA_NAME_FRONT = "ArducamFront";
    private static final String CAMERA_NAME_BACK = "ArducamBack";
    private final PhotonCamera m_aprilTagCameraFront = new PhotonCamera(CAMERA_NAME_FRONT);
    private final PhotonCamera m_aprilTagCameraBack = new PhotonCamera(CAMERA_NAME_BACK);

    private AprilTagFieldLayout m_aprilTagFieldLayout;

    // Forward B&W camera for Apriltags
    // relative position of the camera on the robot to the robot center
    // use measurements to center of Swerve, and include offset
    // pitch is the Y angle, and it is positive down
    private final Transform3d m_robotToFrontAprilTagCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(0.5 - DriveTrain.ROBOT_SWERVE_OFFSET_X_INCHES), 0, Units.inchesToMeters(18.5)),
            new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));

    private final Transform3d m_robotToBackAprilTagCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-15.5 - DriveTrain.ROBOT_SWERVE_OFFSET_X_INCHES), 0, Units.inchesToMeters(10.0)),
            new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(180.0)));

    private final PhotonPoseEstimator m_photonPoseEstimatorFront;
    private final PhotonPoseEstimator m_photonPoseEstimatorBack;

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
        
        // if there is multitag, use the corresponding strategy with reference as back up
        if (USE_MULTITAG) {
            m_photonPoseEstimatorFront = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    m_aprilTagCameraFront, m_robotToFrontAprilTagCam);
            m_photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

            m_photonPoseEstimatorBack = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    m_aprilTagCameraBack, m_robotToBackAprilTagCam);
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

    public void updateOdometry(SwerveDrivePoseEstimator odometry, Field2d field) {
        // Cannot do anything if there is no field layout
        if (m_aprilTagFieldLayout == null)
            return;

        if (PLOT_VISIBLE_TAGS) {
            plotVisibleTags(field, List.of(m_aprilTagCameraFront, m_aprilTagCameraBack));
        }

        // Warning: be careful about fetching values. If cameras are not connected, you get errors
        // Example: cannot fetch timestamp without checking for the camera.
        // Make sure to test!

        Pose2d robotPose = odometry.getEstimatedPosition();
        Optional<EstimatedRobotPose> frontEstimate = 
                getEstimateForCamera(m_aprilTagCameraFront, m_photonPoseEstimatorFront, robotPose);
        Optional<EstimatedRobotPose> backEstimate = 
                getEstimateForCamera(m_aprilTagCameraBack, m_photonPoseEstimatorBack, robotPose);

        // if front estimate is not there just add back estimate 
        if (!frontEstimate.isPresent()) {
            if (backEstimate.isPresent()) {
                Pose2d pose = backEstimate.get().estimatedPose.toPose2d();
                odometry.addVisionMeasurement(pose, m_aprilTagCameraBack.getLatestResult().getTimestampSeconds());
                        
                if (PLOT_POSE_SOLUTIONS) {
                    plotVisionPose(field, pose);
                }
                if (PLOT_ALTERNATE_POSES) {
                    // *** Yes, this is repeated code, and maybe that is bad. 
                    // But this will save some cycles if this PLOT option is turned off.
                    if (backEstimate.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                        plotAlternateSolutions(field,
                                List.of(getAmbiguousPoses(m_aprilTagCameraBack.getLatestResult(), m_robotToBackAprilTagCam)));
                    } else
                        field.getObject("visionAltPoses").setPose(pose);
                }
            } else {
                // no results, so clear the list in the Field
                plotVisionPoses(field,null);
                field.getObject("visionAltPoses").setPoses();
            }

            return;
        } 

        // if back estimate is not there add front estimate because we know it is there
        if (!backEstimate.isPresent()) {
            Pose2d pose = frontEstimate.get().estimatedPose.toPose2d();
            odometry.addVisionMeasurement(pose, m_aprilTagCameraFront.getLatestResult().getTimestampSeconds());
                                    
            if (PLOT_POSE_SOLUTIONS) {
                plotVisionPose(field, pose);
            }
            if (PLOT_ALTERNATE_POSES) {
                // *** Yes, this is repeated code, and maybe that is bad.
                // But this will save some cycles if this PLOT option is turned off.
                if (frontEstimate.get().strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
                    plotAlternateSolutions(field, 
                            List.of(getAmbiguousPoses(m_aprilTagCameraFront.getLatestResult(), m_robotToFrontAprilTagCam)));
                } else
                        field.getObject("visionAltPoses").setPose(pose);
            }

            return;
        }

        // Create a list of Pose3d options for the front camera
        List<Pose3d> frontOptions = new ArrayList<Pose3d>();
        if (frontEstimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            // if multitag is used, add robot pose to frontOptions
            frontOptions.add(frontEstimate.get().estimatedPose);
        } else {
            // if only one tag is visible, add all possible poses to frontOptions
            frontOptions = getAmbiguousPoses(m_aprilTagCameraFront.getLatestResult(), m_robotToFrontAprilTagCam);
        }

        // Create a list of Pose3d options for the back camera
        List<Pose3d> backOptions = new ArrayList<Pose3d>();
        if (backEstimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            // if multitag is used, add robot pose to backOptions
            backOptions.add(backEstimate.get().estimatedPose);
        } else {
            // if only one tag is visible, add all possible poses to backOptions
            backOptions = getAmbiguousPoses(m_aprilTagCameraBack.getLatestResult(), m_robotToBackAprilTagCam);
        }

        Pose3d bestBackPose3d = new Pose3d();
        Pose3d bestFrontPose3d = new Pose3d();
        double minDistance = 1e6;

        if (PLOT_ALTERNATE_POSES) {
            plotAlternateSolutions(field, List.of(frontOptions, backOptions));
        }

        // compare all backposes and frontposes to each other to find correct robot pose
        for (Pose3d backPose : backOptions) {
            for (Pose3d frontPose : frontOptions) {
                double distance = calculateDifference(frontPose, backPose);

                // makes the smallest difference the measurement 
                if (distance < minDistance) {
                    bestBackPose3d = backPose;
                    bestFrontPose3d = frontPose;
                    minDistance = distance;
                }
            }
        }

        odometry.addVisionMeasurement(bestFrontPose3d.toPose2d(), m_aprilTagCameraFront.getLatestResult().getTimestampSeconds());
        odometry.addVisionMeasurement(bestBackPose3d.toPose2d(), m_aprilTagCameraBack.getLatestResult().getTimestampSeconds());

        if (PLOT_POSE_SOLUTIONS) {
            plotVisionPoses(field, List.of(bestFrontPose3d.toPose2d(), bestBackPose3d.toPose2d()));
        }
        return;
    }

    // get the tag ID closest to horizontal center of camera
    // we might want to use this to do fine adjustments on field element locations
    public int getCentralTagId() {
        // make sure camera connected
        if (!m_aprilTagCameraFront.isConnected())
            return -1;

        var targetResult = m_aprilTagCameraFront.getLatestResult();
        // make a temp holder var for least Y translation, set to first tags translation
        double minY = 1.0e6; // big number
        int targetID = -1;
        for (PhotonTrackedTarget tag : targetResult.getTargets()) { // for every target in camera
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

    // Private routines for calculating the odometry info

    private double calculateDifference(Pose3d x, Pose3d y) {
        return x.getTranslation().getDistance(y.getTranslation());
    }

    private Optional<EstimatedRobotPose> getEstimateForCamera(PhotonCamera cam, PhotonPoseEstimator poseEstimator, Pose2d robotPose) {
        if (!cam.isConnected()) return Optional.empty();

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

            if (targetPosition.isEmpty())
                continue;

            // add all possible robot positions to the array that is returned
            ambigiousPoses.add(
                    targetPosition.get()
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse()));
            ambigiousPoses.add(
                    targetPosition.get()
                            .transformBy(target.getAlternateCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse()));
        }

        return ambigiousPoses;
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

        // Note: NetworkTables does not update the timestamp of an entry if the value does not change.
        // The timestamp is used by PVLib to know if there is a new frame, so in a simulation
        // with no uncertainty, it thinks that it is not detecting a tag if the robot is static.
        // So, always have a little bit of uncertainty.
        prop.setCalibError(0.1, 0.03);

        PhotonCameraSim cam = new PhotonCameraSim(m_aprilTagCameraFront, prop);
        cam.setMaxSightRange(Units.feetToMeters(20.0));
        m_visionSim.addCamera(cam, m_robotToFrontAprilTagCam);

        cam = new PhotonCameraSim(m_aprilTagCameraBack, prop);
        cam.setMaxSightRange(Units.feetToMeters(20.0));
        m_visionSim.addCamera(cam, m_robotToBackAprilTagCam);

        m_visionSim.addAprilTags(m_aprilTagFieldLayout);
    }

    // --- Routines to plot the vision solutions on a Field2d ---------

    private void plotVisionPoses(Field2d field, List<Pose2d> poses) {
        if (field == null)
            return;
        if (poses == null)
            field.getObject("visionPoses").setPoses();
        else
            field.getObject("visionPoses").setPoses(poses);
    }

    private void plotVisionPose(Field2d field, Pose2d pose) {
        if (field == null)
            return;
        field.getObject("visionPoses").setPose(pose);
    }

    private void plotVisibleTags(Field2d field, List<PhotonCamera> cameraList) {
        if (field == null)
            return;

        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        for (PhotonCamera cam : cameraList) {
            if (!cam.isConnected()) continue;

            for (PhotonTrackedTarget target : cam.getLatestResult().getTargets()) {
                int targetFiducialId = target.getFiducialId();
                if (targetFiducialId == -1)
                    continue;

                Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId);
                if (!targetPosition.isEmpty())
                    poses.add(targetPosition.get().toPose2d());
            }
        }

        field.getObject("visibleTagPoses").setPoses(poses);
    }

    private void plotAlternateSolutions(Field2d field, List<List<Pose3d>> allPoses) {
        if (field == null)
            return;

        ArrayList<Pose2d> both = new ArrayList<>();
        for (List<Pose3d> pl: allPoses) {
            for (Pose3d p : pl)
                both.add(p.toPose2d());
        }

        field.getObject("visionAltPoses").setPoses(both);
    }
}