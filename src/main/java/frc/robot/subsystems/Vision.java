    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot.subsystems;

    import java.io.IOException;
    import java.util.ArrayList;
    import java.util.Optional;

    import org.photonvision.EstimatedRobotPose;
    import org.photonvision.PhotonCamera;
    import org.photonvision.PhotonPoseEstimator;
    import org.photonvision.simulation.PhotonCameraSim;
    import org.photonvision.simulation.VisionSystemSim;
    import org.photonvision.PhotonPoseEstimator.PoseStrategy;
    import org.photonvision.estimation.TargetModel;
    import org.photonvision.estimation.VisionEstimation;
    import org.photonvision.targeting.PhotonPipelineResult;
    import org.photonvision.targeting.PhotonTrackedTarget;

    // import edu.wpi.first.apriltag.AprilTag;
    import edu.wpi.first.apriltag.AprilTagFieldLayout;
    import edu.wpi.first.apriltag.AprilTagFields;
    import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Pose3d;
    import edu.wpi.first.math.geometry.Rotation3d;
    import edu.wpi.first.math.geometry.Transform3d;
    import edu.wpi.first.math.geometry.Translation3d;
    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.smartdashboard.Field2d;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

    import frc.robot.Constants;
import frc.robot.Robot;

    public class Vision {
        // variable to turn on/off our private tag layout
        // if this is false, the compiler should remove all the unused code.
        public static final boolean USE_PRIVATE_TAG_LAYOUT = false;
        
        // Use the multitag pose estimator
        public static final boolean USE_MULTITAG = true;

        // Plot vision solutions
        public static final boolean PLOT_TAG_SOLUTIONS = true;
        
        // constants for extra tags in the shed  (lengths in meters!!)
        static final double SHED_TAG_NODE_XOFFSET = 0.45;
        static final double SHED_TAG_NODE_ZOFFSET = 0.31;
        static final double SHED_TAG_SUBSTATION_YOFFSET = 1.19;

        private static final String CAMERA_NAME = "ApriltagCamera";
        private final PhotonCamera m_aprilTagCamera = new PhotonCamera(CAMERA_NAME);
        private AprilTagFieldLayout m_aprilTagFieldLayout;

        // Forward B&W camera for Apriltags
        // relative position of the camera on the robot ot the robot center
        private final Transform3d m_robotToAprilTagCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(3.5), -0.136, Units.inchesToMeters(24.75)),
                new Rotation3d(0.0, 0.0, 0.0));

        private final PhotonPoseEstimator m_photonPoseEstimator;

        // Simulation support
        private VisionSystemSim m_visionSim;

        public Vision() {
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

            if (USE_MULTITAG) {
                m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        m_aprilTagCamera, m_robotToAprilTagCam);
                m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            } else {
                m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_aprilTagCamera, m_robotToAprilTagCam);
            }

            // set the driver mode to false
            m_aprilTagCamera.setDriverMode(false);
        }

        public void updateSimulation(Pose2d pose) {
            m_visionSim.update(pose);
        }

        public void updateOdometry(SwerveDrivePoseEstimator odometry, Field2d field) {
            if (!m_aprilTagCamera.isConnected())
                return;

            if (m_aprilTagFieldLayout == null)
                return;

            PhotonPipelineResult targetResult = m_aprilTagCamera.getLatestResult();

            SmartDashboard.putBoolean("vision/hasTargets", targetResult.hasTargets());
            if (!targetResult.hasTargets()) {
                // if no target, clean out the numbers
                SmartDashboard.putNumber("vision/targetID", -1);

                if (PLOT_TAG_SOLUTIONS) {
                    clearTagSolutions(field);
                }    
                return;
            } 
            
            // debugging/display assistance
            if (PLOT_TAG_SOLUTIONS) {
                plotVisibleTags(field, targetResult);
            }    
        
            // Estimate the robot pose.
            // If successful, update the odometry using the timestamp of the measurement
            Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(odometry.getEstimatedPosition());
            SmartDashboard.putNumber("vision/timestamp", targetResult.getTimestampSeconds());
            SmartDashboard.putBoolean("vision/foundSolution", result.isPresent());
            if (result.isPresent()) {
                EstimatedRobotPose camPose = result.get();
                SmartDashboard.putString("vision/solutionMethod", camPose.strategy.toString());

                Pose2d estimatedPose = camPose.estimatedPose.toPose2d();
                plotPose(field, "visionPose", estimatedPose);

                double curImageTimeStamp = targetResult.getTimestampSeconds();
                odometry.addVisionMeasurement(estimatedPose, curImageTimeStamp);

                if (PLOT_TAG_SOLUTIONS) {
                    if (camPose.strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR && camPose.strategy != PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
                        plotAlternateSolution(field, targetResult);
                    }
                    else {
                        plotPose(field, "visionAltPose", null);
                    }
                }
            }
            else if (PLOT_TAG_SOLUTIONS) {
                // bug in PV simulation, does not update the timestamp, so believes there is no solution
                if (Robot.isReal()) {
                    plotPose(field, "visionPose", null);
                  plotPose(field, "visionAltPose", null);
               }
           }
        }

        private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            try {
                m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                return m_photonPoseEstimator.update();
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
            if (!m_aprilTagCamera.isConnected())
                return -1;

            var targetResult = m_aprilTagCamera.getLatestResult();

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
                // if abs Y translation of new tag is less then holder tag, it becomes holder tag
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
                return Optional.empty(); //returns an empty optional
            }
            return Optional.of(tagPose.get().toPose2d());
        }

        // private static AprilTag constructTag(int id, double x, double y, double z, double angle) {
        //     return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0, 0, Math.toRadians(angle))));
        // }

        // // add a new tag relative to another tag. Assume the orientation is the same
        // private static AprilTag constructTagRelative(int id, Pose3d basePose, double x, double y, double z) {
        //     return new AprilTag(id, new Pose3d(basePose.getX() + x, basePose.getY() + y, basePose.getZ() + z, basePose.getRotation()));
        // }

        private void initializeSimulation() {
            m_visionSim = new VisionSystemSim("LigerVision");
            // for now, cheat on the specs of the camera
            PhotonCameraSim cam = new PhotonCameraSim(m_aprilTagCamera);
            m_visionSim.addCamera(cam, m_robotToAprilTagCam);

            m_visionSim.addAprilTags(m_aprilTagFieldLayout);
        }

        // --- Routines to plot the vision solutions on a Field2d   ---------

        private void clearTagSolutions(Field2d field) {
            if (field == null) return;
            field.getObject("tagSolutions").setPoses();
            field.getObject("visionPose").setPoses();        
            field.getObject("visionAltPose").setPoses();      
            field.getObject("visibleTagPoses").setPoses();  
        }

        private void plotPose(Field2d field, String label, Pose2d pose) {
            if (field == null) return;
            if (pose == null) 
                field.getObject(label).setPoses();
            else
                field.getObject(label).setPose(pose);
        }

        private void plotVisibleTags(Field2d field, PhotonPipelineResult result) {
            if (field == null) return;

            ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
            for (PhotonTrackedTarget target : result.getTargets()) {
                int targetFiducialId = target.getFiducialId();
                if (targetFiducialId == -1) continue;
                
                Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId);
                if (targetPosition.isEmpty()) continue;

                poses.add(targetPosition.get().toPose2d());
            }

            field.getObject("visibleTagPoses").setPoses(poses);
        }

        private void plotAlternateSolution(Field2d field, PhotonPipelineResult targetResult) {
            // NOTE this is for PV 2024
            var pnpResults = VisionEstimation.estimateCamPosePNP(m_aprilTagCamera.getCameraMatrix().get(),
                    m_aprilTagCamera.getDistCoeffs().get(), targetResult.getTargets(), m_aprilTagFieldLayout,
                    TargetModel.kAprilTag36h11);
            Pose3d alt = new Pose3d().plus(pnpResults.alt).plus(m_robotToAprilTagCam.inverse());
            plotPose(field, "visionAltPose", alt.toPose2d());
        }
    }