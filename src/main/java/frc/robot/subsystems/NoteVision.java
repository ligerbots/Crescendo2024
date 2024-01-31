// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class NoteVision {
    // Plot vision solutions
    public static final boolean PLOT_TAG_SOLUTIONS = true;

    private static final String CAMERA_NAME = "NoteCamera";
    private final PhotonCamera m_noteCamera = new PhotonCamera(CAMERA_NAME);

    // relative position of the camera on the robot ot the robot center
    private final Transform3d m_robotToNoteCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), 0, Units.inchesToMeters(22.0)),
            new Rotation3d(0.0, 0.0, 0.0));

    // Simulation support
    private VisionSystemSim m_visionSim;

    public NoteVision() {

        // set the driver mode to false
        m_noteCamera.setDriverMode(false);
    }

    // public void updateSimulation(Pose2d pose) {
    // m_visionSim.update(pose);
    // }
    public List<Pose2d> getNotes() {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        if (!m_noteCamera.isConnected()) {
            return poses;
        }

        var results = m_noteCamera.getLatestResult();
        List<PhotonTrackedTarget> targets = results.getTargets();

        for (PhotonTrackedTarget tgt : targets) {
            double d = Math.abs(m_robotToNoteCam.getZ() / Math.tan(Math.toRadians(tgt.getPitch())));
            double yaw = Math.toRadians(tgt.getYaw());
            double x = d * Math.cos(yaw);
            double y = d * Math.sin(yaw);
            poses.add(new Pose2d(x, y, new Rotation2d(0)));
        }
        return poses;
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

        PhotonCameraSim cam = new PhotonCameraSim(m_noteCamera, prop);
        cam.setMaxSightRange(Units.feetToMeters(20.0));
        m_visionSim.addCamera(cam, m_robotToNoteCam);

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

}

 