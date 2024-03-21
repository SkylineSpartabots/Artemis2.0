package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera centerCamera;
    private static PhotonCamera backRightCamera;
    private static PhotonCamera backLeftCamera;

    // private static PhotonCamera backRightCamera;
    private static PhotonPipelineResult centerCameraResult;
    private static PhotonPipelineResult backRightCameraResult;
    private static PhotonPipelineResult backLeftCameraResult;
    private static PhotonTrackedTarget lastValidTarget;

    private static CommandSwerveDrivetrain s_Swerve;

    private double targetYaw;
    private double targetDistance;
    private int targetID;

    private ArrayList<targetCameraPair> targets = new ArrayList<targetCameraPair>();

    public double floorDistance;

    private static Transform3d centerCameraToRobotTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(0.0), Units.inchesToMeters(-10.5)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(180))); // center
                                                                                                                 // cam
    private static Transform3d backRightCameraToRobotTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(8.125), Units.inchesToMeters(9.9375), Units.inchesToMeters(-10.5)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(150))); // center
                                                                                                                 // cam
    private static Transform3d backLeftCameraToRobotTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(8.125), Units.inchesToMeters(-9.9375), Units.inchesToMeters(-10.5)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(-150))); // center
                                                                                                                  // cam

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // private static PhotonCamera visionCamera;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        centerCamera = new PhotonCamera(Constants.Vision.centerCameraName);
        backRightCamera = new PhotonCamera(Constants.Vision.backRightCameraName);
        backLeftCamera = new PhotonCamera(Constants.Vision.backLeftCameraName);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        centerCameraResult = centerCamera.getLatestResult();
        backLeftCameraResult = backLeftCamera.getLatestResult();
        backRightCameraResult = backRightCamera.getLatestResult();
        for (int i = 0; i < backRightCameraResult.getTargets().size(); i++) {
            PhotonTrackedTarget temp = backRightCameraResult.getTargets().get(i);
            if (temp.getPoseAmbiguity() < 0.3) {
                targets.add(
                        new targetCameraPair(temp, SourceCamera.BACK_RIGHT, backRightCameraResult.getTimestampSeconds()));
            }
        }
        for (int i = 0; i < centerCameraResult.getTargets().size(); i++) {
            PhotonTrackedTarget temp = centerCameraResult.getTargets().get(i);
            if (temp.getPoseAmbiguity() < 0.3) {
                targets.add(
                        new targetCameraPair(temp, SourceCamera.CENTER, backRightCameraResult.getTimestampSeconds()));
            }
        }
        for (int i = 0; i < backLeftCameraResult.getTargets().size(); i++) {
            PhotonTrackedTarget temp = backLeftCameraResult.getTargets().get(i);
            if (temp.getPoseAmbiguity() < 0.3) {
                targets.add(
                        new targetCameraPair(temp, SourceCamera.BACK_LEFT, backLeftCameraResult.getTimestampSeconds()));
            }
        }
        // backRightCameraResult = backRightCamera.getLatestResult();
    }

    public PhotonTrackedTarget getCenterTarget() {
        if (backRightCameraResult.hasTargets()) {
            return backRightCameraResult.getBestTarget();
        } else {
            return null;
        }
    }

    public PhotonTrackedTarget getBackLeftTarget() {
        if (backRightCameraResult.hasTargets()) {
            return backRightCameraResult.getBestTarget();
        } else {
            return null;
        }
    }

    public PhotonTrackedTarget getBackRightTarget() {
        if (backRightCameraResult.hasTargets()) {
            return backRightCameraResult.getBestTarget();
        } else {
            return null;
        }
    }

    public targetCameraPair overallBestTarget;

    public targetCameraPair getBestTargetOverall() {
        updateAprilTagResults();

        targetCameraPair bestTarget = null;
        double smallestAmbi = Double.MAX_VALUE;
        for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).target.getPoseAmbiguity() < smallestAmbi) {
                bestTarget = targets.get(i);
                smallestAmbi = targets.get(i).target.getPoseAmbiguity();
            }
        }
        overallBestTarget = bestTarget;
        return bestTarget;
    }

    class targetCameraPair {
        PhotonTrackedTarget target;
        SourceCamera source;
        double timeStamp;

        targetCameraPair(PhotonTrackedTarget target, SourceCamera source, double timeStamp) {
            this.source = source;
            this.target = target;
            this.timeStamp = timeStamp;
        }
    }

    // public PhotonPipelineResult getLatestAprilTagResult(boolean isBackLeft) {
    // //true is left false is right
    // updateAprilTagResults();
    // if (isBackLeft) {
    // return centerCameraResult;
    // } else {
    // return backRightCameraResult;
    // }

    // }

    public PhotonPipelineResult getLatestAprilTagResult() {
        updateAprilTagResults();
        return backRightCameraResult;
    }

    // public List<PhotonTrackedTarget> getTargets(boolean isBackLeft) {
    // if (isBackLeft) {
    // return backLeftCameraResult.getTargets();
    // } else {
    // return backRightCameraResult.getTargets();
    // }
    // }
    public List<PhotonTrackedTarget> getTargets() {
        return backRightCameraResult.getTargets();
    }

    public enum SourceCamera { // enum rahh
        CENTER(centerCameraToRobotTransform),
        BACK_LEFT(backLeftCameraToRobotTransform),
        BACK_RIGHT(backRightCameraToRobotTransform);

        Transform3d pose;

        SourceCamera(Transform3d pose) {
            this.pose = pose;
        }
    }

    public boolean hasSmartTarget() {
        updateAprilTagResults();
        return targets.size() > 0;
    }

    public SourceCamera hasValidTarget() {

        updateAprilTagResults();

        if (targets.size() > 0) {
            return targets.get(0).source;
        }
        return null;

        // if (backLeftHasTarget && !backRightHasTarget) {
        // return CameraResult.BACK_LEFT;
        // } else if (!backLeftHasTarget && backRightHasTarget) {
        // return CameraResult.BACK_RIGHT;
        // } else if (backLeftHasTarget && backRightHasTarget) {
        // return CameraResult.BOTH;
        // } else {
        // return null;
        // }
    }

    // TODO verify that by the end of auto we have lastValidTarget set

    public PhotonTrackedTarget getBestTarget() {
        SourceCamera valids = hasValidTarget();
        if (valids != null) {
            lastValidTarget = backRightCameraResult.getBestTarget();
            // if (valids == CameraResult.BACK_LEFT) { lastValidTarget =
            // backLeftCameraResult.getBestTarget();}
            // else if (valids == CameraResult.BACK_RIGHT) { lastValidTarget =
            // backRightCameraResult.getBestTarget();}
            // else if (valids == CameraResult.BOTH) {
            // if (backLeftCameraResult.getTimestampSeconds() >=
            // backRightCameraResult.getTimestampSeconds()) { // If both cams have a target
            // get the MOST recent one
            // lastValidTarget = backRightCameraResult.getBestTarget();
            // } else {
            // lastValidTarget = backLeftCameraResult.getBestTarget();
            // }
            // }
        }
        return lastValidTarget;
    }

    public double getYaw() {
        if (getBestTarget() != null) {
            return getBestTarget().getYaw();
        }
        return -1;
    }

    public double getPitch() {
        if (getBestTarget() != null) {
            return getBestTarget().getPitch();
        }
        return -1;
    }

    /**
     * @return the absolute distance in meters (there are different methods for
     *         horizontal or vertical)
     */
    public double getFloorDistance() {
        // PhotonTrackedTarget target = getBestTarget();
        PhotonTrackedTarget target = getCenterTarget();

        if (target != null) {
            // code for back right corner camera
            // targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
            // Units.inchesToMeters(9.1),
            // aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
            // Units.degreesToRadians(30),
            // Units.degreesToRadians(target.getPitch()));
            targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.Vision.centerCameraHeight,
                    aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
                    Constants.Vision.centerCameraPitch,
                    Units.degreesToRadians(target.getPitch()));
            floorDistance = targetDistance;
            return targetDistance;
        }
        return -1;
    }

    public boolean hasSpeakerTag() {
        List<PhotonTrackedTarget> targets = centerCamera.getLatestResult().getTargets();
        for (PhotonTrackedTarget a : targets) {
            if (a.getFiducialId() == 4 || a.getFiducialId() == 8) {
                return true;
            }
        }
        return false;
    }

    public double getDistanceToPose(Pose2d pose) {
        return PhotonUtils.getDistanceToPose(s_Swerve.getPose(), pose);
    }

    /**
     * calculates field-relative robot pose from vision reading, feed to pose
     * estimator (Kalman filter)
     */
    public Pose3d calculatePoseFromVision() { // TODO: integrate multicamera resetting
        targetCameraPair pair = getBestTargetOverall();
        if (pair == null) {
            return null;
        } else {
            PhotonTrackedTarget bestTarget = pair.target;
            SourceCamera sourceCamera = pair.source;
            if (bestTarget == null) {
                return null;
            } else if (sourceCamera == null) {
                return null;
            } else {
                return PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get(), sourceCamera.pose);
            }
        }

    }

    @Override
    public void periodic() {
        updateAprilTagResults();
        try {
            // calculatePoseFromVision();
        } catch (Exception e) {
        }

        Logger.recordOutput("has target", hasValidTarget() != null);
        // Logger.recordOutput("Vision/TargetYaw", getYaw());
        // Logger.recordOutput("Vision/TargetPitch", getPitch());
        // Logger.recordOutput("Vision/FloorDistance", getFloorDistance());
        Logger.recordOutput("Vision/BestTargetID", getBestTarget() == null ? -1 : getBestTarget().getFiducialId());
        Logger.recordOutput("Vision/BestTargetAmbiguity",
                getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);

        SmartDashboard.putBoolean("has smart target", hasSmartTarget());
        SmartDashboard.putBoolean("has target", hasValidTarget() != null);
        // SmartDashboard.putNumber("Target yaw", getYaw());
        // SmartDashboard.putNumber("Target pitch", getPitch());
        // SmartDashboard.putNumber("Target floor distance", getFloorDistance());
        // SmartDashboard.putNumber("target id", getBestTarget() == null ? -1 : getBestTarget().getFiducialId());
        // SmartDashboard.putNumber("Pose ambiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);
        // SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());
    }
}

// potential Kalman implementation: get a sequence of camera readings, run line