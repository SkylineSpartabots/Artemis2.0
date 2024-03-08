package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera centerCamera;
    // private static PhotonCamera backRightCamera;
    private static PhotonPipelineResult centerCameraResult;
    private static PhotonPipelineResult backRightCameraResult;
    private static PhotonTrackedTarget lastValidTarget;

    private static CommandSwerveDrivetrain s_Swerve;
    
    private double targetYaw;
    private double targetDistance;
    private int targetID;

    private Transform3d cameraToRobotTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(40),Units.degreesToRadians(0))); //center cam

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//    private static PhotonCamera visionCamera;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        centerCamera = new PhotonCamera(Constants.Vision.centerCameraName);
        // backRightCamera = new PhotonCamera(Constants.Vision.backRightCameraName);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        centerCameraResult = centerCamera.getLatestResult();
        // backRightCameraResult = backRightCamera.getLatestResult();
    }

    // public PhotonPipelineResult getLatestAprilTagResult(boolean isBackLeft) { //true is left false is right
    //     updateAprilTagResults();
    //     if (isBackLeft) {
    //         return centerCameraResult;
    //     } else {
    //         return backRightCameraResult;
    //     }
        
    // }

    public PhotonPipelineResult getLatestAprilTagResult(){
        updateAprilTagResults();
        return centerCameraResult;
    }

    // public List<PhotonTrackedTarget> getTargets(boolean isBackLeft) {
    //    if (isBackLeft) {
    //         return backLeftCameraResult.getTargets();
    //     } else {
    //         return backRightCameraResult.getTargets();
    //     }
    // }
    public List<PhotonTrackedTarget> getTargets(){
        return centerCameraResult.getTargets();
    }

    public enum CameraResult { //enum rahh
        CENTER,
        BACK_LEFT,
        BACK_RIGHT,
        BOTH
    }

    public CameraResult hasValidTarget() {
        PhotonTrackedTarget target = centerCameraResult.getBestTarget();
        Boolean centerHasTarget = centerCameraResult.hasTargets() && target.getFiducialId() >= 1 && target.getFiducialId() <= Constants.Vision.aprilTagMax && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
        // Boolean backRightHasTarget = backRightCameraResult.hasTargets() && backRightCameraResult.getBestTarget().getFiducialId() >= 1 && backRightCameraResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;

        if(centerHasTarget){
            return CameraResult.CENTER;
        } else {
            return null;
        }
        //  if (backLeftHasTarget && !backRightHasTarget) {
        //     return CameraResult.BACK_LEFT;
        // } else if (!backLeftHasTarget && backRightHasTarget) {
        //     return CameraResult.BACK_RIGHT;
        // } else if (backLeftHasTarget && backRightHasTarget) {
        //     return CameraResult.BOTH;
        // } else {
        //     return null; 
        // }
    }

    // TODO verify that by the end of auto we have lastValidTarget set

    public PhotonTrackedTarget getBestTarget() {
        CameraResult valids = hasValidTarget();
        if (valids != null) {
            lastValidTarget = centerCameraResult.getBestTarget();
            // if (valids == CameraResult.BACK_LEFT) { lastValidTarget = backLeftCameraResult.getBestTarget();}
            // else if (valids == CameraResult.BACK_RIGHT) { lastValidTarget = backRightCameraResult.getBestTarget();}
            // else if (valids == CameraResult.BOTH) {
            //     if (backLeftCameraResult.getTimestampSeconds() >= backRightCameraResult.getTimestampSeconds()) { // If both cams have a target get the MOST recent one
            //         lastValidTarget = backRightCameraResult.getBestTarget();
            //     } else {
            //         lastValidTarget = backLeftCameraResult.getBestTarget();
            //     }
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
     * @return the absolute distance in meters (there are different methods for horizontal or vertical)
     */
    public double getFloorDistance(){
        PhotonTrackedTarget target = getBestTarget();
        if (target != null) {
            targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(9.1), 
            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(), 
            Units.degreesToRadians(30), 
            Units.degreesToRadians(target.getPitch()));
            return targetDistance;
        }
        return -1;

    }

    public double getDistanceToPose(Pose2d pose){
        return PhotonUtils.getDistanceToPose(s_Swerve.getPose(), pose);
    }


    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    // public Pose3d calculatePoseFromVision() throws Exception{ //TODO: integrate multicamera resetting
        // PhotonTrackedTarget bestTarget = getBestTarget();
        // if(bestTarget == null){
        //     throw new Exception("No vision target");
        // } else {
        //     Pose3d targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
            // return PhotonUtils.estimateFieldToRobotAprilTag(aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()));
    //     }
    // }

    @Override
    public void periodic() {
        updateAprilTagResults();
        try {
            // calculatePoseFromVision();
        } catch (Exception e){}

        Logger.recordOutput("has target", hasValidTarget() != null);
        Logger.recordOutput("Vision/TargetYaw", getYaw());
        Logger.recordOutput("Vision/TargetPitch", getPitch());
        Logger.recordOutput("Vision/FloorDistance", getFloorDistance());
        Logger.recordOutput("Vision/BestTargetID", getBestTarget() == null ? -1 : getBestTarget().getFiducialId());
        Logger.recordOutput("Vision/BestTargetAmbiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);

        SmartDashboard.putBoolean("has target", hasValidTarget() != null);
        SmartDashboard.putNumber("Target yaw", getYaw());
        SmartDashboard.putNumber("Target pitch", getPitch());
        SmartDashboard.putNumber("Target floor distance", getFloorDistance());
        SmartDashboard.putNumber("target id", getBestTarget() == null ? -1 : getBestTarget().getFiducialId());
        SmartDashboard.putNumber("Pose ambiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);
        //SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());
    }
}

//potential Kalman implementation: get a sequence of camera readings, run linear 