package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera aprilTagCameraL;
    private static PhotonCamera aprilTagCameraR;
    private static PhotonPipelineResult aprilTagCamResultL;
    private static PhotonPipelineResult aprilTagCamResultR;
    private static PhotonTrackedTarget lastValidTarget;
    
    private double targetYaw;
    private double targetDistance;
    private int targetID;

    private Transform3d cameraToRobotTransform = new Transform3d(); //TODO: edit this

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//    private static PhotonCamera visionCamera;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        aprilTagCameraL = new PhotonCamera(Constants.Vision.cameraNameL);
        aprilTagCameraR = new PhotonCamera(Constants.Vision.cameraNameR);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        aprilTagCamResultL = aprilTagCameraL.getLatestResult();
        aprilTagCamResultR = aprilTagCameraR.getLatestResult();
    }

    public PhotonPipelineResult getLatestAprilTagResult(Boolean isLeft) { //true is left false is right
        updateAprilTagResults();
        if (isLeft) {
            return aprilTagCamResultL;
        } else {
            return aprilTagCamResultR;
        }
        
    }

    public List<PhotonTrackedTarget> getTargets(Boolean isLeft) {
       if (isLeft) {
            return aprilTagCamResultL.getTargets();
        } else {
            return aprilTagCamResultR.getTargets();
        }
    }

    public enum CameraResult { //enum rahh
        LEFT,
        RIGHT,
        BOTH
    }

    public CameraResult hasValidTarget() {
        Boolean hasTargetL = aprilTagCamResultL.hasTargets() && aprilTagCamResultL.getBestTarget().getFiducialId() >= 1 && aprilTagCamResultL.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;
        Boolean hasTargetR = aprilTagCamResultR.hasTargets() && aprilTagCamResultR.getBestTarget().getFiducialId() >= 1 && aprilTagCamResultR.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;

         if (hasTargetL && !hasTargetR) {
            return CameraResult.LEFT;
        } else if (!hasTargetL && hasTargetR) {
            return CameraResult.RIGHT;
        } else if (hasTargetL && hasTargetR) {
            return CameraResult.BOTH;
        } else {
            return null; 
        }
    }

    // TODO verify that by the end of auto we have lastValidTarget set

    
    public PhotonTrackedTarget getBestTarget() {
        CameraResult valids = hasValidTarget();
        if (valids != null) {
            if (valids == CameraResult.LEFT) { lastValidTarget = aprilTagCamResultL.getBestTarget();}
            else if (valids == CameraResult.RIGHT) { lastValidTarget = aprilTagCamResultR.getBestTarget();}
            else if (valids == CameraResult.BOTH) {
                if (aprilTagCamResultL.getTimestampSeconds() >= aprilTagCamResultR.getTimestampSeconds()) { // If both cams have a target get the MOST recent one
                    lastValidTarget = aprilTagCamResultR.getBestTarget();
                } else {
                    lastValidTarget = aprilTagCamResultL.getBestTarget();
                }
            }
        } 
        return lastValidTarget;
    } 

    /**
     * @return the absolute distance in meters (there are different methods for horizontal or vertical)
     */
    public double getHypotenuseDistance(){
        targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
            Constants.Vision.cameraHeight, 
            Constants.Vision.aprilTagHeight, 
            Constants.Vision.cameraPitchOffset, 
            Units.degreesToRadians(getBestTarget().getPitch()));
        return targetDistance;
    }

    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    public Pose3d calculatePoseFromVision() throws Exception{ //TODO: integrate multicamera resetting
        PhotonTrackedTarget bestTarget = getBestTarget();
        if(bestTarget == null){
            throw new Exception("No vision target");
        } else {
            Pose3d targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
            return PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), targetPose, cameraToRobotTransform);
        }
    }

    @Override
    public void periodic() {
        // no cam rn lol
        // updateAprilTagResult();
        // SmartDashboard.putBoolean("Has Target", hasValidTarget());
        // SmartDashboard.putBoolean("Has target", hasValidTarget());
        // SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());
    }
}

//potential Kalman implementation: get a sequence of camera readings, run linear 