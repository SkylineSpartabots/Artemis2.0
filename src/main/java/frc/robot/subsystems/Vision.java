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
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera backLeftCamera;
    private static PhotonCamera backRightCamera;
    private static PhotonPipelineResult backLeftCameraResult;
    private static PhotonPipelineResult backRightCameraResult;
    private static PhotonTrackedTarget lastValidTarget;
    
    private double targetYaw;
    private double targetDistance;
    private int targetID;

    private Transform3d cameraToRobotTransform = new Transform3d(); //TODO: edit this
    private Transform3d backRightCameraOffsetTransform = new Transform3d(-0.2057, 0.2550, -0.2262, new Rotation3d(0, 30, 30));

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//    private static PhotonCamera visionCamera;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        backLeftCamera = new PhotonCamera(Constants.Vision.backLeftCameraName);
        backRightCamera = new PhotonCamera(Constants.Vision.backRightCameraName);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        backLeftCameraResult = backLeftCamera.getLatestResult();
        backRightCameraResult = backRightCamera.getLatestResult();
    }

    public PhotonPipelineResult getLatestAprilTagResult(boolean isBackLeft) { //true is left false is right
        updateAprilTagResults();
        if (isBackLeft) {
            return backLeftCameraResult;
        } else {
            return backRightCameraResult;
        }
        
    }

    public List<PhotonTrackedTarget> getTargets(boolean isBackLeft) {
       if (isBackLeft) {
            return backLeftCameraResult.getTargets();
        } else {
            return backRightCameraResult.getTargets();
        }
    }

    public enum CameraResult { //enum rahh
        BACK_LEFT,
        BACK_RIGHT,
        BOTH
    }

    public CameraResult hasValidTarget() {
        Boolean backLeftHasTarget = backLeftCameraResult.hasTargets() && backLeftCameraResult.getBestTarget().getFiducialId() >= 1 && backLeftCameraResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;
        Boolean backRightHasTarget = backRightCameraResult.hasTargets() && backRightCameraResult.getBestTarget().getFiducialId() >= 1 && backRightCameraResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;

         if (backLeftHasTarget && !backRightHasTarget) {
            return CameraResult.BACK_LEFT;
        } else if (!backLeftHasTarget && backRightHasTarget) {
            return CameraResult.BACK_RIGHT;
        } else if (backLeftHasTarget && backRightHasTarget) {
            return CameraResult.BOTH;
        } else {
            return null; 
        }
    }

    public boolean hasSpeakerTarget() {
        boolean found = false;
        for(int i = 0; i < aprilTagCamResult.getTargets().size(); i++) {
            if(aprilTagCamResult.getTargets().get(i).getFiducialId() == 4 || aprilTagCamResult.getTargets().get(i).getFiducialId() == 7){
                found = true;
            }
        }
        return found;
    }

    public PhotonTrackedTarget getSpeakerTarget() {
        for(int i = 0; i < aprilTagCamResult.getTargets().size(); i++){
            if(aprilTagCamResult.getTargets().get(i).getFiducialId() == 4 || aprilTagCamResult.getTargets().get(i).getFiducialId() == 7){
                return aprilTagCamResult.getTargets().get(i);
            }
        }
        return null;
    }

    // TODO verify that by the end of auto we have lastValidTarget set

    
    public PhotonTrackedTarget getBestTarget() {
        CameraResult valids = hasValidTarget();
        if (valids != null) {
            if (valids == CameraResult.BACK_LEFT) { lastValidTarget = backLeftCameraResult.getBestTarget();}
            else if (valids == CameraResult.BACK_RIGHT) { lastValidTarget = backRightCameraResult.getBestTarget();}
            else if (valids == CameraResult.BOTH) {
                if (backLeftCameraResult.getTimestampSeconds() >= backRightCameraResult.getTimestampSeconds()) { // If both cams have a target get the MOST recent one
                    lastValidTarget = backRightCameraResult.getBestTarget();
                } else {
                    lastValidTarget = backLeftCameraResult.getBestTarget();
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
        updateAprilTagResults();
        //SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());
    }
}

//potential Kalman implementation: get a sequence of camera readings, run linear 