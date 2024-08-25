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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

public class VisionOLD extends SubsystemBase {
    private static VisionOLD instance;
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

    public double floorDistance;

    private Transform3d cameraToRobotTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(40),Units.degreesToRadians(0))
    ); //center cam

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//    private static PhotonCamera visionCamera;

    public static VisionOLD getInstance() {
        if (instance == null) {
            instance = new VisionOLD();
        }
        return instance;
    }

    private VisionOLD() {
        centerCamera = new PhotonCamera(Constants.Vision.centerCameraName);
        backRightCamera = new PhotonCamera(Constants.Vision.backRightCameraName);
        backLeftCamera = new PhotonCamera(Constants.Vision.backLeftCameraName);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        centerCameraResult = centerCamera.getLatestResult();
        backLeftCameraResult = backLeftCamera.getLatestResult();
        backRightCameraResult = backRightCamera.getLatestResult();
        // backRightCameraResult = backRightCamera.getLatestResult();
    }

    public PhotonTrackedTarget getCenterTarget(){
        if(centerCameraResult.hasTargets()){
            return centerCameraResult.getBestTarget();
        } else {
            return null;
        }
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
    public List<PhotonTrackedTarget> getTargets(){ // TODO - getBesttargets doesnt do this - return a list not a map - conversions? we will see how to do in a sec
        return centerCameraResult.getTargets();
    }

    public enum CameraResult { //enum rahh
        CENTER,
        BACK_LEFT,
        BACK_RIGHT,
    }

    // This is telling you which cameras have a valid target based on our own ambiguity rules not what photonvision reports
    public CameraResult hasValidTarget() {
        PhotonTrackedTarget target = centerCameraResult.getBestTarget();
        PhotonTrackedTarget rightTarget = backRightCameraResult.getBestTarget();
        PhotonTrackedTarget leftTarget = backLeftCameraResult.getBestTarget();

        Boolean centerHasTarget = centerCameraResult.hasTargets() && target.getFiducialId() >= 1 && target.getFiducialId() <= Constants.Vision.aprilTagMax && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
         Boolean backRightHasTarget = backRightCameraResult.hasTargets() && backRightCameraResult.getBestTarget().getFiducialId() >= 1 && backRightCameraResult.getBestTarget().getFiducialId() <= Constants.VisionOLD.aprilTagMax;
         Boolean backLeftHasTarget = backLeftCameraResult.hasTargets() && backLeftCameraResult.getBestTarget().getFiducialId() >= 1 && backLeftCameraResult.getBestTarget().getFiducialId() <= Constants.VisionOLD.aprilTagMax;

        if(centerHasTarget){
            return CameraResult.CENTER;
        } else {
            return null;
        }
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

    // TODO verify that by the end of auto we have lastValidTarget set

    // this tells you th eultimate best target, it decides based upon all the  camreas  based upon hasvalidtarget above or for me the map
    public PhotonTrackedTarget getBestTarget() {

        // preference order: center left right recent but it should just be the most recent and least ambiguous
        CameraResult valids = hasValidTarget();
        if (valids != null) {
            lastValidTarget = centerCameraResult.getBestTarget();
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
    public double getFloorDistance(){ //TODO
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
                Units.degreesToRadians(target.getPitch())
            );
            floorDistance = targetDistance;
            return targetDistance;
        }
        return -1;

    }

    public boolean hasSpeakerTag(){
        List<PhotonTrackedTarget> targets = centerCamera.getLatestResult().getTargets();
        for(PhotonTrackedTarget a : targets){
            if(a.getFiducialId() == 4 || a.getFiducialId() == 8){
                return true;
            }
        }
        return false;
    }

    public double getDistanceToPose(Pose2d pose){
        return PhotonUtils.getDistanceToPose(s_Swerve.getPose(), pose);
    } //TODO


    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
     public Pose3d calculatePoseFromVision() throws Exception{ //TODO: integrate multicamera resetting
         PhotonTrackedTarget bestTarget = getBestTarget();
         if(bestTarget == null){
             throw new Exception("No vision target");
         } else {
             Pose3d targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
             return PhotonUtils.estimateFieldToRobotAprilTag(PhotonUtils.,aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()), cameraToRobotTransform );
         }
     }

    @Override
    public void periodic() {
        updateAprilTagResults();
        try {
            // calculatePoseFromVision();
        } catch (Exception e){}

        Logger.recordOutput("has target", hasValidTarget() != null);
        Logger.recordOutput("VisionOLD/TargetYaw", getYaw());
        Logger.recordOutput("VisionOLD/TargetPitch", getPitch());
        Logger.recordOutput("VisionOLD/FloorDistance", getFloorDistance());
        Logger.recordOutput("VisionOLD/BestTargetID", getBestTarget() == null ? -1 : getBestTarget().getFiducialId());
        Logger.recordOutput("VisionOLD/BestTargetAmbiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);

        SmartDashboard.putBoolean("has target", hasValidTarget() != null);
        SmartDashboard.putNumber("Target yaw", getYaw());
        SmartDashboard.putNumber("Target pitch", getPitch());
        SmartDashboard.putNumber("Target floor distance", getFloorDistance());
        SmartDashboard.putNumber("target id", getBestTarget() == null ? -1 : getBestTarget().getFiducialId());
        SmartDashboard.putNumber("Pose ambiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);
        //SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());
    }
}