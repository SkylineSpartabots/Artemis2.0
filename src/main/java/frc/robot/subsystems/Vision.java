package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
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
import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera centerCamera;

    // private static PhotonCamera backRightCamera;
    private static PhotonPipelineResult cameraResult;
    private static PhotonTrackedTarget lastValidTarget;

    private double lastProcessedTimestamp = 0.0;

    private static CommandSwerveDrivetrain s_Swerve;
    
    private double targetYaw;
    private double targetDistance;
    private int targetID;

    public double floorDistance;

    private Transform3d cameraToRobotTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(40),Units.degreesToRadians(0))); //center cam

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//    private static PhotonCamera visionCamera;

    private Vision() {
        centerCamera = new PhotonCamera(Constants.Vision.centerCameraName);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        cameraResult = centerCamera.getLatestResult();
    }

    public PhotonPipelineResult getLatestAprilTagResult(){
        updateAprilTagResults();
        return cameraResult;
    }

    // public List<PhotonTrackedTarget> getTargets(boolean isBackLeft) {
    //    if (isBackLeft) {
    //         return backLeftCameraResult.getTargets();
    //     } else {
    //         return backRightCameraResult.getTargets();
    //     }
    // }
    public List<PhotonTrackedTarget> getTargets(){
        return cameraResult.getTargets();
    }

    public CameraResult hasValidTarget() {
        PhotonTrackedTarget target = cameraResult.getBestTarget();
        PhotonTrackedTarget rightTarget = backRightCameraResult.getBestTarget();
        PhotonTrackedTarget leftTarget = backLeftCameraResult.getBestTarget();

        Boolean centerHasTarget = cameraResult.hasTargets() && target.getFiducialId() >= 1 && target.getFiducialId() <= Constants.Vision.aprilTagMax && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
        // Boolean backRightHasTarget = backRightCameraResult.hasTargets() && backRightCameraResult.getBestTarget().getFiducialId() >= 1 && backRightCameraResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;
        // Boolean backLeftHasTarget = backLeftCameraResult.hasTargets() && backLeftCameraResult.getBestTarget().getFiducialId() >= 1 && backLeftCameraResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;

        if(centerHasTarget){
            return cameraResult.CENTER;
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
            lastValidTarget = cameraResult.getBestTarget();
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

    public double getFloorDistance(){
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
    }

    private Boolean shouldUseMultiTag() {
        MultiTargetPNPResult multiTagResult = cameraResult.getMultiTagResult();

        if(multiTagResult.estimatedPose.bestReprojErr > 0.1) {
            System.out.println("Rejected MultiTag: high error");
            return false;
        }
        if(multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
            System.out.println("Rejected MultiTag: insufficient ids");
            return false;
        } 
        if(multiTagResult.estimatedPose.best.getTranslation().getNorm() < 1) {
            System.out.println("Rejected MultiTag: norm check failed");
            return false;
        } 
        if(multiTagResult.estimatedPose.ambiguity > 0.9) {
            System.out.println("Rejected MultiTag: ambiguity");
            return false;
        }

        //in the future we would have a set of tags we would only want to mega tag

        // for (var fiducialID : multiTagResult.fiducialIDsUsed) {
        //     if (fiducialID =! idk) {
        //     }
        // }

        return true;
    } //ill make constants later


    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    public Pose3d updateVision() throws Exception{

        if(cameraResult.getTimestampSeconds() != lastProcessedTimestamp) {
                if(Math.abs(s_Swerve.robotAngularVelocity()) > 175) { //in dps
                    if(cameraResult.getMultiTagResult().estimatedPose.isPresent) {
                        if(shouldUseMultiTag()) {
                            return cameraResult.getMultiTagResult().estimatedPose.best;
                        }
                    }
                }

            } else { System.out.println("Vision skip"); }
            
        }

        // PhotonTrackedTarget bestTarget = getBestTarget();
        // if(bestTarget == null){
        //     throw new Exception("No vision target");
        // } else {
        //     Pose3d targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
        //     return PhotonUtils.estimateFieldToRobotAprilTag(aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()));
        }
    }

    @Override
    public void periodic() {
        updateAprilTagResults();
        if(!cameraResult.targets.isEmpty()) {
            try {
                updateVision();
            } catch (Exception e){}
        }


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

//potential Kalman implementation: get a sequence of camera readings, run line