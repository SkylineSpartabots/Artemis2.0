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

    public List<PhotonTrackedTarget> getTargets(){
        return cameraResult.getTargets();
    }

    public Boolean hasValidTarget() {
        PhotonTrackedTarget target = cameraResult.getBestTarget();
        return cameraResult.hasTargets() && target.getFiducialId() >= 1 && target.getFiducialId() <= Constants.Vision.aprilTagMax && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
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
    public void updateVision() throws Exception{

        if(cameraResult.getTimestampSeconds() != lastProcessedTimestamp) {
                if(Math.abs(s_Swerve.robotAngularVelocity()) > 175) { //in dps
                    if(cameraResult.getMultiTagResult().estimatedPose.isPresent && shouldUseMultiTag()) {
                        s_Swerve.updateOdometryByVision(cameraResult.getMultiTagResult().estimatedPose.best);
                        return;
                    }

                    if(cameraResult.getBestTarget() != null) {
                        PhotonTrackedTarget bestTarget = cameraResult.getBestTarget();
                        Pose3d targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null);
                        // s_Swerve.updateOdometryByVision(PhotonUtils.estimateFieldToRobotAprilTag(aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId())));
                        // ill fix this like
                    } else { System.out.println("Vision failed: no targets");}
                    
                }
            } else { System.out.println("Vision skip"); }
        }

    @Override
    public void periodic() {

        updateAprilTagResults();
        if(!cameraResult.targets.isEmpty()) {
            try {
                updateVision();
            } catch (Exception e){}
        }

    }

}

//potential Kalman implementation: get a sequence of camera readings, run line