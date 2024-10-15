package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.VisionLimits;
import frc.robot.subsystems.CommandSwerveDrivetrain.Drivetrain;

import java.io.ObjectInputStream.GetField;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera centerCamera;

    // private static PhotonCamera backRightCamera;
    private static PhotonPipelineResult cameraResult;
    private static PhotonTrackedTarget lastValidTarget;

    private double lastProcessedTimestamp = -1;

    private static Drivetrain s_Swerve;
    
    private double targetYaw;
    private double targetDistance;
    private int targetID;

    public double floorDistance;

    private Transform3d cameraToRobotTransform = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(40),Units.degreesToRadians(0))); //center cam

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, centerCamera, cameraToRobotTransform);
    
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
    
    public Vision() {

        s_Swerve = s_Swerve.getInstance();

        centerCamera = new PhotonCamera(Constants.Vision.centerCameraName);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
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

    public Boolean hasValidTarget(PhotonPipelineResult camera) {
        if(camera.hasTargets()) {
            PhotonTrackedTarget target = camera.getBestTarget();

            return
            target.getFiducialId() >= 1 &&
            target.getFiducialId() <= Constants.Vision.aprilTagMax &&
            target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
        } else {return false;}
    }

    public double getDistanceToPose(Pose2d pose){
        return PhotonUtils.getDistanceToPose(s_Swerve.getPose(), pose);
    }

    private Boolean shouldUseMultiTag() {
        MultiTargetPNPResult multiTagResult = cameraResult.getMultiTagResult();

        if(multiTagResult.estimatedPose.bestReprojErr > VisionLimits.k_reprojectionLimit) {
            System.out.println("Rejected MultiTag: high error");
            return false;
        }
        if(multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
            System.out.println("Rejected MultiTag: insufficient ids");
            return false;
        } 
        if(multiTagResult.estimatedPose.best.getTranslation().getNorm() < VisionLimits.k_normThreshold) {
            System.out.println("Rejected MultiTag: norm check failed");
            return false;
        } 
        if(multiTagResult.estimatedPose.ambiguity > VisionLimits.k_ambiguityLimit) {
            System.out.println("Rejected MultiTag: high ambiguity");
            return false;
        }

        //in the future we would have a set of tags we would only want to mega tag

        // for (var fiducialID : multiTagResult.fiducialIDsUsed) {
        //     if (fiducialID =! idk) {
        //     }
        // }

        return true;
    }


    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    public void updateVision() throws Exception{

        if(cameraResult.getTimestampSeconds() != lastProcessedTimestamp) {
                if(Math.abs(s_Swerve.robotAngularVelocity()) > VisionLimits.k_rotationLimitDPS) {

                    if(cameraResult.getMultiTagResult().estimatedPose.isPresent && shouldUseMultiTag()) {
                    
                        s_Swerve.updateOdometryByVision(photonPoseEstimator.update());
                    
                    } else if (hasValidTarget(cameraResult)) {

                        Pose3d targetPose = aprilTagFieldLayout.getTagPose(cameraResult.getBestTarget().getFiducialId()).orElse(null);
                        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                            cameraResult.getBestTarget().getBestCameraToTarget(), targetPose, cameraToRobotTransform);
                        
                        System.out.println("Pose Calculated"); //for testing
                        s_Swerve.updateOdometryByVision(robotPose);
                    
                    } else { System.out.println("Vision failed: no targets");}

            } else { System.out.println("Vision failed: high rotation"); }
        } else { System.out.println("Vision failed: old"); }

        lastProcessedTimestamp = cameraResult.getTimestampSeconds();
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