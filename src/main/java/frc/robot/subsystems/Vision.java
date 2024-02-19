package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.Logger;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera aprilTagCamera;
    private static PhotonPipelineResult aprilTagCamResult;
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
        aprilTagCamera = new PhotonCamera(Constants.Vision.cameraName);
        updateAprilTagResult();
    }

    public void updateAprilTagResult() {
        aprilTagCamResult = aprilTagCamera.getLatestResult();
    }

    public PhotonPipelineResult getLatestAprilTagResult() {
        updateAprilTagResult();
        return aprilTagCamResult;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return aprilTagCamResult.getTargets();
    }

    public boolean hasValidTarget() {
        return aprilTagCamResult.hasTargets() && aprilTagCamResult.getBestTarget().getFiducialId() >= 1 && aprilTagCamResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;
    }

    // TODO verify that by the end of auto we have lastValidTarget set
    // theres like no way you dont see one at the start of auto maybe I think
    public PhotonTrackedTarget getBestTarget() {
        if (hasValidTarget()) {
            PhotonTrackedTarget newTarget = aprilTagCamResult.getBestTarget();
            lastValidTarget = newTarget;
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
        updateAprilTagResult();
        SmartDashboard.putBoolean("Has Target", hasValidTarget());
        SmartDashboard.putBoolean("Has target", hasValidTarget());
        SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());
    }
}

//potential Kalman implementation: get a sequence of camera readings, run linear 