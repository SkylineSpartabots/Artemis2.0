package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera aprilTagCamera;
    private static PhotonPipelineResult aprilTagCamResult;
    private static PhotonTrackedTarget lastValidTarget;

    
    //   public static final LinearSystem<N2, N1, N1> swervePlant =
    //     LinearSystemId.identifyDrivetrainSystem(
    //         DCMotor.getFalcon500(2),
    //         Units.lbsToKilograms(7.5),
    //         1.75, // Do we need to multiple b/c cascading elevator?
    //         elevatorGearRatio);

    // public static final KalmanFilter<N2, N1, N1> elevatorEstimator =
    //     new KalmanFilter<>(
    //         Nat.N2(),
    //         Nat.N1(),
    //         elevatorPlant,
    //         VecBuilder.fill(2, 40), // How accurate we
    //         // think our model is, in inches and inches/second.
    //         VecBuilder.fill(0.001), // How accurate we think our encoder position
    //         // data is. In this case we very highly trust our encoder position reading.
    //         0.020);
    
    private double targetYaw;
    private double targetDistance;
    private int targetID;
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
        return aprilTagCamResult.hasTargets() && 1 <= aprilTagCamResult.getBestTarget().getFiducialId() && aprilTagCamResult.getBestTarget().getFiducialId() <= Constants.Vision.aprilTagMax;
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

    public double getDistance(){
        targetDistance = PhotonUtils.calculateDistanceToTargetMeters(
            Constants.Vision.cameraHeight, 
            Constants.Vision.aprilTagHeight, 
            Constants.Vision.cameraPitchOffset, 
            Units.degreesToRadians(getBestTarget().getPitch()));
        return targetDistance;
    }

    public void updateOdometry(){
    }
}

//potential Kalman implementation: get a sequence of camera readings, run linear 