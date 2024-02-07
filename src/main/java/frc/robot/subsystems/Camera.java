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
import edu.wpi.first.math.numbers.N1;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
    private static Camera instance;
    private static PhotonCamera aprilTagCamera;
    private static PhotonPipelineResult aprilTagCamResult;
    private static PhotonTrackedTarget lastValidTarget;

    
      // The observer fuses our encoder data and voltage inputs to reject noise.
    //   private final KalmanFilter<N1, N1, N1> m_observer =
    //   new KalmanFilter<>(
    //       Nat.N1(),
    //       Nat.N1(),
    //       m_flywheelPlant,
    //       VecBuilder.fill(3.0), // How accurate we think our model is
    //       VecBuilder.fill(0.01), // How accurate we think our encoder
    //       // data is
    //       0.020);
    
    private Rotation2d targetYaw;
//    private static PhotonCamera visionCamera;
    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }
    private Camera() {
        aprilTagCamera = new PhotonCamera(Constants.vision.cameraName);
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
        return aprilTagCamResult.hasTargets() && 1 <= aprilTagCamResult.getBestTarget().getFiducialId() && aprilTagCamResult.getBestTarget().getFiducialId() <= Constants.vision.aprilTagMax;
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

    public void updateOdometry(){
    }
}

//potential Kalman implementation: get a sequence of camera readings, run linear 