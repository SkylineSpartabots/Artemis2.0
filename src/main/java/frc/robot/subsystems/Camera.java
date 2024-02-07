package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Rotation2d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
    private static Camera instance;
    private static PhotonCamera aprilTagCamera;
    private static PhotonPipelineResult aprilTagCamResult;
    private static PhotonTrackedTarget lastValidTarget;
    
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
    public boolean hasTargets() {
        return aprilTagCamResult.hasTargets();
    }
    public boolean isValidTarget(PhotonTrackedTarget target) {
        int id = target.getFiducialId();
        return 1 <= id && id <= Constants.vision.aprilTagMax;
    }
    // TODO verify that by the end of auto we have lastValidTarget set
    // theres like no way you dont see one at the start of auto maybe I think
    public PhotonTrackedTarget getBestTarget() {
        if (hasTargets()) {
            PhotonTrackedTarget newTarget = aprilTagCamResult.getBestTarget();
            if (isValidTarget(newTarget)) {
                lastValidTarget = newTarget;
            }
        }
        return lastValidTarget;
    }
}