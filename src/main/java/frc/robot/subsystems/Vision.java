package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Vision extends SubsystemBase {
    public static Vision instance;
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private Transform3d centerCamToRobotTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(40), Units.degreesToRadians(0))
    );
    // TODO ADD MORE TRANSFORMS FOR OTHER  CAMERAS
    // TODO move these to constants - the transforms
    // Cameras
    private static PhotonCamera centerCamera;
    // private static PhotonCamera rightBackCamera;
    // private static PhotonCamera leftBackCamera;

    // Latest Results of every camera
    private static Map<Cameras, PhotonPipelineResult> latestResults = new HashMap<>();

    // Best Targets of every camera
    private static Map<Cameras, PhotonTrackedTarget> bestTargets = new HashMap<>();

    // Cameras with valid targets
    private static Map<Cameras, Boolean> camerasWithValidTargets = new HashMap<>();

    // Valid Camera Names
    public enum Cameras {
        CENTER(centerCamera);
        // RIGHT_BACK(rightBackCamera),
        // LEFT_BACK(leftBackCamera);

        Cameras(PhotonCamera camera) {
        }
    }

    private static CommandSwerveDrivetrain s_Swerve;

    // Get Instance
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    // Constructor
    private Vision() {
        centerCamera = new PhotonCamera("centerCamera");
        // rightBackCamera = new PhotonCamera("rightBackCamera");
        // leftBackCamera = new PhotonCamera("leftBackCamera");
        getBestTargets();
    }


    /**
     * Update the latestResults Map with the latest result from each camera.
     *
     * @return latestResults Map
     */
    public Map<Cameras, PhotonPipelineResult> getLatestResults() {
        latestResults.put(Cameras.CENTER, centerCamera.getLatestResult());
        // latestResults.put(Cameras.RIGHT_BACK, rightBackCamera.getLatestResult());
        // latestResults.put(Cameras.LEFT_BACK, leftBackCamera.getLatestResult());

        return latestResults;
    }

    /**
     * Updates bestTargets Map after calling updateAprilTagResults.
     *
     * @return bestTargets Map
     */
    public Map<Cameras, PhotonTrackedTarget> getBestTargets() {
        getLatestResults(); // Make sure the latestResults are up to date

        if (latestResults.get(Cameras.CENTER) != null) {
            bestTargets.put(Cameras.CENTER, latestResults.get(Cameras.CENTER).getBestTarget());
        }
        // if (latestResults.get(Cameras.RIGHT_BACK) != null) {
        //     bestTargets.put(Cameras.RIGHT_BACK, latestResults.get(Cameras.RIGHT_BACK).getBestTarget());
        // }
        // if (latestResults.get(Cameras.LEFT_BACK) != null) {
        //     bestTargets.put(Cameras.LEFT_BACK, latestResults.get(Cameras.LEFT_BACK).getBestTarget());
        // }

        return bestTargets;
    }

// my implementation of OLDVision.java  hasValidTarget() method is below
// i used a map to store which cameras have valid targets and then call getBestTarget() method to get the best target from any camera

    /**
     * Updates which cameras have valid targets after calling updateBestTargets.
     *
     * @return camerasWithValidTargets Map.
     */
    public Map<Cameras, Boolean> getValidTargets() {
        getBestTargets(); // Make sure besttargets are up to date

        camerasWithValidTargets.put(Cameras.CENTER, isValidTarget(bestTargets.get(Cameras.CENTER)));
        // camerasWithValidTargets.put(Cameras.RIGHT_BACK, isValidTarget(bestTargets.get(Cameras.RIGHT_BACK)));
        // camerasWithValidTargets.put(Cameras.LEFT_BACK, isValidTarget(bestTargets.get(Cameras.LEFT_BACK)));

        return camerasWithValidTargets;
    }

    /**
     * Combines the getTargets of every camera's latestResults.
     * @return List containing every target from every camera.
     */
    public List<PhotonTrackedTarget> getTargetsAsList() {
        List<PhotonTrackedTarget> centerTargets = getLatestResults().get(Cameras.CENTER).getTargets();
        // List<PhotonTrackedTarget> rightBackTargets = getLatestResults().get(Cameras.RIGHT_BACK).getTargets();
        // List<PhotonTrackedTarget> leftBackTargets = getLatestResults().get(Cameras.LEFT_BACK).getTargets();

        List<PhotonTrackedTarget> allTargets = new ArrayList<>(); // Combine all the elements of each individual camera target list into one big list
        allTargets.addAll(centerTargets);
        // allTargets.addAll(rightBackTargets);
        // allTargets.addAll(leftBackTargets);

        return allTargets;
    }
    /**
     * Returns a list of the provided camera's targets
     * @return List containing the targets of the supplied camera.
     */
    public List<PhotonTrackedTarget> getTargetsAsList(Cameras camera) {
        return getLatestResults().get(camera).getTargets() ;
    }

    /**
     * Get the best target from the PhotonVision pipeline based on the selected camera.
     * @param camera Camera to get the best target from.
     * @return The best target from the selected camera based upon the latestResults Map.
     */
    public PhotonTrackedTarget getBestTarget(Cameras camera) {
        return getBestTargets().get(camera);
    }

    // my implementation of OLDVision.java  getBestTarget() method is below

    /**
     * Decides which camera has the best result based on a combined score of age and ambiguity.
     * @return Target with the lowest combined age and ambiguity score.
     */
    public PhotonTrackedTarget getBestTarget() {

        PhotonTrackedTarget bestTarget = null;
        double bestScore = Double.MAX_VALUE; // make sure it isnt tooooo big lol
        double currentTime = System.currentTimeMillis() / 1000.0; // Current time in seconds

        // Loop through the cameras and see which have valid targets
        for (Map.Entry<Cameras, Boolean> entry : getValidTargets().entrySet()) {
            if (entry.getValue()) { // If camera has a valid target
                PhotonTrackedTarget target = getBestTargets().get(entry.getKey());

                double ambiguity = target.getPoseAmbiguity();
                double age = currentTime - latestResults.get(entry.getKey()).getTimestampSeconds(); // Get the time of the latest result - I assume the time should be the same as the bestTarget (PhotonTrackedTarget) but idk
                double score = ambiguity + age; // Combine ambiguity and age to get a score

                // double score = (age * 0.7) + (ambiguity * 0.3); // Weighted sum: age is more important
                // Use this if we need to weight age more importantly, this all would need to be tested to see how everything affects accuracy of results

                // Lowest score wins - minimal ambiguity and age
                if (score < bestScore && isValidTarget(target)) { // Just make sure its valid even though it should be but just checking to be sure
                    bestScore = score;
                    bestTarget = target;
                }
            }
        }
        return bestTarget;
    }

    /**
     * Calculates the position of the robot based on vision readings. Used for odometry.
     * @return Estimated position of the robot.
     * @throws Exception If target is null.
     */
    public Pose3d calculatePoseFromVision() throws Exception {
        PhotonTrackedTarget bestTarget = getBestTarget();
        if (bestTarget != null) {
            Pose3d targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).orElse(null); // Return null if getTagPose doesnt return anything
            if (targetPose != null) {
                return PhotonUtils.estimateFieldToRobotAprilTag(
                        bestTarget.getBestCameraToTarget(),
                        targetPose,
                        centerCamToRobotTransform
                );
            } else {
                throw new Exception("TargetPose is null. Invalid Tag ID");
            }

        } else {
            throw new Exception("No Vision Targets!");
        }
    }

    // Overload land

    /**
     * Gets the Pitch of the best target from any camera.
     *
     * @return Pitch of best target from any camera.
     */
    public double getBestPitch() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            return target.getPitch();
        }
        return -1;
    }

    /**
     * Gets the Yaw of the best target from any camera.
     *
     * @return Yaw of best target from any camera.
     */
    public double getBestYaw() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            return target.getYaw();
        }
        return -1;
    }

    /**
     * Gets the Pitch of the supplied target.
     *
     * @param target Target to get Pitch of.
     * @return Pitch of target.
     */
    public double getPitch(PhotonTrackedTarget target) {
        if (target != null) {
            return target.getPitch();
        }
        return -1;
    }

    /**
     * Gets the Yaw of the supplied target.
     *
     * @param target Target to get Yaw of.
     * @return Yaw of target.
     */
    public double getYaw(PhotonTrackedTarget target) {
        if (target != null) {
            return target.getYaw();
        }
        return -1;
    }

    /**
     * Gets the Pitch of the best target from the supplied camera.
     *
     * @param camera Camera to get best target from.
     * @return Pitch of target from camera
     */
    public double getPitch(Cameras camera) {
        PhotonTrackedTarget target = getBestTargets().get(camera);

        if (target != null) {
            return target.getPitch();
        }
        return -1;
    }

    /**
     * Gets the Yaw of the best target from the supplied camera.
     *
     * @param camera Camera to get best target from.
     * @return Yaw of target from camera
     */
    public double getYaw(Cameras camera) {
        PhotonTrackedTarget target = getBestTargets().get(camera);

        if (target != null) {
            return target.getYaw();
        }
        return -1;
    }
// No roll method :(

    /**
     * Checks if any camera has a speaker target.
     *
     * @param canBeOffsetTag Can the target be an offset speaker tag?
     * @return True if any camera has a speaker tag, false if not.
     */
    public boolean hasSpeakerTag(boolean canBeOffsetTag) {
        List<PhotonTrackedTarget> centerTargets = centerCamera.getLatestResult().getTargets();
        // List<PhotonTrackedTarget> rightBackTargets = rightBackCamera.getLatestResult().getTargets();
        // List<PhotonTrackedTarget> leftBackTargets = leftBackCamera.getLatestResult().getTargets();

        // List<List<PhotonTrackedTarget>> allTargets = List.of(centerTargets, rightBackTargets, leftBackTargets); // A list of lists

        // for (List<PhotonTrackedTarget> list : allTargets) { // For each list in allTargets
            for (PhotonTrackedTarget target : centerTargets) { // For each target in each list
                if (isSpeakerTag(target, canBeOffsetTag)) { // Check if it is a speaker tag
                    return true;
                }
            }
        // }
        return false;
    }

    /**
     * Checks if provided camera has a speaker target.
     *
     * @param camera         Camera to check for a valid speaker tag.
     * @param canBeOffsetTag Can the target be an offset speaker tag?
     * @return True if provided camera has a speaker tag, false if not.
     */
    public boolean hasSpeakerTag(Cameras camera, boolean canBeOffsetTag) {
        List<PhotonTrackedTarget> targets = getLatestResults().get(camera).getTargets();

        for (PhotonTrackedTarget target : targets) { // For each target in each list
            if (isSpeakerTag(target, canBeOffsetTag)) { // Check if it is a speaker tag
                return true;
            }
        }
        return false;
    }

    // should i  return the speaker that has the target? or should i just say yeah somebody has one

    // Utilities

    /**
     * Checks if the target is valid.
     *
     * @param target Target to check.
     * @return True if the target pose is unambiguous and has a real ID, false if pose is ambiguous or ID is invalid.
     */
    public boolean isValidTarget(PhotonTrackedTarget target) {
        return target != null && target.getFiducialId() >= 1 && target.getFiducialId() <= Constants.Vision.aprilTagMax && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
    }

    /**
     * Checks if the target is a speaker tag.
     *
     * @param target         Target to check.
     * @param canBeOffsetTag Can the target be an offset speaker tag?
     * @return True if the target is a speaker tag, false if not.
     */
    public boolean isSpeakerTag(PhotonTrackedTarget target, boolean canBeOffsetTag) {
        if (target == null) {
            return false;
        }
        if (canBeOffsetTag) {
            return target.getFiducialId() == Constants.Vision.AprilTags.redSpeakerOffset || target.getFiducialId() == Constants.Vision.AprilTags.blueSpeakerOffset;
        } else {
            return target.getFiducialId() == Constants.Vision.AprilTags.redSpeakerCenter || target.getFiducialId() == Constants.Vision.AprilTags.blueSpeakerCenter;
        }
    }

    /**
     * Checks if the target is a speaker tag. Always checks if the target is an offset speaker tag.
     *
     * @param target Target to check.
     * @return True if the target is a speaker tag, false if not.
     */
    public boolean isSpeakerTag(PhotonTrackedTarget target) {
        return isSpeakerTag(target, true); // Call the other method so I don't duplicate so much code.
    }


    @Override
    public void periodic() {
        try {
            calculatePoseFromVision();
        } catch (Exception e) {
        }

//        Logger.recordOutput("Has Target", hasValidTarget() != null); //TODO
        Logger.recordOutput("Vision/BestTargetYaw", getBestYaw());
        Logger.recordOutput("Vision/BestTargetPitch", getBestPitch());
//        Logger.recordOutput("Vision/FloorDistance", getFloorDistance()); // TODO
        Logger.recordOutput("Vision/BestTargetID", getBestTarget() != null ? getBestTarget().getFiducialId() : -1); // If not null return FiducialID, otherwise return -1
        Logger.recordOutput("Vision/BestTargetAmbiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);

//        SmartDashboard.putBoolean("has target", hasValidTarget() != null);
        SmartDashboard.putNumber("Vision/BestTargetYaw", getBestYaw());
        SmartDashboard.putNumber("Vision/BestTargetPitch", getBestPitch());
//        SmartDashboard.putNumber("Target floor distance", getFloorDistance());
        SmartDashboard.putNumber("Vision/BestTargetID", getBestTarget() != null ? getBestTarget().getFiducialId() : -1); // If not null return FiducialID, otherwise return -1
        SmartDashboard.putNumber("Vision/BestTargetAmbiguity", getBestTarget() != null ? getBestTarget().getPoseAmbiguity() : -1);
        //SmartDashboard.putNumber("target pitch", getBestTarget().getPitch());    }
    }
}

