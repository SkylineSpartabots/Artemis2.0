package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.VisionLimits;
import frc.robot.subsystems.CommandSwerveDrivetrain.Drivetrain;

import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.function.BiFunction;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
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
import frc.robot.subsystems.Vision.Vision;
import org.opencv.video.KalmanFilter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;


public class PoseEstimator_EKF { //will estimate pose for odometry, and velocity for control systems
    ExtendedKalmanFilter<N5, N4, N2> EKF;
    double dt = 0.002;
    // states: x & y pos, heading, (Pose2d), x & y velocity , 
    // inputs: Pose2d (from vision), x & y acceleration (raw), angular velocity
    // outputs: Pose2d (filtered), velocity magnitude (filtered)

    public PoseEstimator_EKF() {
        initKalman();
    }

    public void predict(double deltaTime) {
        // Implement the motion model to update the state
        // Update covariance with process noise
    }

    public void correct(Pose2d measurement) {
        // Implement the measurement update
    }

    public void update(Pose2d visionPose, double currentTime) {
    }

    public void initKalman() {
        // f function needs to predict the next states based on the previous and the input alone
        BiFunction<Matrix<N5, N1>, Matrix<N4, N1>, Matrix<N5, N1>> f = (state, input) -> {
            double x = state.get(0, 0);
            double y = state.get(1, 0);
            double heading = state.get(2, 0);
            double vx = state.get(3, 0);
            double vy = state.get(4, 0);
        
            double ax = input.get(0, 0);
            double ay = input.get(1, 0);
            double omega = input.get(2, 0);  // Angular velocity
            double dt = input.get(3, 0);     // Time step
            
            return MatBuilder.fill(Nat.N5(), Nat.N1());
        };

        BiFunction<Matrix<N5, N1>, Matrix<N4, N1>, Matrix<N2, N1>> h = (stateEstimate, state) -> {
            double x = stateEstimate.get(0, 0);
            double y = stateEstimate.get(1, 0);
            double vx = stateEstimate.get(3, 0);
            double vy = stateEstimate.get(4, 0);

            return MatBuilder.fill(Nat.N2(), Nat.N1());
        };

        // Noise covariance for state and measurement functions
        Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05, 0.2, 0.2); // obtained from noise when sensor is at rest
        Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.05, 0.1); // idk how to find this but ill figure  it out
        // matrixes are wrong sizes atm
        EKF = new ExtendedKalmanFilter<>(Nat.N5(), Nat.N4(), Nat.N2(), f, h, stateStdDevs, measurementStdDevs, dt);
        /*  ExtendedKalmanFilter​(Nat<States> states, Nat<Inputs> inputs, Nat<Outputs> outputs,
        BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<States,​N1>> f,
        BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<Outputs,​N1>> h,
        Matrix<States,​N1> stateStdDevs, Matrix<Outputs,​N1> measurementStdDevs,
        double dtSeconds)
         */
    
        }
 }