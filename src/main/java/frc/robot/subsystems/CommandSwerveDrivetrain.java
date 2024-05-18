package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import javax.crypto.Mac;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;



/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    static Interpolator<Double> interpolator;

    // traction control variables
    private double lastTimeReset = 0;
    private boolean TractionControlON = false; // for toggling traction control
    private double slipFactor = 5; // how agressive slip correction is, higher = less agressive
    private double slipThreshold = 1.15; // a little bit of slip is good but needs to be tuned
    private double frictionCoefficant = 0.7; // this is an educated guess of the dynamic traction coeffiant (only for in motion friction)
    double prevAcceX = 0;
    double prevAcceY = 0;
    double prevAcceZ = 0;
    double prevaccelerationMagnitude = 0;
    double dt = 0.03;

    private double deadbandFactor = 0.5; // closer to 0 is more linear deadband controls

    private static CommandSwerveDrivetrain s_Swerve = TunerConstants.DriveTrain;
    Pigeon2 pigeon = getPigeon2(); //using the already contructed pigeon

    Vision m_Camera;

    private Field2d m_field = new Field2d();

    public static CommandSwerveDrivetrain getInstance() {
        if (s_Swerve == null) {
            s_Swerve = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, 250, TunerConstants.FrontLeft,
                    TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        }
        return s_Swerve;
    }

    private void limit() {
        for (SwerveModule module : Modules) {
            CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
            configs.SupplyCurrentLimit = 20;
            configs.SupplyCurrentLimitEnable = true;
            configs.StatorCurrentLimit = 40;
            configs.StatorCurrentLimitEnable = true;

            module.getDriveMotor().getConfigurator().apply(configs);
            module.getSteerMotor().getConfigurator().apply(configs);
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules); // look here for parent library methods

        m_Camera = Vision.getInstance();

        if (Utils.isSimulation()) {
            startSimThread();
        }
        limit();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        limit();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetOdo() { // not being used, drivetrain.seedFieldRelative() instead for field centric
                             // driving
        tareEverything();
        tareEverything();
        tareEverything();
    }

    public void resetOdoUtil(Pose2d pose) {
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, pose);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    public void resetOdo(Pose2d pose) {
        resetOdoUtil(pose);
        resetOdoUtil(pose);
        resetOdoUtil(pose);
    }

    public Pose2d getPose() {
        return s_Swerve.m_odometry.getEstimatedPosition();
    }

    public double robotAbsoluteVelocity() {
        double roughVel = 0.0;
        for (int i = 0; i < ModuleCount; i++) {
            roughVel += Modules[i].getCurrentState().speedMetersPerSecond;
        }
        return roughVel / 4.0;
    }

    public void setVoltage(double voltage) {
        for (int i = 0; i < ModuleCount; i++) {
        }
        // s_Swerve.Modules[0].apply(null, null);
    }

    public void updateOdometryByVision() {
        Pose3d poseFromVision = null;
        try {
            // poseFromVision = m_Camera.calculatePoseFromVision();
        } catch (Exception e) {
        }
        // if(poseFromVision != null){
        // s_Swerve.m_odometry.addVisionMeasurement(poseFromVision.toPose2d(),
        // Logger.getRealTimestamp()); //Timer.getFPGATimestamp()
        // }
    }

    public Double[] tractionControl(double driverLX, double driverLY) {

        initKalman(); // i want this only ran once
        
        Double[] outputs = new Double[6]; // reset to null every call

        double desiredVelocity = Math.hypot(driverLX, driverLY);

        double accelerationX = pigeon.getAccelerationX().getValue() - pigeon.getGravityVectorX().getValue();
        double accelerationY = pigeon.getAccelerationY().getValue() - pigeon.getGravityVectorY().getValue();
        double accelerationZ = pigeon.getAccelerationZ().getValue() - pigeon.getGravityVectorZ().getValue(); 
        //technically we dont need z but it should help if the robot tilts a bit (no harm in having it)

        double latency = pigeon.getAccelerationX().getTimestamp().getLatency(); 

            latency = 1.5 - (latency/100);
            accelerationX = interpolator.interpolate(prevAcceX, accelerationX, latency);
            accelerationY = interpolator.interpolate(prevAcceY, accelerationY, latency);
            accelerationZ = interpolator.interpolate(prevAcceZ, accelerationX, latency); //TODO align all timestamps at 50ms since last run, also lets hope this has exterpolation built in 🙏🙏

         double accelerationMagnitude = Math.sqrt(Math.pow(accelerationX, 2) + Math.pow(accelerationY, 2) + Math.pow(accelerationZ, 2));

        Matrix<N2, N1> inputMatrix = MatBuilder.fill(Nat.N2(),Nat.N1(),accelerationMagnitude, desiredVelocity);
        UKF.predict(inputMatrix, dt);

         double estimatedVelocity = UKF.getS(1, 0);

            int k=0;
            for (int i = 0; i < ModuleCount; i++) {
                TalonFX module = Modules[i].getDriveMotor();
                double wheelRPM = Math.abs(module.getVelocity().getValue() * 60);
                double slipRatio = (((2 * Math.PI) / 60) * (wheelRPM * TunerConstants.getWheelRadius()
                        * 0.0254)) / estimatedVelocity;

                if(wheelRPM == 0) { //minimize drift by recalibrating if we are at rest
                    k++;
                    if(k==3) { //if 3 wheels say we are stopped 
                        recalibrateVelocity();
                        break;
                    }
                }

                if (slipRatio > slipThreshold) {
                    outputs[i] = slipRatio;
                }
            }

            double desiredAcceleration = (desiredVelocity - estimatedVelocity) / dt;

            double maxAcceleration = (9.80665 * frictionCoefficant) * dt;
             /* 
             maximum acceleration we can have is equal to g*CoF, where g is the
             acceleration due to gravity and CoF is the coefficient of friction between
             the floor and the wheels (rubber and carpet i assumed), last number is for
             the max acceleration for traction in THIS time step */

            if (desiredAcceleration > maxAcceleration) {
                while (desiredVelocity > (9.80665 * frictionCoefficant) * Math.pow(dt, 2) + estimatedVelocity) {//algebruh - if you wanna go faster than is possible in the time
                driverLX =- 0.02;
                driverLY =- 0.02;
                desiredVelocity = Math.hypot(driverLX,driverLY);
                } //smallest values of drive inputs that dont result in going over calculated max acceleration
            } 

        // UKF.correct(null, null, null); //TODO correct based on new inputs

        outputs[4] = driverLX;
        outputs[5] = driverLY;

        return outputs;

    } // runs periodically as a default command


    private void recalibrateVelocity() {
        UKF.reset();
    }

    private void initKalman() {

        // creating the functions
        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (state, input) -> {
            // Extract current states
            double accele = state.get(0, 0);
            double velocity = state.get(1, 0);

            double desiredVelocity = input.get(0, 0);
            
            // 0.5 * (accele + prevaccelerationMagnitude) * dt

            // predicting
            double nextX = accele; //predict next acceleration based on input
            double nextA = velocity; //predict next velocity based on input
            
            // Construct the predicted next state
            Matrix<N2, N1> nextState = MatBuilder.fill(Nat.N2(),Nat.N1(),nextX, nextA);
            
            return nextState;
        };
        //f function needs to predict the next states based on the previous and the input alone

        BiFunction<Matrix<N2, N1>, Matrix<N1, N1>,Matrix<N1, N1>> h = (stateEstimate, state) -> {
            return MatBuilder.fill(Nat.N1(),Nat.N1(),stateEstimate.get(1,0));
        };

         // h function needs to predict what the measurements would be present based on f's predicted state 

         //Noise covariance for state and measurment funcitons
         Matrix<N2,N1> stateStdDevs = VecBuilder.fill(0.1,0.1);
         Matrix<N1,N1> measurementStdDevs = VecBuilder.fill(0.1);

        //TODO propigate sigma points   
        //TODO cross variance with state vs mesurment prediction (correct)
         UnscentedKalmanFilter<N2,N1,N1> UKF = new UnscentedKalmanFilter<>(Nat.N2(), Nat.N1(), f, h, stateStdDevs, measurementStdDevs, dt);

        //TODO determine state and neasurement standard deviation, could use simulation or smth else
        //TODO f
        // UnscentedKalmanFilter​(Nat<States> states, Nat<Outputs> outputs, BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<States,​N1>> f,
        // BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<Outputs,​N1>> h, Matrix<States,​N1> stateStdDevs, Matrix<Outputs,​N1> measurementStdDevs, double nominalDtSeconds)
        // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/UnscentedKalmanFilter.html
    }

    public void slipCorrection(Double[] inputs) {
        for (int i = 0; i < ModuleCount; i++) {
            if (inputs[i] != null) {
                TalonFX module = Modules[i].getDriveMotor   ();
                module.set(module.get() * (1 - (inputs[i] - slipThreshold)) / slipFactor);
            } // divides by slip factor, more agressive if far above slip threshold
        }
    }

    public double scaledDeadBand(double input) {
        return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    public void toggleTractionControl() { // for testing
        TractionControlON = (TractionControlON == false) ? true : false;
    }

    public Boolean getTractionBool() {
        return TractionControlON;
    }

    public void resetTime() {
        lastTimeReset = System.currentTimeMillis();
    }

    private Pose2d autoStartPose = new Pose2d(2.0, 2.0, new Rotation2d());

    public void setAutoStartPose(Pose2d pose) {
        autoStartPose = pose;
    }

    @Override
    public void periodic() {
        // updateOdometryByVision();
        Pose2d currPose = getPose();

        // allows driver to see if resetting worked
        SmartDashboard.putBoolean("Odo Reset (last 5 sec)",
                lastTimeReset != -1 && Timer.getFPGATimestamp() - lastTimeReset < 5);
        SmartDashboard.putNumber("ODO X", currPose.getX());
        SmartDashboard.putNumber("ODO Y", currPose.getY());
        SmartDashboard.putNumber("ODO ROT", currPose.getRotation().getRadians());
        SmartDashboard.putNumber("AUTO INIT X", autoStartPose.getX());
        SmartDashboard.putNumber("AUTO INIT Y", autoStartPose.getY());

        SmartDashboard.putNumber("Chassis Velocity from wheels", robotAbsoluteVelocity());

        SmartDashboard.putNumber("DT Vel", robotAbsoluteVelocity());
        m_field.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putData("field", m_field);

        for (int i = 0; i < ModuleCount; i++) {
            Logger.recordOutput("Swerve/DriveMotor" + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            Logger.recordOutput("Swerve/CANcoder module " + i,
                    Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            // Logger.recordOutput("Swerve/CANCoder offset molule " + i, getOffset(i));
            SmartDashboard.putNumber("CANcoder position module " + i,
                    Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            // SmartDashboard.putNumber("CANCoder offset molule " + i, getOffset(i));
            SmartDashboard.putNumber("drive motor velocity mod " + i,
                    Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Angle motor velocity mod " + i,
                    Modules[i].getSteerMotor().getVelocity().getValueAsDouble());
        }

    }

    // private double getOffset(int id) {
    // if (id == 1) {
    // return TunerConstants.kFrontLeftEncoderOffset;
    // }
    // else if (id == 2) {
    // return TunerConstants.kFrontRightEncoderOffset;
    // }
    // else if (id == 3) {
    // return TunerConstants.kBackLeftEncoderOffset;
    // }
    // else {
    // return TunerConstants.kBackRightEncoderOffset;
    // }
    // }

}
