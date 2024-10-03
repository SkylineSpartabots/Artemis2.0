package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    static Interpolator<Double> interpolator;
    private static CommandSwerveDrivetrain s_Swerve = TunerConstants.DriveTrain;

    // UnscentedKalmanFilter<N2, N1, N1> UKF;
    Vision m_Camera;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Timer timer;

    public static final double translationDeadband = 0.05;
    public static final double rotDeadband = 0.05;

    // traction control variables
    Pigeon2 pigeon = getPigeon2(); //using the already contructed pigeon

    // Alliance
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    // time inbetween resets
    double dt = 0.02;
    
    private boolean tractionControl = false; // for toggling traction control
    private boolean headingControl = false; // for toggling heading control
    private boolean headingOn = false;
    private double lastTimeReset = 0;

    double prevAccelMagnitude = 0;
    double prevFilteredAccelMagnitude = 0;
    double prevVelocity = 0;        

    private final double slipFactor = 25; // how agressive slip correction is, higher = less agressive
    private final double slipThreshold = 0.20; // how much over and under the slip we allow

    private final double frictionCoefficient = 0.7; // this is an educated guess of the dynamic traction coeffiant (only for in motion friction)
    
    // deadband
    private final double deadbandFactor = 0.8; // closer to 0 is more linear deadband controls

    private Pose2d autoStartPose = new Pose2d(2.0, 2.0, new Rotation2d());
    private Field2d m_field = new Field2d();

    //UKF variables
    // PIDController pidAcceleration = new PIDController(0.3, 0.001, 0);
    // PIDController pidVelocity = new PIDController(0.3, 0.001, 0);

    //Heading 
    PIDController pidHeading = new PIDController(7, 0, 1);

    public void setPidHeadingTolerance() {
        pidHeading.setTolerance(Math.toRadians(2.5));
    }

    double lastHeading = 0; // in radians

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
                                   SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules); // look here for parent library methods
        //  timer = new Timer();                           
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
        SmartDashboard.putNumber("Absolute vel", roughVel);
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

    // ════════════════ Control Systems and Joystick processing ═══════════════════════════════════════════

        // better to be here
public SwerveRequest drive(double driverLY, double driverLX, double driverRX) {
    driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
    driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
    driverRX = scaledDeadBand(driverRX); //desired inputs in velocity

    double desiredVelocity = Math.hypot(driverLX, driverLY);

    if (getTractionBool()) {
        Double[] adjustedInputs = tractionControl(driverLX, driverLY, desiredVelocity);
        driverLX = adjustedInputs[4];
        driverLY = adjustedInputs[5];

        // slipCorrection(adjustedInputs);
    }

    if(getHeadingControlBool()) {
        // driverRX = headingControl(driverRX, desiredVelocity);
         driverRX = pidAlignment(driverRX); // for testing
    }

    return new SwerveRequest.FieldCentric()
        .withVelocityX(driverLY)
        .withVelocityY(driverLX)
        .withRotationalRate(driverRX * Constants.MaxAngularRate)
        .withDeadband(Constants.MaxSpeed * translationDeadband)
        .withRotationalDeadband(Constants.MaxAngularRate * rotDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
}


public Double[] tractionControl(double driverLX, double driverLY, double desiredVelocity) {

    Double[] outputs = new Double[6]; // reset to null every call

    double accelerationMagnitude = obtainAcceleration() * 9.80665; // g to m/s

    if (Math.abs(accelerationMagnitude) < 0.02) { //Pidegon is slightly off so im adding a threshold
        accelerationMagnitude = 0;
    }
    SmartDashboard.putNumber("acceleration magnitude", accelerationMagnitude);

    // Trapaziodal integration (obtain velocity)
    double currentVelocity = prevVelocity + ((accelerationMagnitude + prevFilteredAccelMagnitude) / 2) * dt;
    SmartDashboard.putNumber("currentVelocity", currentVelocity);

    // Obtain desired acceleration from inputs
    double desiredAcceleration = accelerationMagnitude + (desiredVelocity - currentVelocity) / dt;

    // double estimatedVelocity = UKF.getXhat(1);

    // coefficient of friction between the floor and the wheels PLEASE TEST FOR CoF!!
    double maxAcceleration = (9.80665 * frictionCoefficient);

    // ---------------All values for control have been caluculated------------------------------------


        // smallest values of drive inputs that dont result in going over maximum accleration for the time step
    if (desiredAcceleration > maxAcceleration) {

        double epsilon;
        if(driverLY==0) { epsilon = 0; } else { epsilon = driverLX/(driverLY); }

        driverLY = Math.sqrt(Math.pow((currentVelocity + (maxAcceleration*dt)), 2)/(Math.pow(epsilon, 2) + 1));
        driverLX = (epsilon * driverLY);
    }

    for (int i = 0; i < ModuleCount; i++) { 
        TalonFX module = Modules[i].getDriveMotor();
        double wheelRPM = Math.abs(module.getVelocity().getValue() * 60);

        //gets the ratio between what the encoders think our velocity is and the real velocity
        double slipRatio;
        if(currentVelocity == 0) { slipRatio = 1; } else {
            slipRatio = (((2 * Math.PI) / 60) * (wheelRPM * TunerConstants.getWheelRadius() * 0.0254)) / currentVelocity; 
        }

        SmartDashboard.putNumber("Module " + i + " slipratio", slipRatio);
        SmartDashboard.putNumber("Module " + i + " RPM", wheelRPM);
        
        // minimize sensor drift by recalibrating if we are at rest
        // if (wheelRPM < 0.001) { //timer is in seconds btw   && timer.get() >= 1
        //     prevAccelMagnitude = 0;
        //     prevFilteredAccelMagnitude = 0;
        //     prevVelocity = 0;
        //     // UKF.setXhat(MatBuilder.fill(Nat.N2(), Nat.N1(), 0, 0));
        // }

        //if over the upper or lower threshold save the value
        if (slipRatio > (slipThreshold + 1) || slipRatio < (1 - slipThreshold)) {
            outputs[i] = slipRatio;
        }
    }

    // UKF.predict(MatBuilder.fill(Nat.N1(), Nat.N1(), desiredVelocity), dt);

    outputs[4] = driverLX;
    outputs[5] = driverLY;

    return outputs;

} // runs periodically as a default command

    public void slipCorrection(Double[] inputs) {
        // divides by slip factor, more agressive if far above slip threshold 
        for (int i = 0; i < ModuleCount; i++) {

            if (inputs[i] != null) {
                TalonFX module = Modules[i].getDriveMotor();
                
                module.set(module.get() * (1 + (Math.signum(inputs[i] - 1)) * (inputs[i] - slipThreshold)) / slipFactor);
                //https://www.desmos.com/calculator/afe5omf92p how slipfactor changes slip aggression

                SmartDashboard.putBoolean("slipON", true);
            }  else {
                SmartDashboard.putBoolean("slipON", false);
            } 
        }
    }

    public double pidAlignment(double driverRX) {
        boolean rightJoy = Math.abs(driverRX) < (Constants.MaxAngularRate * rotDeadband);

        // Im like 75% sure this is correct, if null it should default to red
        Point target = (alliance.equals(DriverStation.Alliance.Blue)) ? Constants.AlignmentTargets.BLUE_SPEAKER.getValue() : Constants.AlignmentTargets.RED_SPEAKER.getValue();
            
        // Find our (current) x and y, find target's x and y, calculate heading needed to face target, PID to that heading
        if (!rightJoy) {
            Pose2d pose = getPose();

            double desiredHeading = Math.atan2(pose.getX() - target.x, pose.getY() - target.y);

            double currentHeading = pose.getRotation().getRadians();  

            driverRX = pidHeading.calculate(currentHeading, desiredHeading);
        }
        
        return driverRX;
    }

    public double headingControl(double driverRX, double desiredVelocity) {

        boolean rightJoy = Math.abs(driverRX) < (Constants.MaxAngularRate * rotDeadband);

        if (!rightJoy) {
                double currentHeading = getPose().getRotation().getRadians();
                double error = lastHeading - currentHeading;
                if(error < -Math.PI) { lastHeading += 2 * Math.PI; }
                else if(error > Math.PI) { lastHeading -= 2 * Math.PI; }
                driverRX = pidHeading.calculate(currentHeading, lastHeading);
        }

            return driverRX;
    } // me when im bored and i need to expand ignacious drive

    public void setLastHeading(){
        lastHeading = getPose().getRotation().getRadians();
        SmartDashboard.putNumber("lastHeading", lastHeading);
    }

    // public void initKalman() {
    //     // creating the functions
    //     // f function needs to predict the next states based on the previous and the input alone
    //     BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (state, input) -> {
    //         // Extract current states
    //         double accele = state.get(0, 0);
    //         double velocity = state.get(1, 0);

    //         double desiredVelocity = input.get(0, 0);

    //         double nextX = accele + pidAcceleration.calculate(accele, (desiredVelocity - velocity) / dt); // predict next acceleration based on input
    //         double nextA = velocity + pidVelocity.calculate(velocity, desiredVelocity); // predict next velocity base on input

    //         // Construct the predicted next state
    //         return MatBuilder.fill(Nat.N2(), Nat.N1(), nextX, nextA);
    //     };

    //     // h function needs to predict what the measurements would be present based on f's predicted state
    //     BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N1, N1>> h = (stateEstimate, state) -> {
    //         return MatBuilder.fill(Nat.N1(), Nat.N1(), stateEstimate.get(1, 0));
    //     };

    //     // Noise covariance for state and measurment funcitons (not tuned)
    //     Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0, 0); // obtained from noise when sensor is at rest
    //     Matrix<N1, N1> measurementStdDevs = VecBuilder.fill(0); // got from document and gbt

    //     UKF = new UnscentedKalmanFilter<>(Nat.N2(), Nat.N1(), f, h, stateStdDevs, measurementStdDevs, dt);
    //     // determine state and neasurement standard deviation, could use simulation
    //     // UnscentedKalmanFilter​(Nat<States> states, Nat<Outputs> outputs,
    //     // BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<States,​N1>> f,
    //     // BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<Outputs,​N1>> h,
    //     // Matrix<States,​N1> stateStdDevs, Matrix<Outputs,​N1> measurementStdDevs,
    //     // double nominalDtSeconds)
    //     // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/UnscentedKalmanFilter.html
    // goodbye my little kalman

    public double extrapolate(double prevValue, double value, double latency, double dt) { //unused
        return ((value - prevValue) / latency) * dt + value;
    }

    public double obtainAcceleration() {
        double accelerationX = pigeon.getAccelerationX().getValue() - pigeon.getGravityVectorX().getValue();
        double accelerationY = pigeon.getAccelerationY().getValue() - pigeon.getGravityVectorY().getValue();
        
        return Math.signum(Math.atan2(accelerationX, accelerationY)) * Math.sqrt((Math.pow(accelerationX, 2)) + Math.pow(accelerationY, 2));
    }

    public double obtainGyro() { //dont use this -iggy
        double gyroX = pigeon.getAngularVelocityXDevice().getValue();
        double gyroY = pigeon.getAngularVelocityYDevice().getValue();
        double gyroZ = pigeon.getAngularVelocityZDevice().getValue();

        return Math.sqrt(Math.pow(gyroX, 2) + Math.pow(gyroY, 2) + Math.pow(gyroZ, 2));
    }

    public double scaledDeadBand(double input) {
        return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    public void toggleTractionControl() { // for testing
        tractionControl = (tractionControl == false) ? true : false;
        SmartDashboard.putBoolean("traction control", tractionControl);
    }

    public void toggleHeadingControl() { // for testing
        headingControl = (headingControl == false) ? true : false;
        SmartDashboard.putBoolean("heading control", headingControl);
    }
    // maybe create a set heading control method - boolean

    public Boolean getTractionBool() {
        return tractionControl;
    }
    public Boolean getHeadingControlBool() {
        return headingControl;
    }

    public void resetTime() {
        lastTimeReset = System.currentTimeMillis();
    }

    public void setAutoStartPose(Pose2d pose) {
        autoStartPose = pose;
    }
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = Modules[i].getCurrentState();
        }
        return states;
    }

    @Override
    public void periodic() {
        // updateOdometryByVision();
        Pose2d currPose = getPose();

        Logger.recordOutput("acceleration for iggy in gs", obtainAcceleration());

        // allows driver to see if resetting worked
        SmartDashboard.putBoolean("heading control", headingControl);
        SmartDashboard.putBoolean("traction control", tractionControl);
        SmartDashboard.putBoolean("heading on", headingOn);
        // SmartDashboard.putBoolean("Odo Reset (last 5 sec)",
        //         lastTimeReset != -1 && Timer.getFPGATimestamp() - lastTimeReset < 5);
        // SmartDashboard.putNumber("ODO X", currPose.getX());
        // SmartDashboard.putNumber("ODO Y", currPose.getY());
        SmartDashboard.putNumber("ODO ROT", currPose.getRotation().getRadians());
        // SmartDashboard.putNumber("AUTO INIT X", autoStartPose.getX());
        // SmartDashboard.putNumber("AUTO INIT Y", autoStartPose.getY());
        // SmartDashboard.putNumber("Chassis Velocity from wheels", robotAbsoluteVelocity());

        // SmartDashboard.putNumber("DT Vel", robotAbsoluteVelocity());
        m_field.setRobotPose(m_odometry.getEstimatedPosition());
        // SmartDashboard.putData("field", m_field);

        // for (int i = 0; i < ModuleCount; i++) {
        //     Logger.recordOutput("Swerve/DriveMotor" + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
        //     Logger.recordOutput("Swerve/CANcoder module " + i,
        //             Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
        //     // Logger.recordOutput("Swerve/CANCoder offset molule " + i, getOffset(i));
        //     SmartDashboard.putNumber("CANcoder position module " + i,
        //             Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
        //     // SmartDashboard.putNumber("CANCoder offset molule " + i, getOffset(i));
        //     SmartDashboard.putNumber("drive motor velocity mod " + i,
        //             Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
        //     SmartDashboard.putNumber("Angle motor velocity mod " + i,
        //             Modules[i].getSteerMotor().getVelocity().getValueAsDouble());
        // }

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
