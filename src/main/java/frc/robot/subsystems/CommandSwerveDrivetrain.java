package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double deadbandFactor = 0.8;

    private double lastTimeReset = -1;

    private static CommandSwerveDrivetrain s_Swerve = TunerConstants.DriveTrain;

    PIDController pidHeading = new PIDController(8, 0, 1);

    private boolean headingControlOn = false;
    private boolean headingControl = false;
    private double lastHeading = 0;

    private boolean aligning = false;

    Pigeon2 pigeon = getPigeon2(); //using the already contructed pigeon

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    //Vision m_Camera;

    private Field2d m_field = new Field2d();

    public static CommandSwerveDrivetrain getInstance(){
        if(s_Swerve == null){
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

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules); //look here for parent library methods
        
        //m_Camera = Vision.getInstance();

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

    public void resetOdo(){ //not being used, drivetrain.seedFieldRelative() instead for field centric driving
        tareEverything();
    }

    public void resetOdoUtil(Pose2d pose){
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

    public double scaledDeadBand(double input) {
        return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    // ════════════════ Control Systems and Joystick processing ═══════════════════════════════════════════

    public SwerveRequest drive(double driverLY, double driverLX, double driverRX){
        driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = scaledDeadBand(driverRX) * Constants.MaxSpeed;
     //desired inputs in velocity

        if(headingControl) {
            driverRX = headingControl(driverRX);
        } else if (aligning) {
            driverRX = pidAlignment(driverRX); // for testing
        }

        return new SwerveRequest.FieldCentric()
        .withVelocityX(driverLY)
        .withVelocityY(driverLX)
        .withRotationalRate(driverRX * Constants.MaxAngularRate)
        .withDeadband(Constants.MaxSpeed * RobotContainer.translationDeadband)
        .withRotationalDeadband(Constants.MaxAngularRate * RobotContainer.rotDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    public void resetOdo(Pose2d pose){
        resetOdoUtil(pose);
    }

    public double pidAlignment(double driverRX) {

        // Point target = (alliance.equals(DriverStation.Alliance.Blue)) ? Constants.AlignmentTargets.BLUE_SPEAKER.getValue() : Constants.AlignmentTargets.RED_SPEAKER.getValue();
        // Find our (current) x and y, find target's x and y, calculate heading needed to face target, PID to that heading
            
            double desiredHeading = Math.PI;

            double currentHeading = getPose().getRotation().getRadians();  

            driverRX = pidHeading.calculate(currentHeading, desiredHeading);
        
        return driverRX;
    }

    public double getLastHeading() {
        return lastHeading;
    }

    public double headingControl(double driverRX){
        if(Math.abs(driverRX) < 0.1 && (Math.abs(lastHeading - getPose().getRotation().getRadians())) < 0.05 && robotAbsoluteVelocity() > 0.1) { //0.5 is placeholder
            driverRX = pidHeading.calculate(getPose().getRotation().getRadians() , lastHeading);
            headingControlOn = true;
            SmartDashboard.putBoolean("headingON", headingControlOn);
        } else {
            headingControlOn = false;
            SmartDashboard.putBoolean("headingON", headingControlOn);
            SmartDashboard.putNumber("lastHeading", lastHeading);
        }

        return driverRX;
    }

    public Pose2d getPose(){
        return s_Swerve.m_odometry.getEstimatedPosition();
    }

    public double robotAbsoluteVelocity(){
        double roughVel = 0.0;
        for(int i = 0; i < ModuleCount; i++){
            roughVel += Modules[i].getCurrentState().speedMetersPerSecond;
        }
        return roughVel/4.0;
    }

    public double robotAngularVelocity(){
        double angularX = pigeon.getAngularVelocityXDevice().getValue();
        double angularY = pigeon.getAngularVelocityYDevice().getValue();
        
        return Math.signum(Math.atan2(angularY, angularX)) * Math.sqrt((Math.pow(angularX, 2)) + Math.pow(angularY, 2));
    }

    public double robotAcceleration() {
        double accelerationX = pigeon.getAccelerationX().getValue() - pigeon.getGravityVectorX().getValue();
        double accelerationY = pigeon.getAccelerationY().getValue() - pigeon.getGravityVectorY().getValue();
        
        return Math.signum(Math.atan2(accelerationX, accelerationY)) * Math.sqrt((Math.pow(accelerationX, 2)) + Math.pow(accelerationY, 2));
    }

    public void updateOdometryByVision(Transform3d transformFromVision){
        if(transformFromVision != null){
            s_Swerve.m_odometry.addVisionMeasurement(new Pose2d(transformFromVision.getX(),transformFromVision.getY(),
            transformFromVision.getRotation().toRotation2d()), Timer.getFPGATimestamp()); // TODO
        }
    }

    public void updateOdometryByVision(Pose3d transformFromVision){
        if(transformFromVision != null){
            s_Swerve.m_odometry.addVisionMeasurement(transformFromVision.toPose2d(), Timer.getFPGATimestamp()); 
        }
    }

    private Pose2d autoStartPose = new Pose2d(2.0, 2.0, new Rotation2d());

    public void setAutoStartPose(Pose2d pose){
        autoStartPose = pose;
    }
        public void setLastHeading() {
        lastHeading = getPose().getRotation().getRadians(); 
    }

    public void toggleHeadingControl() {
        headingControl = !headingControl;
    }

    public void toggleAlignment() {
        aligning = !aligning;
    }

    public double getHeading() {
        return getPose().getRotation().getRadians();
    }

    @Override
    public void periodic() {
        // updateOdometryByVision();
        Pose2d currPose = getPose();


        //allows driver to see if resetting worked
        SmartDashboard.putBoolean("Odo Reset (last 5 sec)", lastTimeReset != -1 && Timer.getFPGATimestamp() - lastTimeReset < 5);
        SmartDashboard.putNumber("ODO X", currPose.getX());
        SmartDashboard.putNumber("ODO Y", currPose.getY());
        SmartDashboard.putNumber("ODO ROT", currPose.getRotation().getRadians());
        SmartDashboard.putNumber("AUTO INIT X", autoStartPose.getX());
        SmartDashboard.putNumber("AUTO INIT Y", autoStartPose.getY());
        SmartDashboard.putNumber("current heading", getHeading());
        SmartDashboard.putNumber("DT Vel", robotAbsoluteVelocity());
        m_field.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putData("field", m_field); 

        for(int i = 0; i < ModuleCount; i++){
            //Logger.recordOutput("Swerve/DriveMotor" + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            //Logger.recordOutput("Swerve/CANcoder module " + i, Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            //Logger.recordOutput("Swerve/CANCoder offset molule " + i, getOffset(i));
            SmartDashboard.putNumber("CANcoder position module " + i, Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble());
            //SmartDashboard.putNumber("CANCoder offset molule " + i, getOffset(i));
            SmartDashboard.putNumber("drive motor velocity mod " + i, Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Angle motor velocity mod " + i, Modules[i].getSteerMotor().getVelocity().getValueAsDouble());
        }


    }

}
