package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Set;
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

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double lastTimeReset = 20;
    private boolean tractionGO = false;
    private double slipFactor = 0.02; // 2%
    private double slipThreshold = 1.15; // a little bit of slip is good but needs to be tuned

    private static CommandSwerveDrivetrain s_Swerve = TunerConstants.DriveTrain;
    Pigeon2 pigeon = new Pigeon2(2);

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
        Double[] outputs = new Double[6]; // should be reset to null every call
        double frictionCoefficant = 0.7; // this is an educated guess of the dynamic coeffiant
        double WheelAcceleration = 0;

        double desiredSpeed = Math.sqrt((Math.pow(driverLX, 2) + Math.pow(driverLY, 2)));

        for (int i = 0; i < ModuleCount; i++) {
            TalonFX module = Modules[i].getDriveMotor();
            double slipRatio = (Math.abs(module.getRotorVelocity().getValue() * 60) * ((2
                    * Math.PI) / 60) * (TunerConstants.getWheelRadius() * 0.0254)) / 1; // TODO get velocity from
                                                                                        // acceleration values make
                                                                                        // graph, lookup table basically
            if (slipRatio > slipThreshold) {
                outputs[i] = slipRatio;
            }
        }

        // double currentAcceleration = Math.sqrt(
        // Math.pow(pigeon.getAccelerationX().getValue(), 2) +
        // Math.pow(pigeon.getAccelerationY().getValue(), 2));

        for (int i = 0; i < ModuleCount; i++) {
            TalonFX module = Modules[i].getDriveMotor();
            WheelAcceleration = +(Math.abs(module.getAcceleration().getValue() * 60) * ((2 * Math.PI) / 60)
                    * (TunerConstants.getWheelRadius() * 0.0254)); // 0.0254 is meters in an inch
            if (outputs[i] != null) {
                WheelAcceleration = -WheelAcceleration * (slipFactor + (outputs[i] - slipThreshold) / 2);
            }
            ;
        }

        double lastRun = (System.currentTimeMillis() - lastTimeReset) / 1000;

        double desiredAcceleration = (desiredSpeed - (WheelAcceleration / 4)) / lastRun; // not very sure about this
                                                                                         // math
        double maxAcceleration = (9.80665 * frictionCoefficant) * lastRun;
        // maximum acceleration we can have is equal to g*CoF, where g is the
        // acceleration due to gravity and CoF is the coefficient of friction between
        // the floor and the wheels (rubber and carpet i assumed), last number is for
        // the max acceleration for traction in THIS time step

        if (desiredAcceleration > maxAcceleration) {
            driverLX = WheelAcceleration + (maxAcceleration * lastRun);
            driverLY = WheelAcceleration + (maxAcceleration * lastRun);
        }

        outputs[4] = driverLX;
        outputs[5] = driverLY;
        return outputs;
        // run periodically...
    }

    public void slipCorrection(Double[] inputs) {
        for (int i = 0; i < 4; i++) {
            if (inputs[i] != null) {
                TalonFX module = Modules[i].getDriveMotor();
                module.set(module.get() * (1 - (slipFactor + (inputs[i] - slipThreshold) / 2)));
            }
        }
    }

    private double deadbandFactor = 0.5; // closer to 0 is more linear controls

    public double scaledDeadBand(double input) {
        double newScaled = (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
        return newScaled;
    }

    public void toggleTractionControl() { // for testing
        tractionGO = (tractionGO == false) ? true : false;
    }

    public Boolean getTraction() {
        return tractionGO;
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
