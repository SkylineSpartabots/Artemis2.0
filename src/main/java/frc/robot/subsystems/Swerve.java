package frc.robot.subsystems;

import frc.lib.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;


public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController rotationLockPID;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        super(
            new SwerveModuleTalon[]{
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
                new SwerveModuleTalon(Constants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
            },
            new Pigeon(Constants.Ports.PIGEON_ID, Constants.Swerve.GYRO_INVERSION, Constants.Ports.CANIVORE_NAME),
            Constants.Swerve.SWERVE_KINEMATICS
        );

        locationLock = false;
        rotationLockPID = Constants.Swerve.autoRotate.getPIDController(); 
    }

    public void enableRotationLock(double angle) {
        locationLock = true;

        rotationLockPID.setSetpoint(angle);
        rotationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public void disableRotationLock() {
        locationLock = false;
    }

    @Override
    public void drive(Translation2d translation, double rotation, Alliance color) {
        if (locationLock) {
            rotation = rotationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        super.drive(translation, rotation, color);
    }
}