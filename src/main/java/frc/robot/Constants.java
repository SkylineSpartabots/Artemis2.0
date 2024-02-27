// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkBase.IdleMode;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double MaxSpeed = 4.5; // 6 meters per second desired top speed, changed to 4.5 for testing
    public static final double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity

    public static final int timeOutMs = 10;
    public static final double stickDeadband = 0.15;
    public static final double triggerDeadzone = 0.2;

    public static final class Vision {
        public static final int aprilTagMax = 16;
        public static final String cameraNameR = "Camera1";
        public static final String cameraNameL = "";
        public static final double cameraHeight = 0; //fix
        public static final double aprilTagHeight = 0.122; //bottom of each april tag is 122cm above carpet | unnecessary, we have photonvision's field layout import
        public static final double cameraRollOffset = Units.degreesToRadians(0);
        public static final double cameraPitchOffset = Units.degreesToRadians(0);
        public static final double cameraYawOffset = Units.degreesToRadians(0);
    }


    // hardware ports for all hardware components on the robot
    // these include CAN IDs, pneumatic hub ports, etc.

    public static final class hardwarePIDs {
        public static final double shooterkP = 0.5;
        public static final double shooterkI = 0.00;
        public static final double shooterkD = 0.00;

        public static final double pivotkP = 0.5;
        public static final double pivotkI = 0.00;
        public static final double pivotkD = 0.00;
    }

    public static final class HardwarePorts {
        // motors (predicted) IDs not fixed

        public static final int shooterBottomM = 50;
        public static final int shooterTopM = 51;
        public static final int indexerTopM = 31;
        public static final int indexerBottomM = 30;
        public static final int intakeLeaderM = 21;
        public static final int intakeFollowerM = 20;
        public static final int serialM = 22;
        public static final int climbLeaderMotor = 60;
        public static final int climbFollowerMotor = 61;
        public static final int pivotLeaderM = 40;
        public static final int pivotFollowerM = 41;
        public static final int pivotCANcoderID = 42;
        public static final int ampMotor = 62;
        
    }

    public static final class FieldConstants {
        public static final double speakerOpeningHeightMeters = 0.2;

        // how far the middle of the speaker opening extends onto the field
        public static final double speakerOpeningMiddleExtendsMeters = 0.023;
        public static final double speakerOpeningAngleDegrees = 14;
        public static final double subwooferExtendsMeters = 0.0939;
    }

    /* Mechanism Current Limits */
    public static final int intakeContinuousCurrentLimit = 30;
    public static final int intakePeakCurrentLimit = 70;
    public static final int serializationContinuousCurrentLimit = 30;
    public static final int serializationPeakCurrentLimit = 70;
    public static final int ampContinuousCurrentLimit = 30;
    public static final int ampPeakCurrentLimit = 70;
    public static final int shooterContinuousCurrentLimit = 30; //commented out because the shooters stop too fast t
    public static final int shooterPeakCurrentLimit = 70;
    public static final int pivotContinuousCurrentLimit = 30;
    public static final int pivotPeakCurrentLimit = 60;
    public static final int climbContinuousCurrentLimit = 30; //arbitrary, fix later
    public static final int climbPeakCurrentLimit = 70; //arbitrary, fix later
    

    public static final double FIELD_WIDTH_METERS = 8.21055;
    public static final double FIELD_LENGTH_METERS = 16.54175;

    // public static final IdleMode intakeNeutralMode = IdleMode.kCoast;
    // public static final IdleMode shooterNeutralMode = IdleMode.kBrake;

    public static final double noteIntakeDistance = 70.0;

    public static final IdleMode intakeNeutralMode = IdleMode.kCoast;
    public static final IdleMode shooterNeutralMode = IdleMode.kBrake;

    public static final double openLoopRamp = 0.25;

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    /**
     * Returns the value from an interpolating double tree map (based on measure values)
     * @param distance
     * @return shooter velocity for given distance
     */
    public static double getVelocityForDistance(double distance){
        //TODO: add a case for a distance that is past furthest or too close
        return ShootingLookupTable.FlywheelVelocitiesMap.get(distance);
    }

    /**
     * Returns the value from an interpolating double tree map (based on measure values)
     * @param distance
     * @return pivot angle for given distance
     */
    public static double getAngleForDistance(double distance){
        //TODO: add a case for a distance that is past furthest or too close
        return ShootingLookupTable.PivotAngleMap.get(distance);
    }

    public static final class ShootingLookupTable {
        public static final InterpolatingDoubleTreeMap FlywheelVelocitiesMap = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap PivotAngleMap = new InterpolatingDoubleTreeMap();

        public static final double[][] velocitiesMatrix = { //TODO: populate these with real measured values
            {Units.inchesToMeters(265), 3000}, //example
        };

        public static final double[][] anglesMatrix = {
            {Units.feetToMeters(265), 20},
        };

        static {
            for(int i = 0; i < velocitiesMatrix.length; i++){
                FlywheelVelocitiesMap.put(velocitiesMatrix[i][0], velocitiesMatrix[i][1]);
            }

            for(int i = 0; i < anglesMatrix.length; i++){
                PivotAngleMap.put(anglesMatrix[i][0], anglesMatrix[i][1]);
            }
        }
    }
}
