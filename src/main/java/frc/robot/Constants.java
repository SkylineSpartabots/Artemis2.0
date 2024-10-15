// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants.AprilTags;

import org.opencv.core.Point;

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

    public static double MaxSpeed = 6; //can be lowered during testing
    public static double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    public static final double slipFactor = 65;
    public static final double slipThreshold = 0.15;
    public static final int timeOutMs = 10;
    public static final double stickDeadband = 0.05;
    public static final double triggerDeadzone = 0.2;

    public static final class VisionConstants {
        public static final int aprilTagMax = 16;
        public static final String centerCameraName = "Center";
        public static final String backLeftCameraName = "BackLeft";
        public static final String backRightCameraName = "BackRight";
        public static final double cameraHeight = 0; //fix
        public static final double aprilTagHeight = 0.122; //bottom of each april tag is 122cm above carpet | unnecessary, we have photonvision's field layout import
        public static final double cameraRollOffset = Units.degreesToRadians(0);
        public static final double cameraPitchOffset = Units.degreesToRadians(0);
        public static final double cameraYawOffset = Units.degreesToRadians(0);
        public static final double backRightCameraHeight = Units.inchesToMeters(9.1);
        public static final double backRightCameraPitch = Units.degreesToRadians(30);

        public static final double centerCameraHeight = Units.inchesToMeters(10.15);
        public static final double centerCameraPitch = Units.degreesToRadians(15);


        public static final class VisionLimits {

        public static final int k_rotationLimitDPS = 175;
        public static final double k_reprojectionLimit = 0.1;
        public static final double k_normThreshold = 0.1;
        public static final double k_ambiguityLimit = 0.9;

        }

        public static final class AprilTags {
            public static final int blueSourceRight = 1;
            public static final int blueSourceLeft = 2;
            public static final int redSpeakerOffset = 3;
            public static final int redSpeakerCenter = 4;
            public static final int redAmp = 5;
            public static final int blueAmp = 6;
            public static final int blueSpeakerCenter = 7;
            public static final int blueSpeakerOffset = 8;
            public static final int redSourceRight = 9;
            public static final int redSourceLeft = 10;
            public static final int redStageLeft = 11;
            public static final int redStageRight = 12;
            public static final int redStageCenter = 13;
            public static final int blueStageCenter = 14;
            public static final int blueStageLeft = 15;
            public static final int blueStageRight = 16;
        }
    }


    // hardware ports for all hardware components on the robot
    // these include CAN IDs, pneumatic hub ports, etc.

    public static final class robotPIDs {
        public static final double shooterkP = 0.5;
        public static final double shooterkI = 0.00;
        public static final double shooterkD = 0.00;

        public static final double pivotkP = 0.5;
        public static final double pivotkI = 0.00;
        public static final double pivotkD = 0.00;

        public static final class HeadingControlPID {
            public static final double highP = 12;
            public static final double highI = 0;
            public static final double highD = 4;

            public static final double lowP = 7;
            public static final double lowI = 0;
            public static final double lowD = 1.5;
        }

    }

    public static final class HardwarePorts {
        // motors (predicted) IDs not fixed

        public static final int shooterBottomM = 50;
        public static final int shooterTopM = 51;

        public static final int indexerTopM = 30;
        public static final int indexerBottomM = 31;
        
        public static final int intakeLeaderM = 21;
        public static final int intakeFollowerM = 20;
        public static final int serialM = 22;

        public static final int climbLeaderMotor = 60;
        public static final int climbFollowerMotor = 61;

        public static final int pivotLeaderM = 40;
        public static final int pivotFollowerM = 41;
        public static final int pivotCANcoderID = 42;

        public static final int ampM = 62;
        
    }



    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    enum AlignmentTargets {

            // BLUE_SPEAKER(new Point(Vision.getApriltagPose(2).get().getX(),5.55)),
            RED_SPEAKER(new Point(16.06, 5.55)),
            BLUE_AMP(new Point(1.815, 8.3)),
            RED_AMP(new Point(14.715, 8.3));
            
            private Point point;

            public Point getValue(){return point;}

            AlignmentTargets(Point point) {this.point = point;}

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
    public static final int shooterPeakCurrentLimit = 120;
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
        SmartDashboard.putNumber("angle from table", ShootingLookupTable.PivotAngleMap.get(distance));
        return ShootingLookupTable.PivotAngleMap.get(distance);
    }

    public static final class ShootingLookupTable {
        public static InterpolatingDoubleTreeMap FlywheelVelocitiesMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap PivotAngleMap = new InterpolatingDoubleTreeMap();

        public static final double[][] velocitiesMatrix = { //TODO: populate these with real measured values
            {Units.inchesToMeters(265), 50}, //ADD .1 M AFTER MOUNT IS CENTERED
            {2.67, 45}

        };

        public static final double[][] anglesMatrix = {
            //{Units.feetToMeters(265), 20},
            {2.379, 36.768}, //actual distance: 2.5908
            {2.53, 35.53},
            {2.715, 32.50}, //actual distance: 2.921
            {3.067, 32.285},

            {3.34, 30.79}, //actual distance: 3.4544
            // {3.625,31.4},
            {3.76, 29.0},
            // {4.01, 27.1},
            {4.423, 25.19}, //actual distance: 4.3942
            {5.07, 25.08}, //actual distance: 4.99
            {5.73, 24.24}, //actual distance: 5.6388
            
            

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
