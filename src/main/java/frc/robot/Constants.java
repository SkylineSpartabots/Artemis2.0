// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static final double MaxSpeed = 6; // 6 meters per second desired top speed
  public static final double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity
  
  public static final int timeOutMs = 10;
  public static final double stickDeadband = 0.15;
  public static final double triggerDeadzone = 0.2;

  public static final class Vision {
    public static final int aprilTagMax = 16;
    public static final String cameraName = "Arducam_OV9281_USB_Camera";
    public static final double cameraHeight = 0; //fix
    public static final double aprilTagHeight = 0.122; //bottom of each april tag is 122cm above carpet
    public static final double cameraRollOffset = Units.degreesToRadians(0); 
    public static final double cameraPitchOffset = Units.degreesToRadians(0);
    public static final double cameraYawOffset = Units.degreesToRadians(0);
  }


  // hardware ports for all hardware components on the robot
  // these include CAN IDs, pneumatic hub ports, etc. 

  public static final class hardwarePIDs{
    public static final double shooterkP = 0.5;
    public static final double shooterkI = 0.00;
    public static final double shooterkD = 0.00;

    public static final double pivotkP = 0.5;
    public static final double pivotkI = 0.00;
    public static final double pivotkD = 0.00;
  }
  public static final class HardwarePorts {
    // motors (predicted) IDs not fixed

    public static final int shooterTopM = 3;
    public static final int shooterBottomM = 4;
    public static final int indexerTopM = 5;
    public static final int indexerBottomM = 6;
    public static final int intakeLeaderM = 2;
    public static final int intakeFollowerM = 1;
    public static final int climbLeaderMotor = 3;
    public static final int climbFollowerMotor = 4;
    public static final int pivotMotor = 30;
    public static final int pivotCANcoderID = 31;
    public static final int ampMotor = 0;
  }

    /* Mechanism Current Limits */
  public static final int intakeContinuousCurrentLimit = 30;
  public static final int intakePeakCurrentLimit = 70;
  public static final int shooterContinuousCurrentLimit = 30;
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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
