// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

  public static final class vision {
    public static final int aprilTagMax = 16;
    public static final String cameraName = "cam";
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
    public static final int shooterLeaderMotor = 22;
    public static final int shooterFollowerMotor = 23;
    public static final int intakeMotor = 24;
    public static final int climbLeaderMotor = 3;
    public static final int climbFollowerMotor = 4;
    public static final int pivotMotor = 30;
    public static final int pivotCANcoderID = 31;
    public static final int ampMotor = 5;
  }

    /* Mechanism Current Limits */
  public static final int intakeContinuousCurrentLimit = 30;
  public static final int intakePeakCurrentLimit = 70;
  public static final int shooterContinuousCurrentLimit = 30;
  public static final int shooterPeakCurrentLimit = 70;
  public static final int pivotContinuousCurrentLimit = 30;
  public static final int pivotPeakCurrentLimit = 60;
  public static final int climbContinuousCurrentLimit = 40; //TODO: arbitrary numbers, figure out later
  public static final int climbPeakCurrentLimit = 80; //find actual number later
  public static final int ampContinuousCurrentLimit = 10; //find actual number later
  public static final int ampPeakCurrentLimit = 20; //if this fails, blame SAKET for giving us an arbitrary number

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
