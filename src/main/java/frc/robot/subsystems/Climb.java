// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private static Climb s_Climb;

  private CANSparkFlex mLeaderClimb;
  private CANSparkFlex mFollowerClimb;
  
  public static Climb getInstance() {
    if(s_Climb == null){
      s_Climb = new Climb();
  }
  return s_Climb;
  }

  public Climb() {
    mLeaderClimb = new CANSparkFlex(Constants.HardwarePorts.climbLeaderMotor, MotorType.kBrushless);
    configLeaderMotor(mLeaderClimb);

    mFollowerClimb = new CANSparkFlex(Constants.HardwarePorts.climbFollowerMotor, MotorType.kBrushless);
    configFollowerMotor(mFollowerClimb, mLeaderClimb);
  }


  private void configLeaderMotor(CANSparkFlex motor) {
    motor.setSmartCurrentLimit(Constants.climbPeakCurrentLimit);
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(12);

  }

  
  private void configFollowerMotor(CANSparkFlex followerMotor, CANSparkFlex leaderMotor) {
    followerMotor.setSmartCurrentLimit(Constants.climbPeakCurrentLimit);
    followerMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.enableVoltageCompensation(12);

    followerMotor.follow(leaderMotor);
  }

  public void setClimbSpeed(double speed){
    mLeaderClimb.set(speed); //speed should be -1.0 to 1.0
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
