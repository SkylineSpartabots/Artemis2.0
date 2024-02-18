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

  private CANSparkFlex m_leaderClimb;
  private CANSparkFlex m_followerClimb;
  
  public static Climb getInstance() {
    if(s_Climb == null){
      s_Climb = new Climb();
  }
  return s_Climb;
  }

  public Climb() {
    m_leaderClimb = new CANSparkFlex(Constants.HardwarePorts.m_climbLeft, MotorType.kBrushless);
    configLeaderMotor(m_leaderClimb);

    m_followerClimb = new CANSparkFlex(Constants.HardwarePorts.m_climbRight, MotorType.kBrushless);
    configFollowerMotor(m_followerClimb, m_leaderClimb);
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
    m_leaderClimb.set(speed); //speed should be -1.0 to 1.0
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
