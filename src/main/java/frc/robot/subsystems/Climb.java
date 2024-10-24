// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private static Climb instance;
  
  public static Climb getInstance() {
    if(instance == null){
      instance = new Climb();
  }
    return instance;
  }

  private TalonFX m_climbLeader;
  private TalonFX m_climbFollower;

  private Follower follow = new Follower(Constants.HardwarePorts.climbLeaderMotor, false );

  public Climb() {
    m_climbFollower = new TalonFX(Constants.HardwarePorts.climbFollowerMotor);
    m_climbLeader = new TalonFX(Constants.HardwarePorts.climbLeaderMotor);

    m_climbFollower.setControl(follow);

    configMotor(m_climbFollower, true);
    configMotor(m_climbLeader, false);
  }

  private void configMotor(TalonFX motor, Boolean inverted) {
    motor.setInverted(inverted);
  }
  

  public enum ClimbStates {
    ON(0.6),
    OFF(0),
    REV(-0.8);
    
    private double speed;

    public double getValue() {
        return speed;
    }

    ClimbStates(double speed) {
        this.speed = speed;
    }
}

  public void setSpeed(double speed) {
    m_climbLeader.set(speed);
  }

    public void setSpeed(ClimbStates state) {
    m_climbLeader.set(state.getValue());
  }

  

}
