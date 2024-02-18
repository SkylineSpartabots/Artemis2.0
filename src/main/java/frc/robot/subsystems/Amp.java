// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants;

public class Amp extends SubsystemBase {
  private static Amp s_Amp;

    public static Amp getInstance(){
        if(s_Amp == null){
            s_Amp = new Amp();
        }
        return s_Amp;
    }
  private TalonSRX m_amp;

  public Amp() {
    m_amp = new TalonSRX(Constants.HardwarePorts.m_amp);
    m_amp.setNeutralMode(NeutralMode.Brake);
  }


  public void setVelocity(double velocity){
    m_amp.set(ControlMode.Velocity, velocity);
  }

  public void setPercentPower(double power){
    m_amp.set(ControlMode.PercentOutput, power); //input values in [-1, 1]
  }

  public void ejectNote(){
    setPercentPower(-0.05); //arbitrary number; should spin motor very slowly outwards to eject
  }

  public void intakeNote(){
    setPercentPower(0.1); //arbitrary number
  }

  public void holdNote(){
    setPercentPower(0);
  }

  @Override
  public void periodic() {
  }
}
