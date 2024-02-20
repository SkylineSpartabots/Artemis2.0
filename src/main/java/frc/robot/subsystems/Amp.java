// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Amp extends SubsystemBase {
  private static Amp instance;

    public static Amp getInstance(){
        if(instance == null){
            instance = new Amp();
        }
        return instance;
    }

  private TalonSRX ampM;
    
  public Amp() {
    ampM = new TalonSRX(Constants.HardwarePorts.ampMotor);
    configMotor();
  }

  private void configMotor() {
    ampM.setNeutralMode(NeutralMode.Brake);
    ampM.configContinuousCurrentLimit(Constants.ampContinuousCurrentLimit);
    ampM.configPeakCurrentLimit(Constants.ampPeakCurrentLimit);
  }

  //TODO add configure amp motor method and pid if needed by mechanism

  public void setVelocity(double velocity){
    ampM.set(ControlMode.Velocity, velocity);
  }

  public void setPercentPower(double power){
    ampM.set(ControlMode.PercentOutput, power); //input values in [-1, 1]
  }

  public void setAction(AmpActions action) {
    ampM.set(ControlMode.PercentOutput, action.power);
  }

  public enum AmpActions {
    EJECT_NOTE(-0.05),
    INTAKE_NOTE(0.1),
    HOLD_NOTE(0);

    private double power;
    private AmpActions( double power) {

      this.power = power;
    }
  }

  @Override
  public void periodic() {
  }
}
