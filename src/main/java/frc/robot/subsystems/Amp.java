// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Amp extends SubsystemBase {
  public static Amp instance;

    public static Amp getInstance(){
        if(instance == null){
            instance = new Amp();
        }
        return instance;
    }

  public enum AmpState {

    ZERO(0),

    PUSH(5.8),

    DEPLOYED(9.7);

    private double pos;

    private AmpState(double position) {
      pos = position;
    }

    public double getPos() {
      return pos;
    }
  }
  
  private TalonFX ampM;
    
  public Amp() {
    ampM = new TalonFX(Constants.HardwarePorts.ampM);
    configMotor(ampM);
    ampM.setInverted(true);
    ampM.setNeutralMode(NeutralModeValue.Brake);
  }

  public void resetMotorEncoder() {
    ampM.setPosition(0);
  }

  public double getMotorCurrent() {
    return ampM.getStatorCurrent().getValueAsDouble();
  }

  public double getPosition(){
    return ampM.getPosition().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    ampM.setVoltage(voltage);
  }

  public void setSpeed(double speed) { //-1.0 to 1.0
    ampM.set(speed);
  }

  private void configMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();


        currentLimitsConfigs.SupplyCurrentLimit = Constants.ampContinuousCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimit = Constants.ampContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentThreshold = Constants.ampPeakCurrentLimit;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits = currentLimitsConfigs;
        motor.getConfigurator().apply(config);
    }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Amp position", getPosition());
  }
}
