// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

  private TalonFX climbMotor;

  private RelativeEncoder motorEncoder;

  // Measured in motor rotations
  private int maxHeight = 300;
  private int setState;

  public Climb() {
    climbMotor = new TalonFX(Constants.HardwarePorts.climbFollowerMotor);
    configMotor(climbMotor, false);
    setState = 2;
  }


  private void configMotor(TalonFX motor, boolean inverted) {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    currentLimitsConfigs.SupplyCurrentLimit = Constants.climbContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentThreshold = Constants.climbPeakCurrentLimit;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits = currentLimitsConfigs;

    climbMotor.getConfigurator().apply(motorConfig);
  }
  
  public void setClimbSpeed(double speed){
    climbMotor.set(speed); //speed should be -1.0 to 1.0
  }

  public void setState(int state) {
    setState = state;
  }

  public double getPosition() {
    return motorEncoder.getPosition();
  }

  /**
   * Sets the desired voltage to the climb leader motor. 
   * @param voltage The desired voltage. 
   */
  public void setVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }
  public void setPercentOutput(double percent){
    climbMotor.set(percent);
  }

  public double getSetPoint() {
    return setState;
  }

  public double getMotorCurrent() {
    return climbMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Climb follower position", followEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
