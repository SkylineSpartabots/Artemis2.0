// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {
  private static Amp s_Amp;
  
  private CANSparkFlex mAmp;

  public static Amp getInstance() {
    if(s_Amp == null){
      s_Amp = new Amp();
    }
    return s_Amp;
  }

  public Amp() {
    mAmp = new CANSparkFlex(Constants.HardwarePorts.ampMotor, MotorType.kBrushless);
    configureMotor(mAmp);
  }

  public void configureMotor(CANSparkFlex motor){
    motor.setSmartCurrentLimit(Constants.ampPeakCurrentLimit); //arbitraryconstant, fix this later
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(12);
  }

  public void setAmpSpeed(double speed){
    mAmp.set(speed); //speed should be from -1.0 to 1.0
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
