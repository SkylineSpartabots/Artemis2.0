// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EthansGlasses extends SubsystemBase {
  /** Creates a new EthansGlasses. */



  private CANSparkFlex intakeLeader;
  private CANSparkFlex intakeFollower;

  private static EthansGlasses instance;
  public static EthansGlasses getInstance() {
    if (instance == null)
        instance = new EthansGlasses();
    return instance;
  }
  
  public EthansGlasses() {
      intakeLeader = new CANSparkFlex(44, MotorType.kBrushless);
      intakeFollower = new CANSparkFlex(43, MotorType.kBrushless);


  }

  public void turnOn(double val) {
      intakeLeader.set(val);
      intakeFollower.set(val);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
